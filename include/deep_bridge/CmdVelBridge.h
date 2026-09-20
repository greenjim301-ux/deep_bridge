/*
 * deep_bridge_node
 *
 * 订阅 cmd_vel（geometry_msgs::Twist），按固定频率通过云深处山猫 M20S 的
 * UDP/JSON 本体监控协议下发轴指令。跟 unitree_bridge 是同一个定位（cmd_vel ->
 * 具体机型的 SDK/协议），只是 M20S 没有官方 C++ SDK，协议是裸 UDP + JSON，
 * 所以这个包自己实现协议的组包/解包（见 UdpProtocol.h）和收发（本类）。
 *
 * 支持两种使用模式，由 usage_mode 参数选择。轴指令的种类必须和使用模式配对，
 * 所以只有这一个参数，不存在配错组合的可能：
 *   usage_mode=0 常规模式 -> 归一化轴指令（指南 1.2.5）：六个分量是 [-1,1] 的
 *                            "占最大速度的比例"，cmd_vel 的 m/s 需要用
 *                            full_scale_v* 换算，见下面那组参数。
 *   usage_mode=1 导航模式 -> 真实轴指令（指南 1.2.6）：字段相同但直接就是 m/s
 *                            与 rad/s，本体不再额外限速。
 *
 * 协议来源：《山猫M20S 开发资料》-「软件开发指南」V1.0.0 (2026-06-15)。
 * 机器人本体是 UDP 服务端，这个节点是客户端。
 * 两条确认途径，都要用上：
 *   - 指南 1.5 通用接口调用状态响应：每条请求都会回一条 ErrorCode/ErrorMessage，
 *     Type/Command 与请求原样相同，只说明"请求被不被接受"（见 handleGenericResponse）。
 *   - 指南 1.3 状态上报：控制类请求是否真的生效，仍要看机器人持续上报的状态报文
 *     （如运动状态有没有进到 RL 控制）。状态报文只发给"持续发心跳的 IP:端口"，
 *     所以心跳线程必须先于其它任何启动步骤跑起来。
 */
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "deep_bridge/UdpTransport.h"
#include "deep_bridge/json.hpp"

namespace deep_bridge {

class CmdVelBridge {
public:
    CmdVelBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~CmdVelBridge();

    CmdVelBridge(const CmdVelBridge&) = delete;
    CmdVelBridge& operator=(const CmdVelBridge&) = delete;

private:
    // 机器人主动上报的"基础状态"（指南 1.3.1.1，2Hz），是控制安全闸门和启动流程
    // 回读确认的依据——通用响应(1.5)只说明请求被接受，不说明物理状态到没到位。
    struct BasicStatus {
        // 运动状态（指南 1.3.1.1 MotionState）：-2 软急停 / 0 默认值(运动未上报) /
        // 1 站立(临时过渡，自动跳转 RL 控制) / 2 关节阻尼 / 3 开机阻尼 / 4 趴下 /
        // 5 标零 / 16 小车移动 / 17 RL控制 / 0x1001 阻尼趴下。
        // 其中 17=RL控制 是唯一可执行移动控制和步态切换的状态
        int motion_state = -1;
        // 步态（指南 1.3.1.1 Gait）：0x1001=4097 基础(标准) / 0x1002=4098 高台(标准) /
        // 0x3002=12290 平地(导航) / 0x3003=12291 楼梯(导航)。
        // 注：指南 1.2.4 的步态表把 0x1003 写成"楼梯(标准)"，与 1.3.1.1 的 0x1002
        // "高台(标准)"对不上；两处一致的只有导航运动模式的 0x3002/0x3003，也正是这里用的。
        int gait = -1;
        int hes = -1;                 // Hard Emergency Stop：0未触发 1已触发
        int control_usage_mode = -1;  // 使用模式：0常规 1导航 2辅助；安全闸门要求它等于 usage_mode_
        bool sleep = false;
        bool valid = false;
        ros::Time stamp;
    };

    // ---- 建立连接 / 后台接收 / 后台心跳 ----
    void openTransport();
    void receiveLoop();  // 后台线程：阻塞收（带超时以便退出），解析后分发给 handleAsdu
    void heartbeatLoop();  // 后台线程：按 heartbeat_rate_hz_ 持续发心跳，不依赖 ros::spin()
                            // 被调度——构造函数里 applyUsageMode/autoStandOnStart 等启动步骤全是
                            // 阻塞调用，此时 ros::spin() 还没跑起来，ros::Timer 不会自己触发，
                            // 必须用独立线程才能保证心跳在整个启动阶段不断（否则机器人会因为
                            // 收不到持续心跳而停止上报状态，导致 waitForFreshBasicStatus 超时）
    void handleAsdu(const std::string& asdu_json);
    void handleBasicStatus(const nlohmann::json& items);
    void handleAbnormalStatus(const nlohmann::json& items);
    void handleGenericResponse(int type, int command, const nlohmann::json& items);  // 指南 1.5
    bool waitForFreshBasicStatus(double timeout_sec, BasicStatus& out);

    // ---- 请求发送 ----
    uint16_t nextMsgId();
    bool sendRequest(int type, int command, const nlohmann::json& items);
    void sendSpeedCommand(double x, double y, double yaw, double z = 0.0, double roll = 0.0, double pitch = 0.0);

    // ---- 启动流程（构造函数里顺序调用，全部做完才建 cmd_vel 订阅者） ----
    void applyUsageMode();     // 确保处于 usage_mode_ 指定的使用模式（指南 1.2.2），轴指令生效的前提条件
    void autoStandOnStart();   // 下发起立指令（指南 1.2.3 MotionParam=1），自动进入 RL 控制(17)
    void applyGaitOnStart();   // 可选：额外请求步态（指南 1.2.4），默认 0x1001 基础(标准运动模式)
    void logFinalStatus();

    // ---- ROS 回调 ----
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent&);

    std::unique_ptr<UdpTransport> transport_;

    std::thread recv_thread_;
    std::thread heartbeat_thread_;
    std::atomic<bool> running_{false};

    std::mutex status_mutex_;
    BasicStatus last_basic_status_;

    std::mutex msg_id_mutex_;
    uint16_t next_msg_id_ = 0;  // 指南 1.1.5：从 0 递增，65535 后回绕到 0

    std::mutex cmd_mutex_;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vyaw_ = 0.0;
    ros::Time last_cmd_time_;
    bool have_cmd_ = false;

    ros::Subscriber cmd_vel_sub_;
    ros::Timer control_timer_;

    // ---- 参数 ----
    // 指南 1.1.2 的 DTLS 服务端（UDP + 加密）地址与端口，也是本体默认启用的那个。
    // 关掉加密（需联系技术支持改 robotserve 配置）之后要走明文 UDP，把 use_dtls
    // 设成 false 并改成对应的明文端口。
    std::string server_ip_ = "10.21.33.103";
    int server_port_ = 30004;
    bool use_dtls_ = true;
    std::string ca_file_;  // 空=只加密不校验服务端证书，见 UdpTransport.h 的说明
    double dtls_handshake_timeout_sec_ = 5.0;
    std::string cmd_vel_topic_ = "cmd_vel";

    double control_rate_hz_ = 20.0;  // 真实轴指令(指南 1.2.6)的发送频率
    double cmd_timeout_sec_ = 0.5;   // 超过这么久没收到新 cmd_vel 就发全零速度指令（安全看门狗）
    double heartbeat_rate_hz_ = 2.0; // 心跳频率，指南 1.2.1 要求不小于 1Hz，且先发心跳才会收到状态上报

    // 使用模式：0=常规（配归一化轴指令 1.2.5）/ 1=导航（配真实轴指令 1.2.6）。
    // 指南 1.2.2 还有 2=辅助模式（1.2.5 说它同样能执行归一化轴指令），这里不支持。
    int usage_mode_ = 0;

    // 安全限速，两种模式都生效：cmd_vel 先按 m/s / rad/s 夹到这里。
    // 常规模式下再除以 full_scale_v* 换算成比例；导航模式下就是最终下发值
    //（指南 1.2.6 明确本体不会再额外限速，这三个值就是唯一的限速）。
    double max_vx_ = 0.75;  // [m/s]
    double max_vy_ = 0.6;   // [m/s]
    double max_vyaw_ = 1.0; // [rad/s]

    // 归一化轴指令(指南 1.2.5)的满量程：轴指令里的 ±1.0 对应多大的实际速度。
    // 换算就是 ratio = v / full_scale_v，再夹到 ±1。
    // 只在 usage_mode=0 时用到；导航模式直接下发实际速度，不需要换算。
    // 取值是厂商《各个步态的有效速度范围》表（软件包 ≥V1.1.7）里区间的上界，
    // 默认对应 gait_on_start 默认的 0x1001 标准-基础步态。完整表见 config 注释。
    double full_scale_vx_ = 2.0;   // [m/s]
    double full_scale_vy_ = 1.0;   // [m/s]
    double full_scale_vyaw_ = 2.0; // [rad/s]

    bool auto_stand_on_start_ = true;
    double stand_settle_sec_ = 5.0;
    int gait_on_start_ = 0x1001;  // 0x1001=4097 基础(标准运动模式)，见 CmdVelBridge.cpp applyGaitOnStart() 注释
    double mode_settle_sec_ = 1.0;

    bool set_usage_mode_on_start_ = true;
    double status_wait_timeout_sec_ = 3.0;
};

}  // namespace deep_bridge
