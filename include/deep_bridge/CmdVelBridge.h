/*
 * deep_bridge_node
 *
 * 订阅 cmd_vel（geometry_msgs::Twist），按固定频率通过云深处山猫 M20 的
 * UDP/JSON 本体监控协议下发速度指令（Type=2 Command=25，仅导航模式下生效）。
 * 跟 unitree_bridge 是同一个定位（cmd_vel -> 具体机型的 SDK/协议），只是
 * M20 没有官方 C++ SDK，协议是裸 UDP + JSON，所以这个包自己实现协议的组包/
 * 解包（见 UdpProtocol.h）和收发（本类）。
 *
 * 协议来源：《山猫M20 开发指南》-「软件开发指南」「运动控制（basic_server 协议）」。
 * 机器人本体是 UDP 服务端（默认 10.21.31.103:30000），这个节点是客户端。
 * 请求/响应都是异步的：大多数控制类请求手册都写"参考 1.3 节反馈信息判断是否
 * 执行成功"，也就是没有同步应答，得靠机器人持续上报的状态报文回读确认——
 * 而状态报文只发给"持续发心跳的 IP:端口"，所以心跳定时器必须先于其它任何
 * 启动步骤跑起来。
 */
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "deep_bridge/json.hpp"

namespace deep_bridge {

class CmdVelBridge {
public:
    CmdVelBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~CmdVelBridge();

    CmdVelBridge(const CmdVelBridge&) = delete;
    CmdVelBridge& operator=(const CmdVelBridge&) = delete;

private:
    // 机器人主动上报的"基础状态"（basic_server 协议 5.1，2Hz），是控制安全闸门和
    // 启动流程回读确认唯一的依据来源——协议里没有针对控制请求的同步应答。
    struct BasicStatus {
        // 运动状态（协议 1 节）：0空闲 1站立(临时过渡，自动跳转RL控制) 2软急停 3开机阻尼 4趴下 17 RL控制
        // 其中 17=RL控制 是唯一可执行移动控制和步态切换的状态
        int motion_state = -1;
        // 步态（协议 2 节）：0x1001=4097 基础(标准) 0x1003=4099 楼梯(标准) 0x3002=12290 平地(敏捷) 0x3003=12291 楼梯(敏捷)
        int gait = -1;
        int hes = -1;                 // Hard Emergency Stop：0未触发 1已触发
        int control_usage_mode = -1;  // 使用模式：0常规 1导航 2辅助；速度指令(Cmd=25)仅在导航模式下生效
        bool sleep = false;
        bool valid = false;
        ros::Time stamp;
    };

    // ---- 建立连接 / 后台接收 / 后台心跳 ----
    void openSocket();
    void receiveLoop();  // 后台线程：阻塞 recv（带超时以便退出），解析后分发给 handleAsdu
    void heartbeatLoop();  // 后台线程：按 heartbeat_rate_hz_ 持续发心跳，不依赖 ros::spin()
                            // 被调度——构造函数里 applyUsageMode/autoStandOnStart 等启动步骤全是
                            // 阻塞调用，此时 ros::spin() 还没跑起来，ros::Timer 不会自己触发，
                            // 必须用独立线程才能保证心跳在整个启动阶段不断（否则机器人会因为
                            // 收不到持续心跳而停止上报状态，导致 waitForFreshBasicStatus 超时）
    void handleAsdu(const std::string& asdu_json);
    void handleBasicStatus(const nlohmann::json& items);
    void handleAbnormalStatus(const nlohmann::json& items);
    bool waitForFreshBasicStatus(double timeout_sec, BasicStatus& out);

    // ---- 请求发送 ----
    uint16_t nextMsgId();
    bool sendRequest(int type, int command, const nlohmann::json& items);
    void sendSpeedCommand(double x, double y, double yaw, double z = 0.0, double roll = 0.0, double pitch = 0.0);

    // ---- 启动流程（构造函数里顺序调用，全部做完才建 cmd_vel 订阅者） ----
    void applyUsageMode();     // 确保处于导航模式（1101/5 Mode=1），速度指令(2/25)的前提条件
    void autoStandOnStart();   // 下发起立指令（2/22 MotionParam=1），自动进入 RL 控制(17)
    void applyGaitOnStart();   // 可选：额外请求步态（2/23），默认 0x3002 平地(敏捷)
    void logFinalStatus();

    // ---- ROS 回调 ----
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent&);

    int sock_fd_ = -1;

    std::thread recv_thread_;
    std::thread heartbeat_thread_;
    std::atomic<bool> running_{false};

    std::mutex status_mutex_;
    BasicStatus last_basic_status_;

    std::mutex msg_id_mutex_;
    uint16_t next_msg_id_ = 0;  // 手册 1.1.5：从 0 递增，65535 后回绕到 0

    std::mutex cmd_mutex_;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vyaw_ = 0.0;
    ros::Time last_cmd_time_;
    bool have_cmd_ = false;

    ros::Subscriber cmd_vel_sub_;
    ros::Timer control_timer_;

    // ---- 参数 ----
    std::string server_ip_ = "10.21.31.103";  // 手册 1.1.2 给出的默认 UDP 服务端地址
    int server_port_ = 30000;
    std::string cmd_vel_topic_ = "cmd_vel";

    double control_rate_hz_ = 20.0;  // 速度指令(2/25)发送频率，协议 4.5 建议不低于 20Hz
    double cmd_timeout_sec_ = 0.5;   // 超过这么久没收到新 cmd_vel 就发全零速度指令（安全看门狗；协议 4.5 机器人侧 500ms 超时保护）
    double heartbeat_rate_hz_ = 2.0; // 心跳频率，协议 5.1 要求先发心跳才收到状态上报

    double max_vx_ = 0.75;  // [m/s]，速度指令(2/25) X 的钳位上限：cmd_vel 的 linear.x 按 m/s 直接下发并夹到 ±max_vx_
    double max_vy_ = 0.6;   // [m/s]
    double max_vyaw_ = 1.0; // [rad/s]

    bool auto_stand_on_start_ = true;
    double stand_settle_sec_ = 5.0;
    int gait_on_start_ = 0x3002;  // 0x3002=12290 平地(敏捷)，见 CmdVelBridge.cpp applyGaitOnStart() 注释
    double mode_settle_sec_ = 1.0;

    bool set_usage_mode_on_start_ = true;
    double status_wait_timeout_sec_ = 3.0;
};

}  // namespace deep_bridge
