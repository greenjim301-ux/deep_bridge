#include "deep_bridge/CmdVelBridge.h"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <stdexcept>

#include "deep_bridge/UdpProtocol.h"

namespace deep_bridge {

namespace {

// 指南 1.2.2 的使用模式取值。只支持这两种：辅助模式(2) 虽然也能执行归一化轴指令，
// 但这个桥接节点用不到，与其留一条从来没走过的分支，不如明确不支持。
constexpr int kUsageModeRegular = 0;
constexpr int kUsageModeNavigation = 1;

const char* UsageModeName(int mode) {
    return mode == kUsageModeNavigation ? "navigation" : "regular";
}

double Clamp(double v, double max_v) {
    // max_v<=0 是配置错误，当成"没有配置限速"处理，输出 0 而不是原值
    if (max_v <= 1e-6) {
        return 0.0;
    }
    return std::max(-max_v, std::min(max_v, v));
}

// 实际速度 -> 指南 1.2.5 轴指令要求的 [-1,1] 归一化比例。
// full_scale 配错时返回 0 而不是除出一个巨大的比例——宁可不动，也不能因为配置
// 失误让机器人全速冲出去。
double ToRatio(double v, double full_scale) {
    if (full_scale <= 1e-6) {
        return 0.0;
    }
    return std::max(-1.0, std::min(1.0, v / full_scale));
}

std::string NowLocalTimeString() {
    // 指南 1.1.6：Time 字段格式 YYYY-MM-DD HH:MM:SS，本地时区
    const std::time_t now = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &local_tm);
    return std::string(buf);
}
}  // namespace

CmdVelBridge::CmdVelBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param("server_ip", server_ip_, server_ip_);
    pnh.param("server_port", server_port_, server_port_);
    pnh.param("use_dtls", use_dtls_, use_dtls_);
    pnh.param("ca_file", ca_file_, ca_file_);
    pnh.param("dtls_handshake_timeout_sec", dtls_handshake_timeout_sec_, dtls_handshake_timeout_sec_);
    pnh.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
    pnh.param("control_rate_hz", control_rate_hz_, control_rate_hz_);
    pnh.param("cmd_timeout_sec", cmd_timeout_sec_, cmd_timeout_sec_);
    pnh.param("heartbeat_rate_hz", heartbeat_rate_hz_, heartbeat_rate_hz_);
    pnh.param("usage_mode", usage_mode_, usage_mode_);
    pnh.param("max_vx", max_vx_, max_vx_);
    pnh.param("max_vy", max_vy_, max_vy_);
    pnh.param("max_vyaw", max_vyaw_, max_vyaw_);
    pnh.param("full_scale_vx", full_scale_vx_, full_scale_vx_);
    pnh.param("full_scale_vy", full_scale_vy_, full_scale_vy_);
    pnh.param("full_scale_vyaw", full_scale_vyaw_, full_scale_vyaw_);
    pnh.param("auto_stand_on_start", auto_stand_on_start_, auto_stand_on_start_);
    pnh.param("stand_settle_sec", stand_settle_sec_, stand_settle_sec_);
    pnh.param("gait_on_start", gait_on_start_, gait_on_start_);
    pnh.param("mode_settle_sec", mode_settle_sec_, mode_settle_sec_);
    pnh.param("set_usage_mode_on_start", set_usage_mode_on_start_, set_usage_mode_on_start_);
    pnh.param("status_wait_timeout_sec", status_wait_timeout_sec_, status_wait_timeout_sec_);

    if (usage_mode_ != kUsageModeRegular && usage_mode_ != kUsageModeNavigation) {
        ROS_WARN("[deep_bridge] unsupported usage_mode=%d, falling back to %d (regular)", usage_mode_,
                  kUsageModeRegular);
        usage_mode_ = kUsageModeRegular;
    }

    openTransport();

    running_ = true;
    recv_thread_ = std::thread(&CmdVelBridge::receiveLoop, this);

    // 心跳必须最先起来：机器人只向持续发心跳的 IP:端口上报状态（指南 1.3 节），
    // 下面所有启动步骤的回读确认都靠这份持续上报，没有心跳就永远等不到状态。
    // 用独立线程而不是 ros::Timer，是因为下面 applyUsageMode/autoStandOnStart 等启动步骤
    // 全是阻塞调用，此时 main.cpp 里的 ros::spin() 还没开始跑，ros::Timer 不会自己触发，
    // 只发一次心跳撑不了整个启动阶段（机器人这边没有持续收到心跳就会停止上报状态）。
    heartbeat_thread_ = std::thread(&CmdVelBridge::heartbeatLoop, this);

    if (set_usage_mode_on_start_) {
        applyUsageMode();
    }
    if (auto_stand_on_start_) {
        autoStandOnStart();
    }
    if (gait_on_start_ >= 0) {
        applyGaitOnStart();
    }
    logFinalStatus();

    // 启动流程走完之后才建订阅者/控制定时器，确保切换过程中不会有速度指令跟状态切换请求打架
    cmd_vel_sub_ = nh.subscribe(cmd_vel_topic_, 1, &CmdVelBridge::cmdVelCallback, this);
    control_timer_ =
        nh.createTimer(ros::Duration(1.0 / control_rate_hz_), &CmdVelBridge::controlTimerCallback, this);

    ROS_INFO_STREAM("[deep_bridge] server=" << server_ip_ << ":" << server_port_
                                             << (use_dtls_ ? " (DTLS)" : " (plain UDP)")
                                             << " usage_mode=" << usage_mode_ << " ("
                                             << UsageModeName(usage_mode_) << ", "
                                             << (usage_mode_ == kUsageModeNavigation ? "real axis cmd 1.2.6"
                                                                                     : "normalized axis cmd 1.2.5")
                                             << ")"
                                             << " cmd_vel_topic=" << cmd_vel_topic_
                                             << " control_rate_hz=" << control_rate_hz_
                                             << " cmd_timeout_sec=" << cmd_timeout_sec_ << " max_vx=" << max_vx_
                                             << " max_vy=" << max_vy_ << " max_vyaw=" << max_vyaw_
                                             << " full_scale=[" << full_scale_vx_ << "," << full_scale_vy_ << ","
                                             << full_scale_vyaw_ << "]"
                                             << " auto_stand_on_start=" << auto_stand_on_start_);
}

CmdVelBridge::~CmdVelBridge() {
    // 节点退出前主动发一次全零速度指令，不完全依赖控制超时看门狗
    if (transport_) {
        sendSpeedCommand(0.0, 0.0, 0.0);
    }

    running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    // 收发线程都停了才拆连接，否则 DTLS 的 SSL 对象会在别的线程用着的时候被释放
    transport_.reset();
}

void CmdVelBridge::openTransport() {
    UdpTransport::Options options;
    options.server_ip = server_ip_;
    options.server_port = server_port_;
    options.use_dtls = use_dtls_;
    options.ca_file = ca_file_;
    options.handshake_timeout_sec = dtls_handshake_timeout_sec_;
    transport_.reset(new UdpTransport(options));
}

void CmdVelBridge::receiveLoop() {
    std::vector<uint8_t> buf(8192);
    while (running_.load()) {
        // 0.2s 超时给这个循环一个可中断的节奏，配合 running_ 标志退出
        const ssize_t n = transport_->receive(buf.data(), buf.size(), 0.2);
        if (n <= 0) {
            // 0=超时，是正常的轮询节奏；<0 由 UdpTransport 自己打日志。两种都不退出
            // 循环，避免网络短暂抖动就把接收线程整个结束掉。
            continue;
        }

        protocol::DecodedApdu apdu;
        if (!protocol::decodeApdu(buf.data(), static_cast<size_t>(n), apdu)) {
            ROS_WARN_THROTTLE(5.0, "[deep_bridge] dropped malformed UDP packet (%zd bytes)", n);
            continue;
        }
        if (apdu.format != protocol::kFormatJson) {
            ROS_WARN_THROTTLE(5.0, "[deep_bridge] dropped non-JSON ASDU (format=%d)", apdu.format);
            continue;  // 只处理 JSON ASDU
        }
        handleAsdu(apdu.asdu_json);
    }
}

void CmdVelBridge::handleAsdu(const std::string& asdu_json) {
    // 排协议问题时把日志级别开到 DEBUG 就能看到每条原始 ASDU。用 INFO 会刷屏：
    // 状态上报 2Hz，加上每条速度指令都会回一条通用响应，20Hz 下一秒二十多条。
    ROS_DEBUG("[deep_bridge] received ASDU: %s", asdu_json.c_str());
    nlohmann::json root;
    try {
        root = nlohmann::json::parse(asdu_json);
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] failed to parse ASDU JSON at %s:%d: %s | body=%s", __FILE__, __LINE__,
                           e.what(), asdu_json.c_str());
        return;
    }

    int type = 0;
    int command = 0;
    nlohmann::json items = nlohmann::json::object();
    try {
        if (!root.contains("PatrolDevice")) {
            ROS_WARN_THROTTLE(5.0, "[deep_bridge] dropped ASDU without PatrolDevice at %s:%d", __FILE__, __LINE__);
            return;
        }
        const auto& pd = root["PatrolDevice"];
        if (!pd.contains("Type") || !pd.contains("Command")) {
            ROS_WARN_THROTTLE(5.0, "[deep_bridge] dropped ASDU with incomplete PatrolDevice at %s:%d", __FILE__, __LINE__);
            return;
        }

        type = pd["Type"].get<int>();
        command = pd["Command"].get<int>();
        items = pd.at("Items").get<nlohmann::json>();
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] failed to read ASDU fields at %s:%d: %s | body=%s", __FILE__, __LINE__,
                           e.what(), root.dump().c_str());
        return;
    }

    // 目前只处理这个桥接节点安全运行需要的三类报文：基础状态（控制闸门/启动回读）、
    // 异常状态（日志可见性），以及指南 1.5 的通用接口调用状态响应（请求是否被接受）。
    // 运控状态(1.3.1.2)、设备状态(1.3.1.3)、巡检类(1.4)指南里都有定义，暂不需要，
    // 按需在这里加分支即可。
    if (protocol::msg::isBasicStatusType(type) && command == protocol::msg::kCmdStatusReport) {
        handleBasicStatus(items);
    } else if (protocol::msg::isAbnormalStatusType(type) && command == protocol::msg::kCmdStatusReport) {
        handleAbnormalStatus(items);
    } else if (items.contains("ErrorCode")) {
        // 指南 1.5：通用响应的 Type/Command 与请求原样相同，没有自己的专用取值，
        // 只能靠 Items 里的 ErrorCode 认出来。
        handleGenericResponse(type, command, items);
    } else {
        ROS_DEBUG_THROTTLE(10.0, "[deep_bridge] dropped ASDU Type=0x%08X Command=0x%08X", type, command);
    }
}

void CmdVelBridge::handleGenericResponse(int type, int command, const nlohmann::json& items) {
    int error_code = 0;
    std::string error_message;
    try {
        error_code = items.at("ErrorCode").get<int>();
        error_message = items.value("ErrorMessage", std::string());
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] failed to read generic response at %s:%d: %s | body=%s", __FILE__,
                           __LINE__, e.what(), items.dump().c_str());
        return;
    }

    if (error_code == 0) {
        return;  // 成功的响应每条速度指令都会回一条，20Hz 下不能逐条打日志
    }
    ROS_WARN_THROTTLE(2.0, "[deep_bridge] request Type=0x%08X Command=0x%08X rejected: ErrorCode=0x%04X (%s)", type,
                       command, error_code, error_message.c_str());
}

void CmdVelBridge::handleBasicStatus(const nlohmann::json& items) {
    ROS_INFO("[deep_bridge] received BasicStatus ASDU: %s", items.dump().c_str());

    if (!items.contains("BasicStatus")) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] dropped BasicStatus ASDU without BasicStatus field at %s:%d", __FILE__, __LINE__);
        return;
    }
    const auto& bs = items["BasicStatus"];

    BasicStatus status;
    try {
        // 这四个是安全闸门的依据，缺一不可
        status.motion_state = bs.at("MotionState").get<int>();
        status.gait = bs.at("Gait").get<int>();
        status.hes = bs.at("HES").get<int>();
        status.control_usage_mode = bs.at("ControlUsageMode").get<int>();
        // Sleep 只用于日志。指南 1.3.1.1 写的是 bool，老固件上是 int，两种都收——
        // 为一个纯展示字段的类型差异丢掉整份状态，会让安全闸门一直关着、机器人不动。
        const auto& sleep_field = bs.at("Sleep");
        status.sleep = sleep_field.is_boolean() ? sleep_field.get<bool>() : sleep_field.get<int>() != 0;
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] failed to read BasicStatus fields at %s:%d: %s | body=%s", __FILE__,
                           __LINE__, e.what(), bs.dump().c_str());
        return;
    }
    status.valid = true;
    status.stamp = ros::Time::now();

    std::lock_guard<std::mutex> lock(status_mutex_);
    last_basic_status_ = status;
}

void CmdVelBridge::handleAbnormalStatus(const nlohmann::json& items) {
    if (!items.contains("ErrorList")) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] dropped AbnormalStatus ASDU without ErrorList field at %s:%d", __FILE__, __LINE__);
        return;
    }
    const auto& errors = items["ErrorList"];
    if (!errors.is_array()) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] dropped AbnormalStatus ASDU with non-array ErrorList at %s:%d", __FILE__, __LINE__);
        return;
    }
    for (const auto& err : errors) {
        int code = 0;
        int event_type = 0;
        std::string name;
        std::string resources;
        int severity = 0;
        try {
            // 指南 1.3.1.4：Code 故障编码，Name 故障名，Type 1=发生/等级变化 2=已消除，
            // Resources/Severities 是等长数组，逐个部件给出位置和严重等级(3 WARN/4 ERROR/5 FATAL)
            code = err.at("Code").get<int>();
            event_type = err.at("Type").get<int>();
            name = err.value("Name", std::string());
            if (err.contains("Severities")) {
                for (const auto& s : err["Severities"]) {
                    severity = std::max(severity, s.get<int>());
                }
            }
            if (err.contains("Resources")) {
                for (const auto& r : err["Resources"]) {
                    if (!resources.empty()) {
                        resources += ",";
                    }
                    resources += r.is_string() ? r.get<std::string>() : r.dump();
                }
            }
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(5.0, "[deep_bridge] failed to read ErrorList entry at %s:%d: %s | body=%s", __FILE__,
                               __LINE__, e.what(), err.dump().c_str());
            continue;
        }

        if (event_type == 2) {
            ROS_INFO_THROTTLE(2.0, "[deep_bridge] robot error cleared: 0x%04X %s [%s]", code, name.c_str(),
                               resources.c_str());
        } else {
            ROS_WARN_THROTTLE(2.0, "[deep_bridge] robot reported error 0x%04X %s severity=%d parts=[%s]", code,
                               name.c_str(), severity, resources.c_str());
        }
    }
}

bool CmdVelBridge::waitForFreshBasicStatus(double timeout_sec, BasicStatus& out) {
    {
        // 先丢掉旧帧，保证等到的是发完请求之后产生的状态
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_basic_status_.valid = false;
    }

    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
    while (ros::ok() && ros::Time::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (last_basic_status_.valid) {
                out = last_basic_status_;
                return true;
            }
        }
        ros::Duration(0.02).sleep();
    }
    return false;
}

uint16_t CmdVelBridge::nextMsgId() {
    std::lock_guard<std::mutex> lock(msg_id_mutex_);
    const uint16_t id = next_msg_id_;
    ++next_msg_id_;  // uint16_t 自然溢出回绕到 0，正好符合指南 1.1.5 的要求
    return id;
}

bool CmdVelBridge::sendRequest(int type, int command, const nlohmann::json& items) {
    if (!transport_) {
        return false;
    }

    const nlohmann::json asdu = {
        {"PatrolDevice",
         {{"Type", type}, {"Command", command}, {"Time", NowLocalTimeString()}, {"Items", items}}}};

    const std::string asdu_str = asdu.dump();
    const std::vector<uint8_t> packet = protocol::encodeApdu(nextMsgId(), asdu_str);
    if (packet.empty()) {
        ROS_WARN("[deep_bridge] ASDU too large to encode for Type=0x%08X Command=0x%08X (%zu bytes)", type, command,
                  asdu_str.size());
        return false;
    }

    if (!transport_->send(packet.data(), packet.size())) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] send failed for Type=0x%08X Command=0x%08X", type, command);
        return false;
    }
    return true;
}

void CmdVelBridge::sendSpeedCommand(double x, double y, double yaw, double z, double roll, double pitch) {
    const nlohmann::json items = {{"X", x}, {"Y", y}, {"Z", z}, {"Roll", roll}, {"Pitch", pitch}, {"Yaw", yaw}};
    // 轴指令的种类必须和使用模式配对，否则本体直接不认：
    //   常规模式 -> 指南 1.2.5 归一化轴指令，六个分量是 [-1,1] 的"占最大速度的比例"，
    //               m/s 的换算在 controlTimerCallback 里用 full_scale_v* 做；
    //   导航模式 -> 指南 1.2.6 真实轴指令，字段完全相同但值直接是 m/s 与 rad/s。
    // 指南 1.2.5 注：仅特殊步态下六轴全部有效，基础步态只响应 X、Y、Yaw，Z/Roll/Pitch 填 0。
    const int command = usage_mode_ == kUsageModeNavigation ? protocol::msg::kCmdAxisReal
                                                            : protocol::msg::kCmdAxisNormalized;
    sendRequest(protocol::msg::kTypeMotion, command, items);
}

void CmdVelBridge::applyUsageMode() {
    BasicStatus status;
    if (!waitForFreshBasicStatus(status_wait_timeout_sec_, status)) {
        ROS_WARN("[deep_bridge] timed out waiting for initial status report; sending usage-mode switch anyway");
    } else if (status.control_usage_mode == usage_mode_) {
        ROS_INFO("[deep_bridge] already in %s usage mode, skip switch", UsageModeName(usage_mode_));
        return;
    }

    ROS_INFO("[deep_bridge] switching to %s usage mode (Mode=%d) ...", UsageModeName(usage_mode_), usage_mode_);
    // 指南 1.2.2 使用模式切换；Mode：0 常规 / 1 导航 / 2 辅助
    sendRequest(protocol::msg::kTypeUsageMode, protocol::msg::kCmdUsageMode, {{"Mode", usage_mode_}});
    if (mode_settle_sec_ > 0.0) {
        ros::Duration(mode_settle_sec_).sleep();
    }

    if (waitForFreshBasicStatus(status_wait_timeout_sec_, status) && status.control_usage_mode != usage_mode_) {
        ROS_WARN("[deep_bridge] usage mode readback mismatch after switch: got %d, expected %d (%s). The axis "
                 "command this node sends only takes effect in that mode.",
                 status.control_usage_mode, usage_mode_, UsageModeName(usage_mode_));
    }
}

void CmdVelBridge::autoStandOnStart() {
    ROS_WARN("[deep_bridge] auto_stand_on_start=true: make sure the robot has clear space before it stands up");
    ROS_INFO("[deep_bridge] requesting stand (MotionParam=1) ...");

    // 指南 1.2.3 运动状态转换。MotionParam 可下发 1=站立 / 2=关节阻尼 / 3=开机阻尼 /
    // 4=趴下 / 5=标零 / 16=小车移动 / 17=RL控制 / 0x1001=阻尼趴下（空闲与软急停只支持
    // 查询，不支持下发）。站立(1) 是临时过渡状态，机器人会自动跳转到 RL 控制(17)——
    // 唯一可执行移动控制和步态切换的状态，所以这里只发站立指令，再用 BasicStatus
    // 的 MotionState 回读确认是否已经进入 17。
    sendRequest(protocol::msg::kTypeMotion, protocol::msg::kCmdMotionState, {{"MotionParam", 1}});

    if (stand_settle_sec_ > 0.0) {
        ROS_INFO_STREAM("[deep_bridge] waiting " << stand_settle_sec_ << "s for stand-up to physically settle...");
        ros::Duration(stand_settle_sec_).sleep();
    }

    BasicStatus status;
    const ros::Time deadline = ros::Time::now() + ros::Duration(status_wait_timeout_sec_);
    bool in_rl_control = false;
    while (ros::ok() && ros::Time::now() < deadline) {
        const double remaining = (deadline - ros::Time::now()).toSec();
        if (waitForFreshBasicStatus(std::max(0.0, remaining), status) && status.motion_state == 17) {
            in_rl_control = true;
            break;
        }
    }
    if (!in_rl_control) {
        ROS_WARN("[deep_bridge] timed out waiting for RL control state (17) after stand request (last motion_state=%d)",
                  status.motion_state);
    } else {
        ROS_INFO("[deep_bridge] motion state confirmed: RL control (17)");
    }
}

void CmdVelBridge::applyGaitOnStart() {
    ROS_INFO("[deep_bridge] requesting gait 0x%04X (%d) ...", gait_on_start_, gait_on_start_);
    // 指南 1.2.4 运动步态切换。仅在 RL 控制状态(17)且机器人静止时有效，所以必须等
    // 起立完成进入 17 之后再切。
    // 注意步态表里的"标准/导航运动模式"是指南 1.2.3 的运动模式，跟这里的使用模式
    // (常规/导航，指南 1.2.2)是两个维度——实测导航使用模式下用基础步态(0x1001)也能跑，
    // 所以 gait_on_start 不跟着 usage_mode 走，两者各自配置。
    sendRequest(protocol::msg::kTypeMotion, protocol::msg::kCmdGait, {{"GaitParam", gait_on_start_}});
    if (mode_settle_sec_ > 0.0) {
        ros::Duration(mode_settle_sec_).sleep();
    }

    BasicStatus status;
    if (waitForFreshBasicStatus(status_wait_timeout_sec_, status) && status.gait != gait_on_start_) {
        ROS_WARN("[deep_bridge] gait readback mismatch: got %d, requested %d", status.gait, gait_on_start_);
    }
}

void CmdVelBridge::logFinalStatus() {
    BasicStatus status;
    if (!waitForFreshBasicStatus(status_wait_timeout_sec_, status)) {
        ROS_WARN("[deep_bridge] ===== Final status: timed out, could not confirm =====");
        return;
    }
    ROS_INFO("[deep_bridge] ===== Final status: motion_state=%d gait=%d control_usage_mode=%d HES=%d sleep=%d "
              "=====",
              status.motion_state, status.gait, status.control_usage_mode, status.hes,
              static_cast<int>(status.sleep));
}

void CmdVelBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vyaw_ = msg->angular.z;
    last_cmd_time_ = ros::Time::now();
    have_cmd_ = true;
}

void CmdVelBridge::heartbeatLoop() {
    const double period_sec = heartbeat_rate_hz_ > 1e-6 ? 1.0 / heartbeat_rate_hz_ : 1.0;
    constexpr double kPollStep = 0.05;  // 小步 sleep，保证 running_=false 后能及时退出

    while (running_.load()) {
        // 指南 1.2.1 心跳指令，建议不小于 1Hz；机器人只向持续发心跳的 IP:端口上报状态
        sendRequest(protocol::msg::kTypeHeartbeat, protocol::msg::kCmdHeartbeat, nlohmann::json::object());

        double waited = 0.0;
        while (running_.load() && waited < period_sec) {
            const double step = std::min(kPollStep, period_sec - waited);
            ros::Duration(step).sleep();
            waited += step;
        }
    }
}

void CmdVelBridge::controlTimerCallback(const ros::TimerEvent&) {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    bool timed_out = true;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        vx = vx_;
        vy = vy_;
        vyaw = vyaw_;
        timed_out = !have_cmd_ || (ros::Time::now() - last_cmd_time_).toSec() > cmd_timeout_sec_;
    }

    BasicStatus status;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status = last_basic_status_;
    }

    // 安全闸门：还没收到过状态上报 / 硬急停触发 / 使用模式不是我们切过去的那个 /
    // 运动状态不是可行走的 RL 控制态（17）时，一律发全零速度指令，不管 cmd_vel
    // 是否还在正常到达——跟 unitree_bridge 的 cmd_timeout 看门狗同样的思路，
    // 只是这里多了几条闸门条件，因为 M20S 的"能不能走"不只取决于有没有新指令。
    // 使用模式必须严格等于 usage_mode_：一来轴指令只在配对的模式下生效，二来模式
    // 和预期不符本身就说明有别的东西在动机器人，此时停住比继续走安全。
    const bool safe_to_drive = status.valid && status.hes == 0 &&
                               status.control_usage_mode == usage_mode_ && status.motion_state == 17;

    if (timed_out || !safe_to_drive) {
        if (!safe_to_drive) {
            ROS_WARN_THROTTLE(5.0,
                               "[deep_bridge] not safe to drive (valid=%d HES=%d control_usage_mode=%d "
                               "motion_state=%d), sending zero speed command",
                               status.valid, status.hes, status.control_usage_mode, status.motion_state);
        }
        sendSpeedCommand(0.0, 0.0, 0.0);
        return;
    }

    // 两种模式共用的限幅：按安全限速夹住。指南 1.2.6 明确本体"不会做额外的速度限制"，
    // 所以这一步就是唯一的约束。
    const double vx_eff = Clamp(vx, max_vx_);
    const double vy_eff = Clamp(vy, max_vy_);
    const double vyaw_eff = Clamp(vyaw, max_vyaw_);

    if (usage_mode_ == kUsageModeNavigation) {
        // 真实轴指令直接下发实际速度
        sendSpeedCommand(vx_eff, vy_eff, vyaw_eff);
    } else {
        // 归一化轴指令要的是比例不是速度，再除以满量程换算
        sendSpeedCommand(ToRatio(vx_eff, full_scale_vx_), ToRatio(vy_eff, full_scale_vy_),
                         ToRatio(vyaw_eff, full_scale_vyaw_));
    }
}

}  // namespace deep_bridge
