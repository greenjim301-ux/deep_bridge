#include "deep_bridge/CmdVelBridge.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <stdexcept>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "deep_bridge/UdpProtocol.h"

namespace deep_bridge {

namespace {
double Clamp(double v, double max_v) {
    // max_v<=0 是配置错误，当成"没有配置限速"处理，输出 0 而不是原值
    if (max_v <= 1e-6) {
        return 0.0;
    }
    return std::max(-max_v, std::min(max_v, v));
}

std::string NowLocalTimeString() {
    // 手册 1.1.6：Time 字段格式 YYYY-MM-DD HH:MM:SS，本地时区
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
    pnh.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
    pnh.param("control_rate_hz", control_rate_hz_, control_rate_hz_);
    pnh.param("cmd_timeout_sec", cmd_timeout_sec_, cmd_timeout_sec_);
    pnh.param("heartbeat_rate_hz", heartbeat_rate_hz_, heartbeat_rate_hz_);
    pnh.param("max_vx", max_vx_, max_vx_);
    pnh.param("max_vy", max_vy_, max_vy_);
    pnh.param("max_vyaw", max_vyaw_, max_vyaw_);
    pnh.param("auto_stand_on_start", auto_stand_on_start_, auto_stand_on_start_);
    pnh.param("stand_settle_sec", stand_settle_sec_, stand_settle_sec_);
    pnh.param("gait_on_start", gait_on_start_, gait_on_start_);
    pnh.param("mode_settle_sec", mode_settle_sec_, mode_settle_sec_);
    pnh.param("set_usage_mode_on_start", set_usage_mode_on_start_, set_usage_mode_on_start_);
    pnh.param("status_wait_timeout_sec", status_wait_timeout_sec_, status_wait_timeout_sec_);

    openSocket();

    running_ = true;
    recv_thread_ = std::thread(&CmdVelBridge::receiveLoop, this);

    // 心跳必须最先起来：机器人只向持续发心跳的 IP:端口上报状态（手册 1.3 节），
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
                                             << " cmd_vel_topic=" << cmd_vel_topic_
                                             << " control_rate_hz=" << control_rate_hz_
                                             << " cmd_timeout_sec=" << cmd_timeout_sec_ << " max_vx=" << max_vx_
                                             << " max_vy=" << max_vy_ << " max_vyaw=" << max_vyaw_
                                             << " auto_stand_on_start=" << auto_stand_on_start_);
}

CmdVelBridge::~CmdVelBridge() {
    // 节点退出前主动发一次全零速度指令，不完全依赖控制超时看门狗
    if (sock_fd_ >= 0) {
        sendSpeedCommand(0.0, 0.0, 0.0);
    }

    running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void CmdVelBridge::openSocket() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error(std::string("deep_bridge: socket() failed: ") + std::strerror(errno));
    }

    // 接收超时给后台线程的阻塞 recv() 一个上限，配合 running_ 标志实现可中断的接收循环，
    // 避免用另一个线程 close() 同一个 fd 来"打断" recv() 这种依赖平台细节的做法
    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;  // 200ms
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<uint16_t>(server_port_));
    if (::inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error("deep_bridge: invalid server_ip: " + server_ip_);
    }

    // connect() 一个 UDP 套接字只是把目的地址固定下来，之后可以用 send()/recv()，
    // 内核也会丢弃来自其它地址的数据包——机器人只有一个地址，天然适合这么用
    if (::connect(sock_fd_, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error(std::string("deep_bridge: connect() failed: ") + std::strerror(errno));
    }
}

void CmdVelBridge::receiveLoop() {
    std::vector<uint8_t> buf(8192);
    while (running_.load()) {
        const ssize_t n = ::recv(sock_fd_, buf.data(), buf.size(), 0);
        if (n < 0) {
            // 超时（EAGAIN/EWOULDBLOCK）是正常的轮询节奏；其它错误打个警告但不退出循环，
            // 避免网络短暂抖动就把接收线程整个结束掉
            if (errno != EAGAIN && errno != EWOULDBLOCK && running_.load()) {
                ROS_WARN_THROTTLE(5.0, "[deep_bridge] recv() error: %s", std::strerror(errno));
            }
            continue;
        }
        if (n == 0) {
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

    // 目前只处理这个桥接节点安全运行需要的两类上报：基础状态（控制闸门/启动回读）
    // 和异常状态（日志可见性）。设备状态(1002/5)、运控状态(1002/4)、导航相关消息
    // 手册里都有定义，暂不需要，按需在这里加分支即可。
    if (type == 1002 && command == 6) {
        handleBasicStatus(items);
    } else if (type == 1002 && command == 3) {
        handleAbnormalStatus(items);
    } else {
        ROS_DEBUG_THROTTLE(10.0, "[deep_bridge] dropped ASDU Type=%d Command=%d", type, command);
    }
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
        status.motion_state = bs.at("MotionState").get<int>();
        status.gait = bs.at("Gait").get<int>();
        status.hes = bs.at("HES").get<int>();
        status.control_usage_mode = bs.at("ControlUsageMode").get<int>();
        status.sleep = bs.at("Sleep").get<int>();
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
        int component = 0;
        try {
            code = err.at("errorCode").get<int>();
            component = err.at("component").get<int>();
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(5.0, "[deep_bridge] failed to read ErrorList entry at %s:%d: %s | body=%s", __FILE__,
                               __LINE__, e.what(), err.dump().c_str());
            continue;
        }
        ROS_WARN_THROTTLE(2.0, "[deep_bridge] robot reported error 0x%04X on component bitmask 0x%X", code,
                           component);
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
    ++next_msg_id_;  // uint16_t 自然溢出回绕到 0，正好符合手册 1.1.5 的要求
    return id;
}

bool CmdVelBridge::sendRequest(int type, int command, const nlohmann::json& items) {
    if (sock_fd_ < 0) {
        return false;
    }

    const nlohmann::json asdu = {
        {"PatrolDevice",
         {{"Type", type}, {"Command", command}, {"Time", NowLocalTimeString()}, {"Items", items}}}};

    const std::string asdu_str = asdu.dump();
    const std::vector<uint8_t> packet = protocol::encodeApdu(nextMsgId(), asdu_str);
    if (packet.empty()) {
        ROS_WARN("[deep_bridge] ASDU too large to encode for Type=%d Command=%d (%zu bytes)", type, command,
                  asdu_str.size());
        return false;
    }

    const ssize_t sent = ::send(sock_fd_, packet.data(), packet.size(), 0);
    if (sent < 0 || static_cast<size_t>(sent) != packet.size()) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] send() failed for Type=%d Command=%d: %s", type, command,
                           std::strerror(errno));
        return false;
    }
    return true;
}

void CmdVelBridge::sendSpeedCommand(double x, double y, double yaw, double z, double roll, double pitch) {
    const nlohmann::json items = {{"X", x}, {"Y", y}, {"Z", z}, {"Roll", roll}, {"Pitch", pitch}, {"Yaw", yaw}};
    // Type=2 Command=25：运动控制-速度指令（basic_server 协议 4.5，仅导航模式下可使用）。
    // 与 Cmd=21 字段格式相同，但 X/Y/Z/Roll/Pitch/Yaw 传的是实际速度值（m/s、rad/s）；
    // 当前开放的四种移动步态仅有 X、Y、Yaw 三个参数项生效，Z/Roll/Pitch 填 0。
    sendRequest(2, 25, items);
}

void CmdVelBridge::applyUsageMode() {
    BasicStatus status;
    if (!waitForFreshBasicStatus(status_wait_timeout_sec_, status)) {
        ROS_WARN("[deep_bridge] timed out waiting for initial status report; sending usage-mode switch anyway");
    } else if (status.control_usage_mode == 1) {
        ROS_INFO("[deep_bridge] already in navigation usage mode, skip switch");
        return;
    }

    ROS_INFO("[deep_bridge] switching to navigation usage mode (Mode=1) ...");
    sendRequest(1101, 5, {{"Mode", 1}});  // Type=1101 Command=5：使用模式切换；Mode：0常规 1导航 2辅助
    if (mode_settle_sec_ > 0.0) {
        ros::Duration(mode_settle_sec_).sleep();
    }

    if (waitForFreshBasicStatus(status_wait_timeout_sec_, status) && status.control_usage_mode != 1) {
        ROS_WARN("[deep_bridge] usage mode readback mismatch after switch: got %d, expected 1 (navigation). Speed "
                 "commands (2/25) only take effect in navigation mode.",
                 status.control_usage_mode);
    }
}

void CmdVelBridge::autoStandOnStart() {
    ROS_WARN("[deep_bridge] auto_stand_on_start=true: make sure the robot has clear space before it stands up");
    ROS_INFO("[deep_bridge] requesting stand (MotionParam=1) ...");

    // basic_server 协议 4.2：运动状态转换（Cmd=22）可下发的 MotionParam 只有
    // 1=站立 / 2=软急停 / 4=趴下。站立(1) 是临时过渡状态，机器人会自动跳转到
    // RL 控制(17)——唯一可执行移动控制和步态切换的状态，所以这里只发站立指令，
    // 再用 BasicStatus(MotionState) 回读确认是否已经进入 17（协议 6 节快速启动流程）。
    sendRequest(2, 22, {{"MotionParam", 1}});  // Type=2 Command=22：运动状态转换，协议 4.2

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
    // Type=2 Command=23：步态切换（basic_server 协议 4.3）。
    // 仅在 RL 控制状态(17)且机器人静止时有效，所以必须等起立完成进入 17 之后再切。
    sendRequest(2, 23, {{"GaitParam", gait_on_start_}});
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
        sendRequest(100, 100, nlohmann::json::object());  // Type=100 Command=100：心跳指令，手册 1.2.1

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

    // 安全闸门：还没收到过状态上报 / 硬急停触发 / 不在导航模式 / 运动状态不是
    // 可行走的 RL 控制态（17）时，一律发全零速度指令，不管 cmd_vel
    // 是否还在正常到达——跟 unitree_bridge 的 cmd_timeout 看门狗同样的思路，
    // 只是这里多了几条闸门条件，因为 M20 的"能不能走"不只取决于有没有新指令。
    const bool safe_to_drive =
        status.valid && status.hes == 0 && status.control_usage_mode == 1 && status.motion_state == 17;

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

    // Cmd=25 传的是实际速度值（m/s、rad/s），直接下发 cmd_vel 并夹到配置的上限。
    // 参考协议 4.5 的步态有效速度范围（如 0x3002 平地/敏捷：X ±2.0 m/s、Y ±1.0 m/s、Yaw ±1.5 rad/s）。
    const double x = Clamp(vx, max_vx_);
    const double y = Clamp(vy, max_vy_);
    const double yaw = Clamp(vyaw, max_vyaw_);
    sendSpeedCommand(x, y, yaw);
}

}  // namespace deep_bridge
