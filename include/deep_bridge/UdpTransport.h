/*
 * 和山猫 M20S 本体通信的数据报传输层：明文 UDP 或 DTLS（UDP + 加密）。
 *
 * 指南 1.1.2 只写了一句话："DTLS 服务端（UDP + 加密）地址与端口为 10.21.33.103:30004，
 * TLS 服务端（TCP + 加密）地址与端口为 10.21.33.103:30003。默认情况下，端口通信启用加密；
 * 用户可通过修改 robotserve 配置文件关闭加密（需联系技术支持）"——除此之外全文再没有
 * 任何关于证书、CA、PSK 或加密套件的说明，所以这里：
 *   - 默认不校验服务端证书（机器人多半是自签证书，指南也没给可校验的 CA），
 *     构造时会打一条警告说明这一点；
 *   - 想校验就配 ca_file，配上之后按 SSL_VERIFY_PEER 严格校验。
 *
 * 线程安全：CmdVelBridge 里有三个线程会用到它（接收线程 receive、心跳线程和 ROS
 * 控制定时器 send）。明文 UDP 下内核天然支持并发收发；DTLS 下同一个 SSL 对象不允许
 * 并发读写，所以内部用一把锁串行化，并且阻塞等待放在锁外（先 poll 再取锁读），
 * 避免接收阻塞把 20Hz 的速度指令卡住。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <sys/types.h>

struct ssl_ctx_st;
struct ssl_st;

namespace deep_bridge {

class UdpTransport {
public:
    struct Options {
        std::string server_ip;
        int server_port = 0;
        bool use_dtls = true;
        std::string ca_file;                 // 空=不校验服务端证书
        double handshake_timeout_sec = 5.0;  // 仅 DTLS
    };

    explicit UdpTransport(const Options& options);  // 失败抛 std::runtime_error
    ~UdpTransport();

    UdpTransport(const UdpTransport&) = delete;
    UdpTransport& operator=(const UdpTransport&) = delete;

    bool send(const uint8_t* data, size_t len);

    // 返回 >0 收到的字节数；0 表示在 timeout_sec 内没有数据；<0 表示出错
    ssize_t receive(uint8_t* buf, size_t len, double timeout_sec);

private:
    void openSocket();
    void startDtls();
    bool waitReadable(double timeout_sec) const;  // poll()，不持锁

    // SSL_read 返回值的统一判定：>0 原样给出，WANT_READ/WANT_WRITE 当成"这次没有
    // 应用数据"(0)，其余打日志并返回 -1。两个 SSL_read 调用点共用，保证"传输层
    // 自己打日志"这个契约在 DTLS 侧没有漏网分支。调用方需持有 ssl_mutex_。
    ssize_t classifySslRead(int n);

    Options options_;
    int sock_fd_ = -1;

    // DTLS 状态。ssl_ctx_st/ssl_st 前置声明，避免把 openssl 头文件带进所有调用方。
    ssl_ctx_st* ssl_ctx_ = nullptr;
    ssl_st* ssl_ = nullptr;
    mutable std::mutex ssl_mutex_;
};

}  // namespace deep_bridge
