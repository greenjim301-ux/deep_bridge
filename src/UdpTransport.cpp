#include "deep_bridge/UdpTransport.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/ssl.h>

#include <ros/ros.h>

namespace deep_bridge {

namespace {

// 单次阻塞读的上限。DTLS 下 SSL_read 是持锁调用的，这个值决定了万一它要处理一个
// 非应用数据记录（比如握手重传）而等不到后续报文时，最长会把发送方堵多久。
constexpr int kSocketRecvTimeoutMs = 100;

// 把 OpenSSL 错误队列里的内容全部取出来拼成一行，便于直接看出握手失败的原因
// （证书校验失败、对端要求客户端证书、根本没开加密等等各有不同的错误码）。
bool MakeSockAddr(const std::string& ip, int port, struct sockaddr_in& out) {
    std::memset(&out, 0, sizeof(out));
    out.sin_family = AF_INET;
    out.sin_port = htons(static_cast<uint16_t>(port));
    return ::inet_pton(AF_INET, ip.c_str(), &out.sin_addr) > 0;
}

std::string DrainOpenSslErrors() {
    std::string out;
    unsigned long err = 0;
    while ((err = ERR_get_error()) != 0) {
        char buf[256];
        ERR_error_string_n(err, buf, sizeof(buf));
        if (!out.empty()) {
            out += "; ";
        }
        out += buf;
    }
    return out.empty() ? std::string("no OpenSSL error detail") : out;
}

}  // namespace

UdpTransport::UdpTransport(const Options& options) : options_(options) {
    openSocket();
    if (options_.use_dtls) {
        try {
            startDtls();
        } catch (...) {
            if (sock_fd_ >= 0) {
                ::close(sock_fd_);
                sock_fd_ = -1;
            }
            throw;
        }
    }
}

UdpTransport::~UdpTransport() {
    if (ssl_ != nullptr) {
        // 发一条 close_notify 就走，不等对端回。对端不回也无所谓，这里正在退出。
        SSL_shutdown(ssl_);
        SSL_free(ssl_);  // 连带释放 SSL_set_bio 交给它的 BIO
        ssl_ = nullptr;
    }
    if (ssl_ctx_ != nullptr) {
        SSL_CTX_free(ssl_ctx_);
        ssl_ctx_ = nullptr;
    }
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

void UdpTransport::openSocket() {
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error(std::string("deep_bridge: socket() failed: ") + std::strerror(errno));
    }

    struct timeval tv {};
    tv.tv_sec = kSocketRecvTimeoutMs / 1000;
    tv.tv_usec = (kSocketRecvTimeoutMs % 1000) * 1000;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in server_addr {};
    if (!MakeSockAddr(options_.server_ip, options_.server_port, server_addr)) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error("deep_bridge: invalid server_ip: " + options_.server_ip);
    }

    // connect() 一个 UDP 套接字只是把目的地址固定下来，之后可以用 send()/recv()，
    // 内核也会丢弃来自其它地址的数据包——机器人只有一个地址，天然适合这么用。
    // DTLS 也需要这个：BIO_new_dgram 走的是 connected socket 语义。
    if (::connect(sock_fd_, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error(std::string("deep_bridge: connect() failed: ") + std::strerror(errno));
    }
}

void UdpTransport::startDtls() {
    ssl_ctx_ = SSL_CTX_new(DTLS_client_method());
    if (ssl_ctx_ == nullptr) {
        throw std::runtime_error("deep_bridge: SSL_CTX_new(DTLS_client_method) failed: " + DrainOpenSslErrors());
    }

    if (options_.ca_file.empty()) {
        // 指南没有给出可用于校验的 CA，机器人多半是自签证书。这里只加密不认证：
        // 能防窃听，但防不住中间人。配上 ca_file 就会走下面的严格校验分支。
        SSL_CTX_set_verify(ssl_ctx_, SSL_VERIFY_NONE, nullptr);
        ROS_WARN("[deep_bridge] DTLS enabled without a CA file: the robot's certificate is NOT verified "
                 "(encrypted but not authenticated). Set ~ca_file to enable verification.");
    } else {
        if (SSL_CTX_load_verify_locations(ssl_ctx_, options_.ca_file.c_str(), nullptr) != 1) {
            const std::string detail = DrainOpenSslErrors();
            SSL_CTX_free(ssl_ctx_);
            ssl_ctx_ = nullptr;
            throw std::runtime_error("deep_bridge: failed to load ca_file '" + options_.ca_file + "': " + detail);
        }
        SSL_CTX_set_verify(ssl_ctx_, SSL_VERIFY_PEER, nullptr);
    }

    ssl_ = SSL_new(ssl_ctx_);
    if (ssl_ == nullptr) {
        const std::string detail = DrainOpenSslErrors();
        SSL_CTX_free(ssl_ctx_);
        ssl_ctx_ = nullptr;
        throw std::runtime_error("deep_bridge: SSL_new failed: " + detail);
    }

    BIO* bio = BIO_new_dgram(sock_fd_, BIO_NOCLOSE);
    if (bio == nullptr) {
        const std::string detail = DrainOpenSslErrors();
        SSL_free(ssl_);
        ssl_ = nullptr;
        SSL_CTX_free(ssl_ctx_);
        ssl_ctx_ = nullptr;
        throw std::runtime_error("deep_bridge: BIO_new_dgram failed: " + detail);
    }
    // 必须显式告诉 dgram BIO 对端地址并标记成 connected。不标记的话 OpenSSL 的
    // dgram_write 走的是 sendto(&data->peer)，而 data->peer 是全零地址，握手第一个
    // 报文就发不出去，表现为 SSL_do_handshake 直接返回 SSL_ERROR_SYSCALL 且错误队列为空。
    struct sockaddr_in peer_addr {};
    MakeSockAddr(options_.server_ip, options_.server_port, peer_addr);  // openSocket 里已校验过
    BIO_ADDR* bio_addr = BIO_ADDR_new();
    if (bio_addr == nullptr ||
        BIO_ADDR_rawmake(bio_addr, AF_INET, &peer_addr.sin_addr, sizeof(peer_addr.sin_addr),
                         peer_addr.sin_port) != 1) {
        const std::string detail = DrainOpenSslErrors();
        BIO_ADDR_free(bio_addr);
        BIO_free(bio);
        SSL_free(ssl_);
        ssl_ = nullptr;
        SSL_CTX_free(ssl_ctx_);
        ssl_ctx_ = nullptr;
        throw std::runtime_error("deep_bridge: BIO_ADDR_rawmake failed: " + detail);
    }
    BIO_ctrl(bio, BIO_CTRL_DGRAM_SET_CONNECTED, 0, bio_addr);
    BIO_ADDR_free(bio_addr);

    SSL_set_bio(ssl_, bio, bio);
    SSL_set_connect_state(ssl_);

    ROS_INFO("[deep_bridge] starting DTLS handshake with %s:%d ...", options_.server_ip.c_str(),
             options_.server_port);

    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(std::max(0.5, options_.handshake_timeout_sec));
    while (true) {
        const int ret = SSL_do_handshake(ssl_);
        if (ret == 1) {
            break;
        }
        const int err = SSL_get_error(ssl_, ret);
        if (err != SSL_ERROR_WANT_READ && err != SSL_ERROR_WANT_WRITE) {
            const std::string detail = DrainOpenSslErrors();
            SSL_free(ssl_);
            ssl_ = nullptr;
            SSL_CTX_free(ssl_ctx_);
            ssl_ctx_ = nullptr;
            throw std::runtime_error("deep_bridge: DTLS handshake failed (SSL_get_error=" + std::to_string(err) +
                                     "): " + detail);
        }
        if (ros::WallTime::now() >= deadline) {
            SSL_free(ssl_);
            ssl_ = nullptr;
            SSL_CTX_free(ssl_ctx_);
            ssl_ctx_ = nullptr;
            throw std::runtime_error("deep_bridge: DTLS handshake timed out after " +
                                     std::to_string(options_.handshake_timeout_sec) + "s. Check that " +
                                     options_.server_ip + ":" + std::to_string(options_.server_port) +
                                     " is the DTLS port and that encryption is actually enabled on the robot.");
        }
        // 握手报文可能丢，超时了要重传本轮 flight
        DTLSv1_handle_timeout(ssl_);
    }

    ROS_INFO("[deep_bridge] DTLS handshake done, cipher=%s", SSL_get_cipher(ssl_));
}

bool UdpTransport::waitReadable(double timeout_sec) const {
    struct pollfd pfd {};
    pfd.fd = sock_fd_;
    pfd.events = POLLIN;
    const int timeout_ms = timeout_sec <= 0.0 ? 0 : static_cast<int>(timeout_sec * 1000.0);
    return ::poll(&pfd, 1, timeout_ms) > 0;
}

bool UdpTransport::send(const uint8_t* data, size_t len) {
    if (sock_fd_ < 0) {
        return false;
    }

    if (!options_.use_dtls) {
        const ssize_t sent = ::send(sock_fd_, data, len, 0);
        return sent >= 0 && static_cast<size_t>(sent) == len;
    }

    std::lock_guard<std::mutex> lock(ssl_mutex_);
    if (ssl_ == nullptr) {
        return false;
    }
    const int sent = SSL_write(ssl_, data, static_cast<int>(len));
    if (sent <= 0) {
        ROS_WARN_THROTTLE(5.0, "[deep_bridge] SSL_write failed (SSL_get_error=%d): %s",
                           SSL_get_error(ssl_, sent), DrainOpenSslErrors().c_str());
        return false;
    }
    return static_cast<size_t>(sent) == len;
}

ssize_t UdpTransport::receive(uint8_t* buf, size_t len, double timeout_sec) {
    if (sock_fd_ < 0) {
        return -1;
    }

    if (!options_.use_dtls) {
        const ssize_t n = ::recv(sock_fd_, buf, len, 0);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return 0;  // 超时，不是错误
        }
        return n;
    }

    // 先看 SSL 里有没有已经解密好、还没取走的应用数据
    {
        std::lock_guard<std::mutex> lock(ssl_mutex_);
        if (ssl_ != nullptr && SSL_pending(ssl_) > 0) {
            const int n = SSL_read(ssl_, buf, static_cast<int>(len));
            return n > 0 ? n : -1;
        }
    }

    // 阻塞等待放在锁外，否则接收会把心跳/速度指令的 SSL_write 卡住
    if (!waitReadable(timeout_sec)) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(ssl_mutex_);
    if (ssl_ == nullptr) {
        return -1;
    }
    const int n = SSL_read(ssl_, buf, static_cast<int>(len));
    if (n > 0) {
        return n;
    }
    const int err = SSL_get_error(ssl_, n);
    if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE) {
        // 刚才 poll 到的是握手重传之类的非应用数据记录，不是应用报文
        return 0;
    }
    ROS_WARN_THROTTLE(5.0, "[deep_bridge] SSL_read failed (SSL_get_error=%d): %s", err,
                       DrainOpenSslErrors().c_str());
    return -1;
}

}  // namespace deep_bridge
