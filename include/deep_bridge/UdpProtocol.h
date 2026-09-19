/*
 * 山猫 M20S 本体监控协议的 APDU 组包/解包。
 *
 * 协议来源：《山猫M20S 开发资料》-「软件开发指南」V1.0.0 (2026-06-15) 1.1 节。
 * APDU = 16 字节协议头 + ASDU（这里固定用 JSON 格式，指南 1.1.6 建议优先用 JSON，
 * "处理性能更快且数据结构更全面，支持更多消息类型"）。
 *
 * 协议头结构（小端字节序，长度/报文ID都是低字节在前），见指南 1.1.5：
 *   [0..3]   同步字符 0xEB 0x91 0xEB 0x90，固定
 *   [4..5]   ASDU 字节长度
 *   [6..7]   报文 ID，请求方控制，响应帧原样回带
 *   [8]      ASDU 格式：0x00=XML 0x01=JSON
 *   [9]      当前包编号
 *   [10]     协议版本号，固定 0x01
 *   [11..15] 预留 5 字节，填 0
 *
 * 纯头文件、无状态，两端（发送/接收）共用。
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace deep_bridge {
namespace protocol {

constexpr size_t kHeaderSize = 16;
constexpr uint8_t kSync[4] = {0xEB, 0x91, 0xEB, 0x90};
constexpr uint8_t kFormatXml = 0x00;
constexpr uint8_t kFormatJson = 0x01;
constexpr uint8_t kProtocolVersion = 0x01;

// 指南 1.1.5 对"当前包编号"的说明是"从0起始，顺序递增，最大255"，但值一列写的是
// 0x00，且没说多包 APDU 怎么拆——这里发的每个 APDU 都是单包（ASDU 上限 65535 字节，
// 一个数据报装得下），所以固定填 0。请求/响应的对应关系本来就由报文 ID 负责。
constexpr uint8_t kSinglePacketIndex = 0x00;

// ---- ASDU 的 Type/Command 取值（指南 1.2 控制类 / 1.3 状态类）----
namespace msg {
// 1.2 控制类
constexpr int kTypeHeartbeat = 0x00100064;      // 1.2.1 心跳指令
constexpr int kCmdHeartbeat = 0x00000005;
constexpr int kTypeUsageMode = 0x00100002;      // 1.2.2 使用模式切换
constexpr int kCmdUsageMode = 0x00500002;
constexpr int kTypeMotion = 0x00100001;         // 1.2.3/1.2.4/1.2.5/1.2.6 共用同一个 Type
constexpr int kCmdMotionState = 0x00200002;     // 1.2.3 运动状态转换
constexpr int kCmdGait = 0x00300002;            // 1.2.4 运动步态切换
constexpr int kCmdAxisNormalized = 0x00100002;  // 1.2.5 轴指令，[-1,1] 归一化，仅常规/辅助模式
constexpr int kCmdAxisReal = 0x00110002;        // 1.2.6 真实轴指令，m/s 与 rad/s，仅导航模式

// 1.3 状态类：机器人主动上报，Command 统一是 0x00F00000
constexpr int kTypeBasicStatus = 0x00100064;    // 1.3.1.1 基础状态上报，2Hz
constexpr int kTypeAbnormalStatus = 0x0010007F; // 1.3.1.4 异常状态上报，2Hz
constexpr int kCmdStatusReport = 0x00F00000;

// 实测（2026-09-19，lubancat 上对真机）：本体上报的 Type 高半字是 0x0030，不是指南
// 1.3.1.1/1.3.1.4 写的 0x0010，低半字一致。只认指南的值会收不到任何状态上报，安全
// 闸门就永远打不开。两套都收：指南的值留着，万一别的固件版本按文档来。
// 注意控制类请求（1.2.x）用 0x0010 是对的——心跳发 0x00100064 本体确实会开始上报。
constexpr int kTypeBasicStatusAlt = 0x00300064;
constexpr int kTypeAbnormalStatusAlt = 0x0030007F;

inline bool isBasicStatusType(int type) {
    return type == kTypeBasicStatus || type == kTypeBasicStatusAlt;
}
inline bool isAbnormalStatusType(int type) {
    return type == kTypeAbnormalStatus || type == kTypeAbnormalStatusAlt;
}
}  // namespace msg

// 组一个完整 APDU：协议头 + ASDU JSON 字节。asdu_json 必须已经是最终要发送的
// JSON 字符串（调用方负责 dump），这里只管协议头和长度字段。
inline std::vector<uint8_t> encodeApdu(uint16_t msg_id, const std::string& asdu_json) {
    if (asdu_json.size() > 0xFFFFu) {
        // 指南 1.1.5：长度字段是 2 字节，ASDU 最大长度限制为 65535 字节
        return {};
    }

    std::vector<uint8_t> packet(kHeaderSize + asdu_json.size(), 0);
    std::memcpy(packet.data(), kSync, sizeof(kSync));

    const auto len = static_cast<uint16_t>(asdu_json.size());
    packet[4] = static_cast<uint8_t>(len & 0xFF);
    packet[5] = static_cast<uint8_t>((len >> 8) & 0xFF);
    packet[6] = static_cast<uint8_t>(msg_id & 0xFF);
    packet[7] = static_cast<uint8_t>((msg_id >> 8) & 0xFF);
    packet[8] = kFormatJson;
    packet[9] = kSinglePacketIndex;
    packet[10] = kProtocolVersion;
    // packet[11..15] 已经被 vector 的 0 初始化覆盖，预留字段不用再单独写

    std::memcpy(packet.data() + kHeaderSize, asdu_json.data(), asdu_json.size());
    return packet;
}

struct DecodedApdu {
    uint16_t msg_id = 0;
    uint8_t format = 0;
    std::string asdu_json;  // 原始 ASDU 字节，调用方自行 nlohmann::json::parse
};

// 解一个收到的 UDP 数据报。只做协议头校验（同步字符、声明长度是否越界），
// 不解析 ASDU 内容本身——ASDU 是否是合法 JSON 由调用方在 parse 时发现。
inline bool decodeApdu(const uint8_t* data, size_t len, DecodedApdu& out) {
    if (data == nullptr || len < kHeaderSize) {
        return false;
    }
    if (std::memcmp(data, kSync, sizeof(kSync)) != 0) {
        return false;
    }

    const auto asdu_len = static_cast<uint16_t>(data[4] | (static_cast<uint16_t>(data[5]) << 8));
    if (len < kHeaderSize + static_cast<size_t>(asdu_len)) {
        return false;  // 声明的 ASDU 长度比实际收到的数据报还长，丢弃
    }

    out.msg_id = static_cast<uint16_t>(data[6] | (static_cast<uint16_t>(data[7]) << 8));
    out.format = data[8];
    out.asdu_json.assign(reinterpret_cast<const char*>(data + kHeaderSize), asdu_len);
    return true;
}

}  // namespace protocol
}  // namespace deep_bridge
