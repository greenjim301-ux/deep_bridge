/*
 * 山猫 M20 本体监控协议的 APDU 组包/解包。
 *
 * 协议来源：《山猫 M20 系列软件接口手册(beta) V0.1.0》1.1.5/1.1.6 节。
 * APDU = 16 字节协议头 + ASDU（这里固定用 JSON 格式，手册建议优先用 JSON，
 * "处理性能更快且数据结构更全面，支持更多消息类型"）。
 *
 * 协议头结构（小端字节序，长度/报文ID都是低字节在前）：
 *   [0..3]  同步字符 0xEB 0x91 0xEB 0x90，固定
 *   [4..5]  ASDU 字节长度
 *   [6..7]  报文 ID，请求方控制，响应帧原样回带
 *   [8]     ASDU 格式：0x00=XML 0x01=JSON
 *   [9..15] 预留 7 字节，填 0
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

// 组一个完整 APDU：协议头 + ASDU JSON 字节。asdu_json 必须已经是最终要发送的
// JSON 字符串（调用方负责 dump），这里只管协议头和长度字段。
inline std::vector<uint8_t> encodeApdu(uint16_t msg_id, const std::string& asdu_json) {
    if (asdu_json.size() > 0xFFFFu) {
        // 手册 1.1.5：长度字段是 2 字节，ASDU 最大长度限制为 65535 字节
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
    // packet[9..15] 已经被 vector 的 0 初始化覆盖，预留字段不用再单独写

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
