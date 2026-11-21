// MSP v1 framing for Betaflight: '$' 'M' '<' size cmd payload checksum, checksum = XOR of size, cmd, payload.
// Source: Betaflight 2025.12.5 msp/msp_serial.c (see docs/research.md section 2.2).
#pragma once

#include <cstdint>
#include <vector>

namespace msp {

const uint8_t kRc = 105;         // reply: uint16 per channel, order roll, pitch, YAW, THROTTLE, aux1...
const uint8_t kSetRawRc = 200;   // request: uint16 per channel, order roll, pitch, THROTTLE, YAW, aux1... (map AETR1234)

inline std::vector<uint8_t> request(uint8_t cmd, const std::vector<uint8_t>& payload = {}) {
    std::vector<uint8_t> out = {'$', 'M', '<', uint8_t(payload.size()), cmd};
    uint8_t checksum = uint8_t(payload.size()) ^ cmd;
    for (uint8_t b : payload) {
        out.push_back(b);
        checksum ^= b;
    }
    out.push_back(checksum);
    return out;
}

inline std::vector<uint8_t> setRawRc(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw) {
    std::vector<uint8_t> payload;
    // The FC uses only the first four; aux channels are sent as 1500 because the message carries 8.
    for (uint16_t v : {roll, pitch, throttle, yaw, uint16_t(1500), uint16_t(1500), uint16_t(1500), uint16_t(1500)}) {
        payload.push_back(uint8_t(v & 0xFF));
        payload.push_back(uint8_t(v >> 8));
    }
    return request(kSetRawRc, payload);
}

// Feed received bytes one at a time. Returns true when a complete, checksum-correct reply has arrived;
// then `cmd` and `payload` hold it until the next byte is fed.
struct Parser {
    uint8_t cmd = 0;
    std::vector<uint8_t> payload;

    bool feed(uint8_t byte) {
        switch (state_) {
            case 0: state_ = byte == '$' ? 1 : 0; return false;
            case 1: state_ = byte == 'M' ? 2 : 0; return false;
            case 2: state_ = byte == '>' ? 3 : 0; return false;   // '!' (error reply) is dropped here
            case 3: size_ = byte; checksum_ = byte; payload.clear(); state_ = 4; return false;
            case 4: cmd = byte; checksum_ ^= byte; state_ = size_ ? 5 : 6; return false;
            case 5:
                payload.push_back(byte);
                checksum_ ^= byte;
                if (payload.size() == size_) state_ = 6;
                return false;
            default:
                state_ = 0;
                return byte == checksum_;
        }
    }

private:
    int state_ = 0;
    uint8_t size_ = 0, checksum_ = 0;
};

inline uint16_t u16(const std::vector<uint8_t>& payload, size_t index) {
    return uint16_t(payload[2 * index] | (payload[2 * index + 1] << 8));
}

}  // namespace msp
