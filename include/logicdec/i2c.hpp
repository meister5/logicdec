// logicdec - i2c.hpp
//
// I2C decoding from SCL/SDA logic levels.
//
// The whole protocol hangs off one rule: SDA is only allowed to change while
// SCL is low. A transition while SCL is high is therefore not data, it is a
// framing condition - falling is START, rising is STOP. Everything else is
// bits clocked in on SCL rising edges, eight at a time, each byte followed by
// one acknowledge bit driven by the receiver.

#ifndef LOGICDEC_I2C_HPP
#define LOGICDEC_I2C_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "logicdec/annotation.hpp"
#include "logicdec/sample_stream.hpp"

namespace logicdec {

struct I2cConfig {
    int scl = 0;
    int sda = 1;
};

struct I2cTransfer {
    std::uint8_t address = 0;      // 7-bit address, direction bit removed
    bool read = false;             // direction taken from the R/W bit
    bool address_acked = false;    // false means nothing answered at that address
    std::vector<std::uint8_t> data;
    std::vector<bool> acks;        // one per data byte, in order
    SampleIndex start = 0;
    SampleIndex end = 0;
    bool complete = false;         // ended on a STOP or repeated START, not on the capture running out

    std::size_t size() const { return data.size(); }
};

struct I2cResult {
    std::vector<Annotation> annotations;
    std::vector<I2cTransfer> transfers;

    bool clean() const { return is_clean(annotations); }
    std::size_t errors() const { return count_kind(annotations, AnnotationKind::Error); }
};

namespace detail {

inline std::string i2c_address_label(std::uint8_t address, bool read, bool acked) {
    std::string label = to_hex(address, 2) + (read ? " read" : " write");
    if (!acked) {
        label += " (NACK)";
    }
    return label;
}

}  // namespace detail

// Decode a capture into I2C transfers.
//
// Only 7-bit addressing is interpreted. A 10-bit address arrives as two bytes
// with a reserved 11110xx prefix; the bytes are still reported, they are simply
// not merged into one address.
inline I2cResult decode_i2c(const SampleStream& stream, const I2cConfig& config = {}) {
    I2cResult result;
    if (stream.size() < 2) {
        return result;
    }

    const ChannelView scl = stream.channel(config.scl);
    const ChannelView sda = stream.channel(config.sda);

    bool in_transfer = false;
    bool expecting_ack = false;
    bool address_byte = false;
    int bit_count = 0;
    std::uint32_t shift = 0;
    SampleIndex byte_start = 0;
    std::size_t address_annotation = 0;  // patched once the address ack is seen
    I2cTransfer transfer;

    auto finish_transfer = [&](SampleIndex end, bool complete) {
        if (!in_transfer) {
            return;
        }
        transfer.end = end;
        transfer.complete = complete;
        result.transfers.push_back(transfer);
        transfer = I2cTransfer{};
        in_transfer = false;
    };

    bool previous_scl = scl[0];
    bool previous_sda = sda[0];

    for (SampleIndex i = 1; i < stream.size(); ++i) {
        const bool current_scl = scl[i];
        const bool current_sda = sda[i];

        // Framing conditions: SDA moving while the clock is held high.
        if (previous_scl && current_scl && current_sda != previous_sda) {
            if (previous_sda && !current_sda) {
                const bool repeated = in_transfer;
                finish_transfer(i, /*complete=*/true);
                result.annotations.push_back(
                    {i, i + 1,
                     repeated ? AnnotationKind::RepeatedStart : AnnotationKind::Start,
                     0,
                     repeated ? "Sr" : "S"});
                in_transfer = true;
                transfer = I2cTransfer{};
                transfer.start = i;
                address_byte = true;
                expecting_ack = false;
                bit_count = 0;
                shift = 0;
            } else {
                result.annotations.push_back({i, i + 1, AnnotationKind::Stop, 0, "P"});
                finish_transfer(i, /*complete=*/true);
                expecting_ack = false;
                bit_count = 0;
                shift = 0;
            }
            previous_scl = current_scl;
            previous_sda = current_sda;
            continue;
        }

        // Data is valid on the rising clock edge.
        if (!previous_scl && current_scl && in_transfer) {
            if (expecting_ack) {
                const bool acked = !current_sda;  // the receiver pulls SDA low to acknowledge
                result.annotations.push_back(
                    {i, i + 1,
                     acked ? AnnotationKind::Ack : AnnotationKind::Nack,
                     static_cast<std::uint32_t>(acked ? 0 : 1),
                     acked ? "ACK" : "NACK"});
                if (address_byte) {
                    transfer.address_acked = acked;
                    address_byte = false;
                    // The address annotation was emitted before its ack was
                    // known; a device that never answers is the single most
                    // useful thing to see in a transcript, so relabel it now.
                    if (!acked && address_annotation < result.annotations.size()) {
                        result.annotations[address_annotation].label =
                            detail::i2c_address_label(transfer.address, transfer.read, false);
                    }
                } else {
                    transfer.acks.push_back(acked);
                }
                expecting_ack = false;
                bit_count = 0;
                shift = 0;
            } else {
                if (bit_count == 0) {
                    byte_start = i;
                }
                shift = (shift << 1) | (current_sda ? 1u : 0u);
                if (++bit_count == 8) {
                    const std::uint8_t byte = static_cast<std::uint8_t>(shift);
                    if (address_byte) {
                        transfer.address = static_cast<std::uint8_t>(byte >> 1);
                        transfer.read = (byte & 1u) != 0;
                        address_annotation = result.annotations.size();
                        result.annotations.push_back(
                            {byte_start, i + 1, AnnotationKind::Address, transfer.address,
                             detail::i2c_address_label(transfer.address, transfer.read, true)});
                    } else {
                        transfer.data.push_back(byte);
                        result.annotations.push_back(
                            {byte_start, i + 1, AnnotationKind::Data, byte, to_hex(byte, 2)});
                    }
                    expecting_ack = true;
                }
            }
        }

        previous_scl = current_scl;
        previous_sda = current_sda;
    }

    if (in_transfer) {
        // The capture ended mid-transfer. Reporting the partial transfer is
        // more useful than dropping it, but it must not look complete: a
        // truncated capture and a device that stopped clocking look identical
        // in the bytes alone.
        result.annotations.push_back(
            {stream.size() - 1, stream.size(), AnnotationKind::Error, 0,
             "capture ended mid-transfer"});
        finish_transfer(stream.size(), /*complete=*/false);
    }

    return result;
}

// A one-line transcript per transfer, in the shape a bus capture is usually
// read: address, direction, payload.
inline std::vector<std::string> summarize(const I2cResult& result) {
    std::vector<std::string> lines;
    lines.reserve(result.transfers.size());
    for (const I2cTransfer& transfer : result.transfers) {
        std::string line = to_hex(transfer.address, 2);
        line += transfer.read ? " R" : " W";
        if (!transfer.address_acked) {
            line += " [no ack]";
        }
        for (std::uint8_t byte : transfer.data) {
            line += " " + to_hex(byte, 2);
        }
        if (!transfer.complete) {
            line += " [truncated]";
        }
        lines.push_back(line);
    }
    return lines;
}

}  // namespace logicdec

#endif  // LOGICDEC_I2C_HPP
