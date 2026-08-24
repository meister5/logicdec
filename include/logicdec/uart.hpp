// logicdec - uart.hpp
//
// Asynchronous serial decoding: start bit, data bits, optional parity, stop
// bit(s).
//
// There is no clock on the wire, so every frame is recovered from its own
// falling start edge and the configured baud rate. Bits are read at their
// centres rather than their edges, which is what gives the decode margin
// against the transmitter's clock being a few percent off - the same reason
// a real UART oversamples.

#ifndef LOGICDEC_UART_HPP
#define LOGICDEC_UART_HPP

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "logicdec/annotation.hpp"
#include "logicdec/edges.hpp"
#include "logicdec/sample_stream.hpp"

namespace logicdec {

enum class Parity { None, Even, Odd };

struct UartConfig {
    int channel = 0;
    double baud = 115200.0;
    int data_bits = 8;
    Parity parity = Parity::None;
    double stop_bits = 1.0;
    bool msb_first = false;   // almost always LSB first
    bool invert = false;      // true for a capture taken before an inverting driver
};

struct UartFrame {
    std::uint32_t value = 0;
    bool framing_error = false;  // the stop bit was not at the idle level
    bool parity_error = false;
    SampleIndex start = 0;
    SampleIndex end = 0;

    bool ok() const { return !framing_error && !parity_error; }
};

struct UartResult {
    std::vector<UartFrame> frames;
    std::vector<Annotation> annotations;

    bool clean() const { return is_clean(annotations); }

    // Frames as text, with non-printable bytes shown as '.'. Handy when the
    // point of the capture was to read a device's log output.
    std::string text() const {
        std::string out;
        out.reserve(frames.size());
        for (const UartFrame& frame : frames) {
            const std::uint32_t value = frame.value;
            out.push_back(value >= 0x20 && value < 0x7F ? static_cast<char>(value)
                                                        : (value == '\n' || value == '\r' || value == '\t'
                                                               ? static_cast<char>(value)
                                                               : '.'));
        }
        return out;
    }

    std::vector<std::uint32_t> values() const {
        std::vector<std::uint32_t> out;
        out.reserve(frames.size());
        for (const UartFrame& frame : frames) {
            out.push_back(frame.value);
        }
        return out;
    }
};

// Samples needed per bit at this rate. Below about three the bit centre cannot
// be placed reliably and the decode is guesswork, so it is refused.
inline double samples_per_bit(const SampleStream& stream, double baud) {
    if (baud <= 0.0) {
        throw std::invalid_argument("baud rate must be positive");
    }
    return stream.sample_rate() / baud;
}

inline UartResult decode_uart(const SampleStream& stream, const UartConfig& config = {}) {
    UartResult result;
    if (stream.size() < 2) {
        return result;
    }
    if (config.data_bits < 5 || config.data_bits > 9) {
        throw std::invalid_argument("UART data bits must be between 5 and 9");
    }
    if (config.stop_bits < 1.0) {
        throw std::invalid_argument("UART needs at least one stop bit");
    }

    const double bit = samples_per_bit(stream, config.baud);
    if (bit < 3.0) {
        throw std::invalid_argument(
            "sample rate is too low for this baud rate: at least 3 samples per bit are required");
    }

    const ChannelView view = stream.channel(config.channel);
    auto level = [&](SampleIndex i) { return config.invert ? !view[i] : view[i]; };
    auto at_offset = [&](SampleIndex origin, double bits) {
        return origin + static_cast<SampleIndex>(bits * bit + 0.5);
    };

    const int parity_bits = config.parity == Parity::None ? 0 : 1;
    const double frame_bits = 1.0 + config.data_bits + parity_bits + config.stop_bits;

    SampleIndex i = 0;
    while (i + 1 < stream.size()) {
        // Hunt for the falling edge that begins a frame.
        if (!(level(i) && !level(i + 1))) {
            ++i;
            continue;
        }

        const SampleIndex edge = i + 1;
        if (at_offset(edge, frame_bits - 0.5) >= stream.size()) {
            break;  // not enough capture left for a whole frame
        }

        // Confirm the start bit is still low at its centre. A shorter dip is a
        // glitch or a bus settling, and treating it as a frame would produce a
        // byte of noise plus a framing error further along.
        if (level(at_offset(edge, 0.5))) {
            i = edge;
            continue;
        }

        UartFrame frame;
        frame.start = edge;

        std::uint32_t value = 0;
        int ones = 0;
        for (int b = 0; b < config.data_bits; ++b) {
            const bool bit_value = level(at_offset(edge, 1.5 + b));
            if (bit_value) {
                ++ones;
                if (config.msb_first) {
                    value = (value << 1) | 1u;
                } else {
                    value |= 1u << b;
                }
            } else if (config.msb_first) {
                value <<= 1;
            }
        }
        frame.value = value;

        if (config.parity != Parity::None) {
            const bool parity_bit = level(at_offset(edge, 1.5 + config.data_bits));
            const int total = ones + (parity_bit ? 1 : 0);
            const bool even = (total % 2) == 0;
            frame.parity_error = (config.parity == Parity::Even) ? !even : even;
        }

        const bool stop = level(at_offset(edge, 1.5 + config.data_bits + parity_bits));
        frame.framing_error = !stop;
        frame.end = at_offset(edge, frame_bits);

        result.frames.push_back(frame);
        result.annotations.push_back(
            {frame.start, frame.end, AnnotationKind::Data, frame.value,
             to_hex(frame.value, (config.data_bits + 3) / 4)});
        if (frame.parity_error) {
            result.annotations.push_back(
                {frame.start, frame.end, AnnotationKind::Error, frame.value, "parity error"});
        }
        if (frame.framing_error) {
            result.annotations.push_back(
                {frame.start, frame.end, AnnotationKind::Error, frame.value, "framing error"});
            // The line was not idle where the stop bit should have been, so the
            // frame boundary is not trustworthy. Resynchronise from the next
            // idle-to-low transition instead of assuming this frame's length.
            i = frame.start + 1;
            continue;
        }

        i = at_offset(edge, 1.0 + config.data_bits + parity_bits);
    }

    return result;
}

// Guess the baud rate from the shortest pulse in the capture.
//
// The narrowest run of a level is one bit time, provided the traffic contains
// at least one isolated bit - which it does for anything but a long run of
// 0x00 or 0xFF. Returns 0 when the line never moves.
inline double estimate_baud(const SampleStream& stream, int channel) {
    const PulseWidths widths = measure_pulses(stream, channel);
    std::size_t shortest = 0;
    for (const std::vector<std::size_t>* group : {&widths.high, &widths.low}) {
        for (std::size_t width : *group) {
            if (width > 0 && (shortest == 0 || width < shortest)) {
                shortest = width;
            }
        }
    }
    if (shortest == 0) {
        return 0.0;
    }
    return stream.sample_rate() / static_cast<double>(shortest);
}

// Snap an estimate to the nearest standard rate, since a measured 114943 baud
// is really 115200 with a sampling error.
inline double nearest_standard_baud(double estimate) {
    static const double kStandard[] = {300,   600,   1200,   2400,   4800,   9600,
                                       19200, 38400, 57600,  74880,  115200, 230400,
                                       250000, 460800, 500000, 921600, 1000000, 2000000};
    if (estimate <= 0.0) {
        return 0.0;
    }
    double best = kStandard[0];
    double best_error = 1e18;
    for (double candidate : kStandard) {
        const double error = estimate > candidate ? estimate / candidate : candidate / estimate;
        if (error < best_error) {
            best_error = error;
            best = candidate;
        }
    }
    return best;
}

}  // namespace logicdec

#endif  // LOGICDEC_UART_HPP
