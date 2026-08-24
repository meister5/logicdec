// logicdec - annotation.hpp
//
// The common output vocabulary. Every decoder produces a flat, time-ordered
// list of annotations spanning sample ranges, which is what a logic analyzer
// draws above the waveform and what a CLI prints as a transcript.
//
// Protocol-specific structs (I2cTransfer, SpiWord, UartFrame) sit alongside
// these for programmatic use; annotations are the shared view.

#ifndef LOGICDEC_ANNOTATION_HPP
#define LOGICDEC_ANNOTATION_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "logicdec/sample_stream.hpp"

namespace logicdec {

enum class AnnotationKind {
    Start,           // I2C START, UART start bit, SPI chip-select assert
    RepeatedStart,   // I2C Sr
    Stop,            // bus release / chip-select deassert / UART stop bit
    Address,         // I2C slave address + direction
    Data,            // a decoded byte or word
    Ack,
    Nack,
    Parity,
    Error,           // framing error, truncated transfer, unexpected condition
    Warning,         // decoded, but something was off
};

inline const char* to_string(AnnotationKind kind) {
    switch (kind) {
        case AnnotationKind::Start:         return "start";
        case AnnotationKind::RepeatedStart: return "repeated-start";
        case AnnotationKind::Stop:          return "stop";
        case AnnotationKind::Address:       return "address";
        case AnnotationKind::Data:          return "data";
        case AnnotationKind::Ack:           return "ack";
        case AnnotationKind::Nack:          return "nack";
        case AnnotationKind::Parity:        return "parity";
        case AnnotationKind::Error:         return "error";
        case AnnotationKind::Warning:       return "warning";
    }
    return "unknown";
}

struct Annotation {
    SampleIndex start = 0;   // first sample covered, inclusive
    SampleIndex end = 0;     // last sample covered, exclusive
    AnnotationKind kind = AnnotationKind::Data;
    std::uint32_t value = 0;  // decoded payload where one exists
    std::string label;        // human-readable form, e.g. "0x3C read"

    std::size_t width() const { return end > start ? end - start : 0; }
};

// True when a decode contains nothing the caller needs to look at by hand.
inline bool is_clean(const std::vector<Annotation>& annotations) {
    for (const Annotation& annotation : annotations) {
        if (annotation.kind == AnnotationKind::Error) {
            return false;
        }
    }
    return true;
}

inline std::size_t count_kind(const std::vector<Annotation>& annotations, AnnotationKind kind) {
    std::size_t total = 0;
    for (const Annotation& annotation : annotations) {
        if (annotation.kind == kind) {
            ++total;
        }
    }
    return total;
}

// Hex without pulling in <iomanip>, which would drag iostreams into every
// translation unit that includes a decoder.
inline std::string to_hex(std::uint32_t value, int digits = 2) {
    static const char* kDigits = "0123456789ABCDEF";
    std::string out(static_cast<std::size_t>(digits), '0');
    for (int i = digits - 1; i >= 0; --i) {
        out[static_cast<std::size_t>(i)] = kDigits[value & 0xFu];
        value >>= 4;
    }
    return "0x" + out;
}

}  // namespace logicdec

#endif  // LOGICDEC_ANNOTATION_HPP
