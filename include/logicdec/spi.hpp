// logicdec - spi.hpp
//
// SPI decoding for all four clock modes, with optional chip select.
//
// SPI has no framing of its own: the wire carries a clock and one bit per edge
// and nothing that says where a word begins. Word boundaries come from the
// configuration (word size, bit order) and transaction boundaries come from
// chip select. Decode with the wrong mode and you still get bytes out - they
// are just the wrong bytes, shifted by one bit. That is the failure this
// decoder is built to make visible rather than plausible.

#ifndef LOGICDEC_SPI_HPP
#define LOGICDEC_SPI_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "logicdec/annotation.hpp"
#include "logicdec/edges.hpp"
#include "logicdec/sample_stream.hpp"

namespace logicdec {

struct SpiConfig {
    int clk = 0;
    int mosi = 1;    // -1 if not captured
    int miso = -1;   // -1 if not captured
    int cs = -1;     // -1 for a free-running capture with no chip select

    int bits = 8;
    bool cpol = false;       // clock idle level
    bool cpha = false;       // false: sample on the leading edge
    bool msb_first = true;
    bool cs_active_low = true;

    // Which clock edge carries valid data. The four modes reduce to this one
    // line: sample on the rising edge when CPOL and CPHA agree.
    bool samples_on_rising() const { return cpol == cpha; }

    int mode() const { return (cpol ? 2 : 0) | (cpha ? 1 : 0); }
};

struct SpiWord {
    std::uint32_t mosi = 0;
    std::uint32_t miso = 0;
    SampleIndex start = 0;
    SampleIndex end = 0;
    std::size_t transaction = 0;
};

struct SpiResult {
    std::vector<SpiWord> words;
    std::vector<Annotation> annotations;
    std::size_t transactions = 0;

    bool clean() const { return is_clean(annotations); }

    std::vector<std::uint32_t> mosi_bytes() const {
        std::vector<std::uint32_t> out;
        out.reserve(words.size());
        for (const SpiWord& word : words) {
            out.push_back(word.mosi);
        }
        return out;
    }

    std::vector<std::uint32_t> miso_bytes() const {
        std::vector<std::uint32_t> out;
        out.reserve(words.size());
        for (const SpiWord& word : words) {
            out.push_back(word.miso);
        }
        return out;
    }
};

inline SpiResult decode_spi(const SampleStream& stream, const SpiConfig& config = {}) {
    SpiResult result;
    if (stream.size() < 2) {
        return result;
    }
    if (config.bits < 1 || config.bits > 32) {
        throw std::invalid_argument("SPI word size must be between 1 and 32 bits");
    }

    const ChannelView clk = stream.channel(config.clk);
    const bool has_mosi = config.mosi >= 0;
    const bool has_miso = config.miso >= 0;
    if (!has_mosi && !has_miso) {
        throw std::invalid_argument("SPI decoding needs at least one data channel");
    }

    const ChannelView mosi = stream.channel(has_mosi ? config.mosi : config.clk);
    const ChannelView miso = stream.channel(has_miso ? config.miso : config.clk);
    const bool has_cs = config.cs >= 0;
    const ChannelView cs = stream.channel(has_cs ? config.cs : config.clk);

    auto selected = [&](SampleIndex i) {
        if (!has_cs) {
            return true;
        }
        return cs[i] != config.cs_active_low;
    };

    const bool sample_rising = config.samples_on_rising();

    std::uint32_t mosi_shift = 0;
    std::uint32_t miso_shift = 0;
    int bit_count = 0;
    SampleIndex word_start = 0;

    auto reset_word = [&]() {
        mosi_shift = 0;
        miso_shift = 0;
        bit_count = 0;
    };

    auto flush_partial = [&](SampleIndex at) {
        if (bit_count == 0) {
            return;
        }
        // Bits that never completed a word. Almost always a mode or word-size
        // mismatch, occasionally a capture that starts mid-transaction.
        result.annotations.push_back(
            {word_start, at, AnnotationKind::Warning, static_cast<std::uint32_t>(bit_count),
             std::to_string(bit_count) + " leftover bit(s)"});
        reset_word();
    };

    bool previous_clk = clk[0];
    bool previous_selected = selected(0);
    if (has_cs && previous_selected) {
        result.transactions = 1;
        result.annotations.push_back({0, 1, AnnotationKind::Start, 0, "CS asserted"});
    } else if (!has_cs) {
        result.transactions = 1;
    }

    for (SampleIndex i = 1; i < stream.size(); ++i) {
        const bool current_selected = selected(i);
        if (has_cs && current_selected != previous_selected) {
            if (current_selected) {
                reset_word();
                ++result.transactions;
                word_start = i;
                result.annotations.push_back({i, i + 1, AnnotationKind::Start, 0, "CS asserted"});
            } else {
                flush_partial(i);
                result.annotations.push_back({i, i + 1, AnnotationKind::Stop, 0, "CS released"});
            }
            previous_selected = current_selected;
            previous_clk = clk[i];
            continue;
        }
        previous_selected = current_selected;

        const bool current_clk = clk[i];
        const bool edge = current_clk != previous_clk;
        previous_clk = current_clk;

        if (!edge || !current_selected) {
            continue;
        }
        if (current_clk != sample_rising) {
            continue;  // the other edge of the pair; data is not valid here
        }

        if (bit_count == 0) {
            word_start = i;
        }

        const std::uint32_t mosi_bit = has_mosi && mosi[i] ? 1u : 0u;
        const std::uint32_t miso_bit = has_miso && miso[i] ? 1u : 0u;
        if (config.msb_first) {
            mosi_shift = (mosi_shift << 1) | mosi_bit;
            miso_shift = (miso_shift << 1) | miso_bit;
        } else {
            mosi_shift |= mosi_bit << bit_count;
            miso_shift |= miso_bit << bit_count;
        }

        if (++bit_count == config.bits) {
            SpiWord word;
            word.mosi = mosi_shift;
            word.miso = miso_shift;
            word.start = word_start;
            word.end = i + 1;
            word.transaction = result.transactions == 0 ? 0 : result.transactions - 1;
            result.words.push_back(word);

            const int digits = (config.bits + 3) / 4;
            std::string label = has_mosi ? to_hex(word.mosi, digits) : std::string("-");
            if (has_miso) {
                label += " / " + to_hex(word.miso, digits);
            }
            result.annotations.push_back({word.start, word.end, AnnotationKind::Data, word.mosi, label});
            reset_word();
        }
    }

    flush_partial(stream.size());
    return result;
}

}  // namespace logicdec

#endif  // LOGICDEC_SPI_HPP
