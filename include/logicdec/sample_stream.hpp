// logicdec - sample_stream.hpp
//
// The capture buffer every decoder reads from: a packed array of samples where
// each bit is one digital channel, plus the sample rate that turns indices into
// time.
//
// Storing samples packed rather than as one array per channel matters on the
// hardware this targets. A Cardputer or an ESP32 sampling four channels at
// 1 MS/s produces a megabyte a second; splitting that into per-channel vectors
// costs four times the RAM for no gain, since decoders read several channels at
// the same instant anyway.

#ifndef LOGICDEC_SAMPLE_STREAM_HPP
#define LOGICDEC_SAMPLE_STREAM_HPP

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace logicdec {

using Sample = std::uint8_t;
using SampleIndex = std::size_t;

constexpr int kMaxChannels = 8;

class SampleStream;

// A single channel, addressable by sample index. Cheap to copy; borrows the
// stream, so it must not outlive it.
class ChannelView {
public:
    ChannelView(const Sample* data, std::size_t size, int channel)
        : data_(data), size_(size), mask_(static_cast<Sample>(1u << channel)) {}

    bool operator[](SampleIndex index) const { return (data_[index] & mask_) != 0; }
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }

private:
    const Sample* data_;
    std::size_t size_;
    Sample mask_;
};

class SampleStream {
public:
    SampleStream() = default;

    SampleStream(std::vector<Sample> samples, double sample_rate_hz, int channel_count = kMaxChannels)
        : samples_(std::move(samples)), rate_(sample_rate_hz), channels_(channel_count) {
        if (rate_ <= 0.0) {
            throw std::invalid_argument("sample rate must be positive");
        }
        if (channels_ < 1 || channels_ > kMaxChannels) {
            throw std::invalid_argument("channel count must be between 1 and 8");
        }
    }

    std::size_t size() const { return samples_.size(); }
    bool empty() const { return samples_.empty(); }
    double sample_rate() const { return rate_; }
    int channel_count() const { return channels_; }
    const std::vector<Sample>& raw() const { return samples_; }

    // Seconds from the start of the capture to `index`.
    double time_at(SampleIndex index) const { return static_cast<double>(index) / rate_; }

    // Duration in seconds of `count` samples.
    double duration_of(std::size_t count) const { return static_cast<double>(count) / rate_; }

    double duration() const { return duration_of(samples_.size()); }

    // Number of samples in `seconds`, rounded to nearest.
    std::size_t samples_in(double seconds) const {
        return static_cast<std::size_t>(seconds * rate_ + 0.5);
    }

    bool at(int channel, SampleIndex index) const {
        check_channel(channel);
        if (index >= samples_.size()) {
            throw std::out_of_range("sample index out of range");
        }
        return (samples_[index] & (1u << channel)) != 0;
    }

    ChannelView channel(int channel) const {
        check_channel(channel);
        return ChannelView(samples_.data(), samples_.size(), channel);
    }

    // Build from one ASCII row per channel: "0011", "0101". '_' reads as 0 and
    // '-' as 1 so a waveform can be drawn readably in a test; spaces and '|'
    // are ignored, which makes long patterns groupable.
    //
    //   SampleStream::from_patterns({"__--__--", "_-_-_-_-"}, 1e6)
    static SampleStream from_patterns(const std::vector<std::string>& rows, double sample_rate_hz) {
        if (rows.empty()) {
            throw std::invalid_argument("at least one channel pattern is required");
        }
        if (rows.size() > static_cast<std::size_t>(kMaxChannels)) {
            throw std::invalid_argument("at most 8 channel patterns are supported");
        }

        std::vector<std::string> cleaned;
        cleaned.reserve(rows.size());
        for (const std::string& row : rows) {
            std::string out;
            out.reserve(row.size());
            for (char c : row) {
                switch (c) {
                    case '0':
                    case '_':
                        out.push_back('0');
                        break;
                    case '1':
                    case '-':
                        out.push_back('1');
                        break;
                    case ' ':
                    case '|':
                        break;
                    default:
                        throw std::invalid_argument(std::string("unexpected pattern character: ") + c);
                }
            }
            cleaned.push_back(std::move(out));
        }

        const std::size_t length = cleaned.front().size();
        for (const std::string& row : cleaned) {
            if (row.size() != length) {
                throw std::invalid_argument("all channel patterns must be the same length");
            }
        }

        std::vector<Sample> samples(length, 0);
        for (std::size_t channel = 0; channel < cleaned.size(); ++channel) {
            for (std::size_t i = 0; i < length; ++i) {
                if (cleaned[channel][i] == '1') {
                    samples[i] |= static_cast<Sample>(1u << channel);
                }
            }
        }

        return SampleStream(std::move(samples), sample_rate_hz, static_cast<int>(cleaned.size()));
    }

    // Build from per-channel logic levels.
    static SampleStream from_channels(const std::vector<std::vector<bool>>& channels,
                                      double sample_rate_hz) {
        if (channels.empty()) {
            throw std::invalid_argument("at least one channel is required");
        }
        if (channels.size() > static_cast<std::size_t>(kMaxChannels)) {
            throw std::invalid_argument("at most 8 channels are supported");
        }

        const std::size_t length = channels.front().size();
        for (const std::vector<bool>& channel : channels) {
            if (channel.size() != length) {
                throw std::invalid_argument("all channels must be the same length");
            }
        }

        std::vector<Sample> samples(length, 0);
        for (std::size_t channel = 0; channel < channels.size(); ++channel) {
            for (std::size_t i = 0; i < length; ++i) {
                if (channels[channel][i]) {
                    samples[i] |= static_cast<Sample>(1u << channel);
                }
            }
        }

        return SampleStream(std::move(samples), sample_rate_hz, static_cast<int>(channels.size()));
    }

    // Inverse of from_patterns, for failure messages and for eyeballing a capture.
    std::vector<std::string> to_patterns() const {
        std::vector<std::string> rows(static_cast<std::size_t>(channels_));
        for (int channel = 0; channel < channels_; ++channel) {
            std::string& row = rows[static_cast<std::size_t>(channel)];
            row.reserve(samples_.size());
            for (Sample sample : samples_) {
                row.push_back((sample & (1u << channel)) ? '-' : '_');
            }
        }
        return rows;
    }

    // A copy covering [first, last), preserving the sample rate. Used to hand a
    // single transaction to a decoder without copying the whole capture.
    SampleStream slice(SampleIndex first, SampleIndex last) const {
        if (first > last || last > samples_.size()) {
            throw std::out_of_range("slice bounds out of range");
        }
        return SampleStream(std::vector<Sample>(samples_.begin() + static_cast<std::ptrdiff_t>(first),
                                                samples_.begin() + static_cast<std::ptrdiff_t>(last)),
                            rate_, channels_);
    }

private:
    void check_channel(int channel) const {
        if (channel < 0 || channel >= channels_) {
            throw std::out_of_range("channel " + std::to_string(channel) + " is not in the capture");
        }
    }

    std::vector<Sample> samples_;
    double rate_ = 1.0;
    int channels_ = kMaxChannels;
};

}  // namespace logicdec

#endif  // LOGICDEC_SAMPLE_STREAM_HPP
