// logicdec - synth.hpp
//
// Waveform generation: build a capture that a decoder should be able to read
// back exactly.
//
// This exists because the alternative way to test a decoder is to capture real
// traffic, hand-check it once, and freeze it as a fixture - which pins the
// tests to whatever the bus happened to be doing that day and makes the awkward
// cases (a NACK, a mode mismatch, a corrupted stop bit) nearly impossible to
// produce on demand. Round-tripping synth -> decode -> compare covers the
// parameter space instead, and the generators are useful on their own for
// exercising a device driver with no device attached.

#ifndef LOGICDEC_SYNTH_HPP
#define LOGICDEC_SYNTH_HPP

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "logicdec/i2c.hpp"
#include "logicdec/sample_stream.hpp"
#include "logicdec/spi.hpp"
#include "logicdec/uart.hpp"

namespace logicdec {
namespace synth {

// Holds a level per channel and appends runs of samples at those levels.
class Recorder {
public:
    explicit Recorder(int channel_count)
        : channels_(static_cast<std::size_t>(channel_count)),
          current_(static_cast<std::size_t>(channel_count), false) {
        if (channel_count < 1 || channel_count > kMaxChannels) {
            throw std::invalid_argument("channel count must be between 1 and 8");
        }
    }

    void set(int channel, bool level) {
        if (channel >= 0) {
            current_[static_cast<std::size_t>(channel)] = level;
        }
    }

    bool get(int channel) const {
        return channel >= 0 && current_[static_cast<std::size_t>(channel)];
    }

    void hold(std::size_t samples) {
        for (std::size_t channel = 0; channel < channels_.size(); ++channel) {
            channels_[channel].insert(channels_[channel].end(), samples, current_[channel]);
        }
    }

    std::size_t size() const { return channels_.empty() ? 0 : channels_.front().size(); }

    SampleStream build(double sample_rate) const {
        return SampleStream::from_channels(channels_, sample_rate);
    }

private:
    std::vector<std::vector<bool>> channels_;
    std::vector<bool> current_;
};

inline std::size_t divide_rate(double sample_rate, double signal_rate, int parts) {
    const double samples = sample_rate / signal_rate / parts;
    if (samples < 1.0) {
        throw std::invalid_argument("sample rate is too low to synthesise this signal rate");
    }
    return static_cast<std::size_t>(samples + 0.5);
}

// I2C waveform generator.
//
// Bit timing is quarter-period granular, which is enough to place the two
// things that define the protocol: data changing only while SCL is low, and
// the START/STOP transitions happening while SCL is high.
class I2c {
public:
    I2c(double sample_rate = 1e6, double clock_hz = 100e3, int scl = 0, int sda = 1)
        : rate_(sample_rate),
          quarter_(divide_rate(sample_rate, clock_hz, 4)),
          scl_(scl),
          sda_(sda),
          recorder_(std::max(scl, sda) + 1) {
        recorder_.set(scl_, true);
        recorder_.set(sda_, true);
        recorder_.hold(quarter_ * 4);
    }

    I2c& start() {
        level(true, true, quarter_ * 2);
        level(true, false, quarter_ * 2);  // SDA falls with SCL high
        level(false, false, quarter_);
        return *this;
    }

    I2c& repeated_start() {
        level(false, true, quarter_);      // release SDA while the clock is low
        level(true, true, quarter_ * 2);
        level(true, false, quarter_ * 2);  // and drop it again with SCL high
        level(false, false, quarter_);
        return *this;
    }

    I2c& stop() {
        level(false, false, quarter_);
        level(true, false, quarter_ * 2);
        level(true, true, quarter_ * 2);   // SDA rises with SCL high
        level(true, true, quarter_ * 4);
        return *this;
    }

    // One byte plus its acknowledge bit.
    I2c& byte(std::uint8_t value, bool ack = true) {
        for (int i = 7; i >= 0; --i) {
            bit((value >> i) & 1u);
        }
        bit(ack ? 0 : 1);  // the receiver pulls SDA low to acknowledge
        return *this;
    }

    I2c& address(std::uint8_t address, bool read, bool ack = true) {
        return byte(static_cast<std::uint8_t>((address << 1) | (read ? 1u : 0u)), ack);
    }

    // A complete write: START, address, payload, STOP.
    I2c& write(std::uint8_t address, const std::vector<std::uint8_t>& data, bool ack = true) {
        start().address(address, false, ack);
        for (std::uint8_t value : data) {
            byte(value, ack);
        }
        return stop();
    }

    // A complete read. The final byte is NACKed, as a master does to tell the
    // slave to stop sending.
    I2c& read(std::uint8_t address, const std::vector<std::uint8_t>& data) {
        start().address(address, true);
        for (std::size_t i = 0; i < data.size(); ++i) {
            byte(data[i], /*ack=*/i + 1 < data.size());
        }
        return stop();
    }

    I2c& idle(int bits = 2) {
        level(true, true, quarter_ * 4 * static_cast<std::size_t>(bits));
        return *this;
    }

    SampleStream build() const { return recorder_.build(rate_); }

private:
    void bit(unsigned value) {
        const bool data = value != 0;
        level(false, data, quarter_);      // set data while the clock is low
        level(true, data, quarter_ * 2);   // rising edge samples it
        level(false, data, quarter_);
    }

    void level(bool clock, bool data, std::size_t samples) {
        recorder_.set(scl_, clock);
        recorder_.set(sda_, data);
        recorder_.hold(samples);
    }

    double rate_;
    std::size_t quarter_;
    int scl_;
    int sda_;
    Recorder recorder_;
};

// SPI waveform generator for all four modes.
class Spi {
public:
    Spi(const SpiConfig& config, double sample_rate = 1e7, double clock_hz = 1e6)
        : config_(config),
          rate_(sample_rate),
          half_(divide_rate(sample_rate, clock_hz, 2)),
          recorder_(channel_count(config)) {
        recorder_.set(config_.clk, config_.cpol);
        recorder_.set(config_.cs, config_.cs_active_low);  // deasserted
        recorder_.hold(half_ * 4);
    }

    // One chip-select assertion carrying `mosi` words, with `miso` returned in
    // the same slots. A shorter miso vector is padded with zeroes.
    Spi& transaction(const std::vector<std::uint32_t>& mosi,
                     const std::vector<std::uint32_t>& miso = {}) {
        recorder_.set(config_.cs, !config_.cs_active_low);  // asserted
        recorder_.set(config_.clk, config_.cpol);
        recorder_.hold(half_);

        for (std::size_t w = 0; w < mosi.size(); ++w) {
            const std::uint32_t out = mosi[w];
            const std::uint32_t in = w < miso.size() ? miso[w] : 0u;
            for (int b = 0; b < config_.bits; ++b) {
                const int shift = config_.msb_first ? config_.bits - 1 - b : b;
                word_bit((out >> shift) & 1u, (in >> shift) & 1u);
            }
        }

        recorder_.set(config_.clk, config_.cpol);
        recorder_.hold(half_);
        recorder_.set(config_.cs, config_.cs_active_low);  // deasserted
        recorder_.hold(half_ * 4);
        return *this;
    }

    SampleStream build() const { return recorder_.build(rate_); }

private:
    static int channel_count(const SpiConfig& config) {
        return std::max({config.clk, config.mosi, config.miso, config.cs}) + 1;
    }

    void word_bit(std::uint32_t mosi_bit, std::uint32_t miso_bit) {
        recorder_.set(config_.mosi, mosi_bit != 0);
        recorder_.set(config_.miso, miso_bit != 0);
        if (config_.cpha) {
            // Data changes on the leading edge, so drive it as the clock leaves
            // idle and let the trailing edge do the sampling.
            recorder_.set(config_.clk, !config_.cpol);
            recorder_.hold(half_);
            recorder_.set(config_.clk, config_.cpol);
            recorder_.hold(half_);
        } else {
            // Data is already valid when the clock leaves idle.
            recorder_.set(config_.clk, config_.cpol);
            recorder_.hold(half_);
            recorder_.set(config_.clk, !config_.cpol);
            recorder_.hold(half_);
        }
    }

    SpiConfig config_;
    double rate_;
    std::size_t half_;
    Recorder recorder_;
};

// UART waveform generator, with deliberate corruption available so the error
// paths in the decoder can be tested.
struct UartDefect {
    bool bad_stop = false;    // hold the stop bit at the wrong level
    bool flip_parity = false;
};

class Uart {
public:
    Uart(const UartConfig& config, double sample_rate = 1e6)
        : config_(config),
          rate_(sample_rate),
          bit_(divide_rate(sample_rate, config.baud, 1)),
          recorder_(config.channel + 1) {
        recorder_.set(config_.channel, idle_level());
        recorder_.hold(bit_ * 2);
    }

    Uart& byte(std::uint32_t value, const UartDefect& defect = {}) {
        emit(false, bit_);  // start bit

        int ones = 0;
        for (int b = 0; b < config_.data_bits; ++b) {
            const int shift = config_.msb_first ? config_.data_bits - 1 - b : b;
            const bool level = ((value >> shift) & 1u) != 0;
            ones += level ? 1 : 0;
            emit(level, bit_);
        }

        if (config_.parity != Parity::None) {
            bool parity = (config_.parity == Parity::Even) ? (ones % 2 != 0) : (ones % 2 == 0);
            if (defect.flip_parity) {
                parity = !parity;
            }
            emit(parity, bit_);
        }

        const std::size_t stop_samples = static_cast<std::size_t>(config_.stop_bits * bit_ + 0.5);
        emit(!defect.bad_stop, stop_samples);
        return *this;
    }

    Uart& bytes(const std::string& text) {
        for (char c : text) {
            byte(static_cast<std::uint8_t>(c));
        }
        return *this;
    }

    Uart& bytes(const std::vector<std::uint32_t>& values) {
        for (std::uint32_t value : values) {
            byte(value);
        }
        return *this;
    }

    Uart& idle(int bits = 2) {
        emit(true, bit_ * static_cast<std::size_t>(bits));
        return *this;
    }

    SampleStream build() {
        idle(2);  // leave room for the last frame's stop bit to be measured
        return recorder_.build(rate_);
    }

private:
    bool idle_level() const { return !config_.invert; }

    // `mark` is the logical level: true is idle/1, inverted at the pin when
    // the capture was taken through an inverting driver.
    void emit(bool mark, std::size_t samples) {
        recorder_.set(config_.channel, config_.invert ? !mark : mark);
        recorder_.hold(samples);
    }

    UartConfig config_;
    double rate_;
    std::size_t bit_;
    Recorder recorder_;
};

// A square wave, for exercising the timing measurements.
inline SampleStream square(double sample_rate, double frequency_hz, double duty_cycle,
                           int cycles, int channel = 0) {
    if (duty_cycle <= 0.0 || duty_cycle >= 1.0) {
        throw std::invalid_argument("duty cycle must be between 0 and 1, exclusive");
    }
    const std::size_t period = divide_rate(sample_rate, frequency_hz, 1);
    const std::size_t high = std::max<std::size_t>(1, static_cast<std::size_t>(period * duty_cycle + 0.5));

    Recorder recorder(channel + 1);
    recorder.set(channel, false);
    recorder.hold(period);
    for (int i = 0; i < cycles; ++i) {
        recorder.set(channel, true);
        recorder.hold(high);
        recorder.set(channel, false);
        recorder.hold(period - high);
    }
    return recorder.build(sample_rate);
}

}  // namespace synth
}  // namespace logicdec

#endif  // LOGICDEC_SYNTH_HPP
