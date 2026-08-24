#include "logicdec/uart.hpp"

#include "logicdec/synth.hpp"
#include "test_framework.hpp"

using namespace logicdec;

namespace {

std::vector<std::uint32_t> values(std::initializer_list<int> items) {
    std::vector<std::uint32_t> out;
    for (int item : items) {
        out.push_back(static_cast<std::uint32_t>(item));
    }
    return out;
}

}  // namespace

TEST(Uart, text_round_trips) {
    UartConfig config;
    config.baud = 115200;
    SampleStream stream = synth::Uart(config, 4e6).bytes("Hello, bus!").build();

    UartResult result = decode_uart(stream, config);
    CHECK_EQ(result.text(), std::string("Hello, bus!"));
    CHECK(result.clean());
}

TEST(Uart, every_byte_value_round_trips) {
    UartConfig config;
    config.baud = 9600;

    std::vector<std::uint32_t> all;
    for (int i = 0; i < 256; ++i) {
        all.push_back(static_cast<std::uint32_t>(i));
    }

    SampleStream stream = synth::Uart(config, 1e6).bytes(all).build();
    CHECK_EQ(decode_uart(stream, config).values(), all);
}

TEST(Uart, standard_baud_rates_round_trip) {
    for (double baud : {9600.0, 19200.0, 38400.0, 57600.0, 115200.0}) {
        UartConfig config;
        config.baud = baud;
        SampleStream stream = synth::Uart(config, baud * 16).bytes("abc").build();
        CHECK_EQ(decode_uart(stream, config).text(), std::string("abc"));
    }
}

TEST(Uart, even_parity_round_trips) {
    UartConfig config;
    config.baud = 9600;
    config.parity = Parity::Even;

    SampleStream stream = synth::Uart(config, 1e6).bytes(values({0x00, 0x01, 0x03, 0xFF})).build();
    UartResult result = decode_uart(stream, config);

    CHECK_EQ(result.values(), values({0x00, 0x01, 0x03, 0xFF}));
    CHECK(result.clean());
}

TEST(Uart, odd_parity_round_trips) {
    UartConfig config;
    config.baud = 9600;
    config.parity = Parity::Odd;

    SampleStream stream = synth::Uart(config, 1e6).bytes(values({0x00, 0x55, 0xAA})).build();
    UartResult result = decode_uart(stream, config);

    CHECK_EQ(result.values(), values({0x00, 0x55, 0xAA}));
    CHECK(result.clean());
}

TEST(Uart, a_flipped_parity_bit_is_caught) {
    UartConfig config;
    config.baud = 9600;
    config.parity = Parity::Even;

    synth::Uart generator(config, 1e6);
    synth::UartDefect defect;
    defect.flip_parity = true;
    generator.byte(0x41).byte(0x42, defect).byte(0x43);

    UartResult result = decode_uart(generator.build(), config);
    REQUIRE(result.frames.size() == 3);
    CHECK(!result.frames[0].parity_error);
    CHECK(result.frames[1].parity_error);
    CHECK(!result.frames[2].parity_error);
    CHECK(!result.clean());
}

TEST(Uart, a_corrupted_stop_bit_is_a_framing_error) {
    UartConfig config;
    config.baud = 9600;

    synth::Uart generator(config, 1e6);
    synth::UartDefect defect;
    defect.bad_stop = true;
    generator.byte(0x41).byte(0x42, defect).idle(4).byte(0x43);

    UartResult result = decode_uart(generator.build(), config);

    bool saw_framing_error = false;
    for (const UartFrame& frame : result.frames) {
        saw_framing_error = saw_framing_error || frame.framing_error;
    }
    CHECK(saw_framing_error);
    CHECK(!result.clean());
}

TEST(Uart, decoding_recovers_after_a_framing_error) {
    UartConfig config;
    config.baud = 9600;

    synth::Uart generator(config, 1e6);
    synth::UartDefect defect;
    defect.bad_stop = true;
    generator.byte(0x42, defect).idle(4).bytes("ok");

    UartResult result = decode_uart(generator.build(), config);
    // The good bytes after the glitch still come through.
    CHECK(result.text().find("ok") != std::string::npos);
}

TEST(Uart, seven_data_bits) {
    UartConfig config;
    config.baud = 9600;
    config.data_bits = 7;

    SampleStream stream = synth::Uart(config, 1e6).bytes("7bit").build();
    CHECK_EQ(decode_uart(stream, config).text(), std::string("7bit"));
}

TEST(Uart, two_stop_bits) {
    UartConfig config;
    config.baud = 9600;
    config.stop_bits = 2.0;

    SampleStream stream = synth::Uart(config, 1e6).bytes("two").build();
    UartResult result = decode_uart(stream, config);

    CHECK_EQ(result.text(), std::string("two"));
    CHECK(result.clean());
}

TEST(Uart, an_inverted_line_round_trips) {
    UartConfig config;
    config.baud = 9600;
    config.invert = true;

    SampleStream stream = synth::Uart(config, 1e6).bytes("inv").build();
    CHECK_EQ(decode_uart(stream, config).text(), std::string("inv"));
}

TEST(Uart, msb_first_round_trips) {
    UartConfig config;
    config.baud = 9600;
    config.msb_first = true;

    SampleStream stream = synth::Uart(config, 1e6).bytes(values({0x01, 0x80, 0xC3})).build();
    CHECK_EQ(decode_uart(stream, config).values(), values({0x01, 0x80, 0xC3}));
}

TEST(Uart, a_short_glitch_is_not_mistaken_for_a_start_bit) {
    // A single-sample dip at 1 MS/s is 1/104th of a 9600-baud bit. Treating it
    // as a start bit would produce a byte of noise and desynchronise the rest
    // of the stream.
    UartConfig config;
    config.baud = 9600;

    synth::Uart generator(config, 1e6);
    generator.idle(2);
    SampleStream clean_stream = generator.build();

    std::vector<std::string> rows = clean_stream.to_patterns();
    rows[0][10] = '_';  // one sample low
    SampleStream glitched = SampleStream::from_patterns(rows, 1e6);

    CHECK_EQ(decode_uart(glitched, config).frames.size(), std::size_t{0});
}

TEST(Uart, an_idle_line_decodes_to_nothing) {
    UartConfig config;
    config.baud = 9600;
    SampleStream stream = SampleStream::from_patterns({std::string(2000, '1')}, 1e6);

    CHECK_EQ(decode_uart(stream, config).frames.size(), std::size_t{0});
}

TEST(Uart, too_few_samples_per_bit_is_refused_rather_than_guessed) {
    UartConfig config;
    config.baud = 115200;
    SampleStream stream = SampleStream::from_patterns({std::string(200, '1')}, 200e3);

    // 1.7 samples per bit cannot place a bit centre; decoding anyway would
    // produce confident nonsense.
    CHECK_THROWS(decode_uart(stream, config));
}

TEST(Uart, invalid_configurations_are_rejected) {
    SampleStream stream = SampleStream::from_patterns({std::string(2000, '1')}, 1e6);

    UartConfig too_few_bits;
    too_few_bits.data_bits = 3;
    CHECK_THROWS(decode_uart(stream, too_few_bits));

    UartConfig no_stop;
    no_stop.stop_bits = 0.0;
    CHECK_THROWS(decode_uart(stream, no_stop));

    UartConfig zero_baud;
    zero_baud.baud = 0.0;
    CHECK_THROWS(decode_uart(stream, zero_baud));
}

TEST(Uart, baud_can_be_estimated_from_the_waveform) {
    for (double baud : {9600.0, 38400.0, 115200.0}) {
        UartConfig config;
        config.baud = baud;
        SampleStream stream = synth::Uart(config, baud * 32).bytes("UUU\x01").build();

        const double estimate = estimate_baud(stream, 0);
        CHECK_NEAR(estimate / baud, 1.0, 0.1);
        CHECK_NEAR(nearest_standard_baud(estimate), baud, 1.0);
    }
}

TEST(Uart, baud_estimation_on_a_flat_line_returns_zero) {
    SampleStream stream = SampleStream::from_patterns({std::string(500, '1')}, 1e6);
    CHECK_NEAR(estimate_baud(stream, 0), 0.0, 1e-12);
    CHECK_NEAR(nearest_standard_baud(0.0), 0.0, 1e-12);
}

TEST(Uart, decoding_at_the_wrong_baud_rate_does_not_produce_the_right_text) {
    UartConfig config;
    config.baud = 9600;
    SampleStream stream = synth::Uart(config, 1e6).bytes("payload").build();

    UartConfig wrong = config;
    wrong.baud = 19200;
    CHECK(decode_uart(stream, wrong).text() != std::string("payload"));
}
