#include "logicdec/spi.hpp"

#include "logicdec/synth.hpp"
#include "test_framework.hpp"

using namespace logicdec;

namespace {

SpiConfig standard_config() {
    SpiConfig config;
    config.clk = 0;
    config.mosi = 1;
    config.miso = 2;
    config.cs = 3;
    return config;
}

std::vector<std::uint32_t> words(std::initializer_list<int> values) {
    std::vector<std::uint32_t> out;
    for (int value : values) {
        out.push_back(static_cast<std::uint32_t>(value));
    }
    return out;
}

}  // namespace

TEST(Spi, mode_zero_round_trips) {
    SpiConfig config = standard_config();
    SampleStream stream = synth::Spi(config).transaction(words({0x9F, 0x00}), words({0x00, 0xEF})).build();
    SpiResult result = decode_spi(stream, config);

    REQUIRE(result.words.size() == 2);
    CHECK_EQ(result.mosi_bytes(), words({0x9F, 0x00}));
    CHECK_EQ(result.miso_bytes(), words({0x00, 0xEF}));
    CHECK(result.clean());
}

TEST(Spi, all_four_modes_round_trip) {
    for (int mode = 0; mode < 4; ++mode) {
        SpiConfig config = standard_config();
        config.cpol = (mode & 2) != 0;
        config.cpha = (mode & 1) != 0;
        CHECK_EQ(config.mode(), mode);

        SampleStream stream = synth::Spi(config).transaction(words({0xA5, 0x3C, 0xFF})).build();
        SpiResult result = decode_spi(stream, config);

        REQUIRE(result.words.size() == 3);
        CHECK_EQ(result.mosi_bytes(), words({0xA5, 0x3C, 0xFF}));
    }
}

TEST(Spi, sampling_edge_follows_from_the_mode) {
    // The four modes reduce to one rule; this pins it down so a refactor cannot
    // quietly swap two of them.
    SpiConfig config;
    config.cpol = false; config.cpha = false; CHECK_EQ(config.samples_on_rising(), true);
    config.cpol = false; config.cpha = true;  CHECK_EQ(config.samples_on_rising(), false);
    config.cpol = true;  config.cpha = false; CHECK_EQ(config.samples_on_rising(), false);
    config.cpol = true;  config.cpha = true;  CHECK_EQ(config.samples_on_rising(), true);
}

TEST(Spi, decoding_with_the_wrong_mode_does_not_silently_return_the_right_bytes) {
    // This is the failure that costs an afternoon on a bench: the decode looks
    // successful and the bytes are wrong. It must not agree with the correct
    // mode's output.
    SpiConfig correct = standard_config();
    SampleStream stream = synth::Spi(correct).transaction(words({0xA5, 0x3C})).build();

    SpiConfig wrong = correct;
    wrong.cpha = true;
    SpiResult mismatched = decode_spi(stream, wrong);

    CHECK(mismatched.mosi_bytes() != words({0xA5, 0x3C}));
}

TEST(Spi, lsb_first_round_trips) {
    SpiConfig config = standard_config();
    config.msb_first = false;
    SampleStream stream = synth::Spi(config).transaction(words({0x01, 0x80})).build();

    CHECK_EQ(decode_spi(stream, config).mosi_bytes(), words({0x01, 0x80}));
}

TEST(Spi, bit_order_actually_matters) {
    SpiConfig msb = standard_config();
    SampleStream stream = synth::Spi(msb).transaction(words({0x01})).build();

    SpiConfig lsb = msb;
    lsb.msb_first = false;
    REQUIRE(decode_spi(stream, lsb).words.size() == 1);
    CHECK_EQ(decode_spi(stream, lsb).mosi_bytes(), words({0x80}));
}

TEST(Spi, word_sizes_other_than_eight_bits) {
    for (int bits : {4, 12, 16}) {
        SpiConfig config = standard_config();
        config.bits = bits;
        const std::uint32_t value = (1u << (bits - 1)) | 1u;

        SampleStream stream = synth::Spi(config).transaction({value}).build();
        SpiResult result = decode_spi(stream, config);

        REQUIRE(result.words.size() == 1);
        CHECK_EQ(result.words[0].mosi, value);
    }
}

TEST(Spi, chip_select_separates_transactions) {
    SpiConfig config = standard_config();
    synth::Spi generator(config);
    generator.transaction(words({0x01, 0x02}));
    generator.transaction(words({0x03}));

    SpiResult result = decode_spi(generator.build(), config);

    CHECK_EQ(result.transactions, std::size_t{2});
    REQUIRE(result.words.size() == 3);
    CHECK_EQ(result.words[0].transaction, std::size_t{0});
    CHECK_EQ(result.words[2].transaction, std::size_t{1});
    CHECK_EQ(count_kind(result.annotations, AnnotationKind::Start), std::size_t{2});
    CHECK_EQ(count_kind(result.annotations, AnnotationKind::Stop), std::size_t{2});
}

TEST(Spi, clock_activity_outside_chip_select_is_ignored) {
    SpiConfig config = standard_config();
    SampleStream idle_clocking = SampleStream::from_patterns(
        {"0101010101010101010101010101010101",  // clk toggling
         "1111111111111111111111111111111111",  // mosi high
         "0000000000000000000000000000000000",  // miso
         "1111111111111111111111111111111111"}, // cs deasserted (active low)
        1e6);

    SpiResult result = decode_spi(idle_clocking, config);
    CHECK_EQ(result.words.size(), std::size_t{0});
}

TEST(Spi, a_partial_word_is_flagged_rather_than_padded) {
    SpiConfig config = standard_config();
    config.bits = 8;
    SampleStream stream = synth::Spi(config).transaction(words({0xAB})).build();

    SpiConfig wider = config;
    wider.bits = 12;  // the transaction only carries 8 bits
    SpiResult result = decode_spi(stream, wider);

    CHECK_EQ(result.words.size(), std::size_t{0});
    CHECK_EQ(count_kind(result.annotations, AnnotationKind::Warning), std::size_t{1});
}

TEST(Spi, a_capture_without_chip_select_is_one_transaction) {
    SpiConfig config = standard_config();
    config.cs = -1;
    SampleStream stream = synth::Spi(standard_config()).transaction(words({0x11, 0x22})).build();

    SpiResult result = decode_spi(stream, config);
    CHECK_EQ(result.transactions, std::size_t{1});
    CHECK_EQ(result.mosi_bytes(), words({0x11, 0x22}));
}

TEST(Spi, miso_only_captures_are_allowed) {
    SpiConfig full = standard_config();
    SampleStream stream = synth::Spi(full).transaction(words({0x00}), words({0x5A})).build();

    SpiConfig miso_only = full;
    miso_only.mosi = -1;
    SpiResult result = decode_spi(stream, miso_only);

    REQUIRE(result.words.size() == 1);
    CHECK_EQ(result.words[0].miso, std::uint32_t{0x5A});
}

TEST(Spi, a_capture_with_no_data_channel_is_rejected) {
    SpiConfig config = standard_config();
    config.mosi = -1;
    config.miso = -1;
    SampleStream stream = synth::Spi(standard_config()).transaction(words({0x00})).build();

    CHECK_THROWS(decode_spi(stream, config));
}

TEST(Spi, an_impossible_word_size_is_rejected) {
    SpiConfig config = standard_config();
    SampleStream stream = synth::Spi(config).transaction(words({0x00})).build();

    config.bits = 0;
    CHECK_THROWS(decode_spi(stream, config));
    config.bits = 33;
    CHECK_THROWS(decode_spi(stream, config));
}

TEST(Spi, active_high_chip_select) {
    SpiConfig config = standard_config();
    config.cs_active_low = false;
    SampleStream stream = synth::Spi(config).transaction(words({0x7E})).build();

    CHECK_EQ(decode_spi(stream, config).mosi_bytes(), words({0x7E}));
}
