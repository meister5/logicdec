#include "logicdec/i2c.hpp"

#include "logicdec/synth.hpp"
#include "test_framework.hpp"

using namespace logicdec;

namespace {

std::vector<std::uint8_t> bytes(std::initializer_list<int> values) {
    std::vector<std::uint8_t> out;
    for (int value : values) {
        out.push_back(static_cast<std::uint8_t>(value));
    }
    return out;
}

}  // namespace

TEST(I2c, a_write_round_trips) {
    SampleStream stream = synth::I2c().write(0x3C, bytes({0x00, 0xAE})).build();
    I2cResult result = decode_i2c(stream);

    REQUIRE(result.transfers.size() == 1);
    const I2cTransfer& transfer = result.transfers[0];
    CHECK_EQ(transfer.address, std::uint8_t{0x3C});
    CHECK_EQ(transfer.read, false);
    CHECK_EQ(transfer.address_acked, true);
    CHECK_EQ(transfer.data, bytes({0x00, 0xAE}));
    CHECK(transfer.complete);
    CHECK(result.clean());
}

TEST(I2c, a_read_round_trips_and_nacks_the_last_byte) {
    SampleStream stream = synth::I2c().read(0x68, bytes({0x12, 0x34, 0x56})).build();
    I2cResult result = decode_i2c(stream);

    REQUIRE(result.transfers.size() == 1);
    const I2cTransfer& transfer = result.transfers[0];
    CHECK_EQ(transfer.address, std::uint8_t{0x68});
    CHECK_EQ(transfer.read, true);
    CHECK_EQ(transfer.data, bytes({0x12, 0x34, 0x56}));
    // The master NACKs the final byte to tell the slave to stop sending.
    REQUIRE(transfer.acks.size() == 3);
    CHECK_EQ(transfer.acks[0], true);
    CHECK_EQ(transfer.acks[2], false);
}

TEST(I2c, every_seven_bit_address_decodes_correctly) {
    for (int address = 0; address < 128; ++address) {
        SampleStream stream =
            synth::I2c().write(static_cast<std::uint8_t>(address), bytes({0x5A})).build();
        I2cResult result = decode_i2c(stream);
        REQUIRE(result.transfers.size() == 1);
        CHECK_EQ(static_cast<int>(result.transfers[0].address), address);
    }
}

TEST(I2c, every_byte_value_round_trips) {
    for (int value = 0; value < 256; ++value) {
        SampleStream stream = synth::I2c().write(0x50, bytes({value})).build();
        I2cResult result = decode_i2c(stream);
        REQUIRE(result.transfers.size() == 1);
        REQUIRE(result.transfers[0].data.size() == 1);
        CHECK_EQ(static_cast<int>(result.transfers[0].data[0]), value);
    }
}

TEST(I2c, an_unanswered_address_is_reported) {
    // Nothing on the bus at this address: the address byte goes out, nobody
    // pulls SDA low, and the transfer carries no data. Distinguishing this from
    // "wrote zero bytes successfully" is the whole point of tracking the ack.
    SampleStream stream = synth::I2c().start().address(0x77, false, /*ack=*/false).stop().build();
    I2cResult result = decode_i2c(stream);

    REQUIRE(result.transfers.size() == 1);
    CHECK_EQ(result.transfers[0].address, std::uint8_t{0x77});
    CHECK_EQ(result.transfers[0].address_acked, false);
    CHECK_EQ(count_kind(result.annotations, AnnotationKind::Nack), std::size_t{1});
}

TEST(I2c, a_repeated_start_splits_the_transfers) {
    // The register-read idiom: write the register pointer, then read back
    // without releasing the bus.
    synth::I2c generator;
    generator.start().address(0x68, false).byte(0x3B).repeated_start().address(0x68, true).byte(0xFF, false).stop();
    I2cResult result = decode_i2c(generator.build());

    REQUIRE(result.transfers.size() == 2);
    CHECK_EQ(result.transfers[0].read, false);
    CHECK_EQ(result.transfers[0].data, bytes({0x3B}));
    CHECK_EQ(result.transfers[1].read, true);
    CHECK_EQ(result.transfers[1].data, bytes({0xFF}));
    CHECK_EQ(count_kind(result.annotations, AnnotationKind::RepeatedStart), std::size_t{1});
    CHECK_EQ(count_kind(result.annotations, AnnotationKind::Start), std::size_t{1});
}

TEST(I2c, back_to_back_transfers_are_separated) {
    synth::I2c generator;
    generator.write(0x10, bytes({0x01})).write(0x20, bytes({0x02, 0x03}));
    I2cResult result = decode_i2c(generator.build());

    REQUIRE(result.transfers.size() == 2);
    CHECK_EQ(result.transfers[0].address, std::uint8_t{0x10});
    CHECK_EQ(result.transfers[1].address, std::uint8_t{0x20});
    CHECK_EQ(result.transfers[1].data.size(), std::size_t{2});
    CHECK_EQ(count_kind(result.annotations, AnnotationKind::Stop), std::size_t{2});
}

TEST(I2c, a_truncated_capture_is_flagged_rather_than_reported_as_complete) {
    SampleStream full = synth::I2c().write(0x3C, bytes({0xAA, 0xBB, 0xCC})).build();
    SampleStream cut = full.slice(0, full.size() * 2 / 3);
    I2cResult result = decode_i2c(cut);

    REQUIRE(result.transfers.size() == 1);
    CHECK_EQ(result.transfers[0].complete, false);
    CHECK_EQ(result.errors(), std::size_t{1});
    CHECK(!result.clean());
}

TEST(I2c, channels_can_be_swapped) {
    SampleStream stream = synth::I2c(1e6, 100e3, /*scl=*/3, /*sda=*/2).write(0x3C, bytes({0x42})).build();

    I2cConfig config;
    config.scl = 3;
    config.sda = 2;
    I2cResult result = decode_i2c(stream, config);

    REQUIRE(result.transfers.size() == 1);
    CHECK_EQ(result.transfers[0].data, bytes({0x42}));
}

TEST(I2c, decoding_with_scl_and_sda_swapped_does_not_invent_a_transfer) {
    SampleStream stream = synth::I2c().write(0x3C, bytes({0x42})).build();
    I2cConfig swapped;
    swapped.scl = 1;
    swapped.sda = 0;

    I2cResult result = decode_i2c(stream, swapped);
    // It may find framing conditions in the noise, but it must not produce a
    // complete transfer that looks like real traffic.
    for (const I2cTransfer& transfer : result.transfers) {
        CHECK(transfer.data.empty() || !transfer.complete || transfer.address != 0x3C);
    }
}

TEST(I2c, bus_speed_does_not_change_the_result) {
    for (double clock : {10e3, 100e3, 400e3}) {
        SampleStream stream = synth::I2c(4e6, clock).write(0x3C, bytes({0xDE, 0xAD})).build();
        I2cResult result = decode_i2c(stream);
        REQUIRE(result.transfers.size() == 1);
        CHECK_EQ(result.transfers[0].data, bytes({0xDE, 0xAD}));
    }
}

TEST(I2c, an_idle_bus_decodes_to_nothing) {
    SampleStream stream = SampleStream::from_patterns({std::string(500, '1'), std::string(500, '1')}, 1e6);
    I2cResult result = decode_i2c(stream);

    CHECK_EQ(result.transfers.size(), std::size_t{0});
    CHECK(result.clean());
}

TEST(I2c, annotations_cover_the_samples_they_describe) {
    SampleStream stream = synth::I2c().write(0x3C, bytes({0x42})).build();
    I2cResult result = decode_i2c(stream);

    REQUIRE(!result.annotations.empty());
    for (const Annotation& annotation : result.annotations) {
        CHECK(annotation.end > annotation.start);
        CHECK(annotation.end <= stream.size());
    }
}

TEST(I2c, the_summary_reads_like_a_bus_transcript) {
    synth::I2c generator;
    generator.write(0x3C, bytes({0x00, 0xAE})).read(0x68, bytes({0x12}));
    std::vector<std::string> lines = summarize(decode_i2c(generator.build()));

    REQUIRE(lines.size() == 2);
    CHECK_EQ(lines[0], std::string("0x3C W 0x00 0xAE"));
    CHECK_EQ(lines[1], std::string("0x68 R 0x12"));
}
