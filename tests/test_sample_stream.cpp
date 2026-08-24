#include "logicdec/sample_stream.hpp"

#include "test_framework.hpp"

using namespace logicdec;

TEST(SampleStream, from_patterns_packs_channels_into_bits) {
    SampleStream stream = SampleStream::from_patterns({"0011", "0101"}, 1e6);

    CHECK_EQ(stream.size(), std::size_t{4});
    CHECK_EQ(stream.channel_count(), 2);
    CHECK_EQ(stream.at(0, 0), false);
    CHECK_EQ(stream.at(0, 2), true);
    CHECK_EQ(stream.at(1, 1), true);
    CHECK_EQ(stream.at(1, 2), false);
}

TEST(SampleStream, waveform_characters_are_accepted) {
    SampleStream drawn = SampleStream::from_patterns({"__--__"}, 1e6);
    SampleStream digits = SampleStream::from_patterns({"001100"}, 1e6);
    CHECK_EQ(drawn.raw(), digits.raw());
}

TEST(SampleStream, spacing_characters_are_ignored) {
    SampleStream grouped = SampleStream::from_patterns({"0011 | 0011"}, 1e6);
    CHECK_EQ(grouped.size(), std::size_t{8});
}

TEST(SampleStream, patterns_round_trip) {
    SampleStream stream = SampleStream::from_patterns({"__--", "-__-"}, 1e6);
    std::vector<std::string> rows = stream.to_patterns();

    REQUIRE(rows.size() == 2);
    CHECK_EQ(rows[0], std::string("__--"));
    CHECK_EQ(rows[1], std::string("-__-"));
}

TEST(SampleStream, mismatched_pattern_lengths_are_rejected) {
    CHECK_THROWS(SampleStream::from_patterns({"0011", "01"}, 1e6));
}

TEST(SampleStream, unexpected_pattern_characters_are_rejected) {
    CHECK_THROWS(SampleStream::from_patterns({"00x1"}, 1e6));
}

TEST(SampleStream, more_than_eight_channels_is_rejected) {
    std::vector<std::string> rows(9, "01");
    CHECK_THROWS(SampleStream::from_patterns(rows, 1e6));
}

TEST(SampleStream, a_zero_sample_rate_is_rejected) {
    // Time conversions would silently divide by zero and report infinities.
    CHECK_THROWS(SampleStream(std::vector<Sample>{0, 1}, 0.0));
    CHECK_THROWS(SampleStream(std::vector<Sample>{0, 1}, -1.0));
}

TEST(SampleStream, out_of_range_access_is_reported) {
    SampleStream stream = SampleStream::from_patterns({"0011"}, 1e6);
    CHECK_THROWS(stream.at(1, 0));   // channel not in the capture
    CHECK_THROWS(stream.at(0, 99));  // past the end
}

TEST(SampleStream, time_conversions_use_the_sample_rate) {
    SampleStream stream = SampleStream::from_patterns({"00110011"}, 2e6);

    CHECK_NEAR(stream.time_at(4), 2e-6, 1e-12);
    CHECK_NEAR(stream.duration(), 4e-6, 1e-12);
    CHECK_EQ(stream.samples_in(1e-6), std::size_t{2});
}

TEST(SampleStream, channel_view_reads_the_same_bits) {
    SampleStream stream = SampleStream::from_patterns({"0011", "1100"}, 1e6);
    ChannelView view = stream.channel(1);

    CHECK_EQ(view.size(), std::size_t{4});
    CHECK_EQ(view[0], true);
    CHECK_EQ(view[3], false);
}

TEST(SampleStream, slice_keeps_the_rate_and_channels) {
    SampleStream stream = SampleStream::from_patterns({"00110011", "01010101"}, 1e6);
    SampleStream part = stream.slice(2, 6);

    CHECK_EQ(part.size(), std::size_t{4});
    CHECK_EQ(part.channel_count(), 2);
    CHECK_NEAR(part.sample_rate(), 1e6, 1e-9);
    CHECK_EQ(part.to_patterns()[0], std::string("--__"));
}

TEST(SampleStream, slice_bounds_are_checked) {
    SampleStream stream = SampleStream::from_patterns({"0011"}, 1e6);
    CHECK_THROWS(stream.slice(3, 1));
    CHECK_THROWS(stream.slice(0, 99));
}

TEST(SampleStream, from_channels_matches_from_patterns) {
    SampleStream a = SampleStream::from_channels({{false, false, true, true}, {true, false, true, false}}, 1e6);
    SampleStream b = SampleStream::from_patterns({"0011", "1010"}, 1e6);
    CHECK_EQ(a.raw(), b.raw());
}

TEST(SampleStream, an_empty_stream_is_usable) {
    SampleStream stream;
    CHECK(stream.empty());
    CHECK_EQ(stream.size(), std::size_t{0});
}
