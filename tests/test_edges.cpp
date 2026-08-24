#include "logicdec/edges.hpp"

#include "logicdec/synth.hpp"
#include "test_framework.hpp"

using namespace logicdec;

TEST(Edges, transitions_are_found_in_order) {
    SampleStream stream = SampleStream::from_patterns({"0011001"}, 1e6);
    std::vector<EdgeEvent> edges = find_edges(stream, 0);

    REQUIRE(edges.size() == 3);
    CHECK_EQ(edges[0].index, SampleIndex{2});
    CHECK(edges[0].edge == Edge::Rising);
    CHECK_EQ(edges[1].index, SampleIndex{4});
    CHECK(edges[1].edge == Edge::Falling);
    CHECK_EQ(edges[2].index, SampleIndex{6});
}

TEST(Edges, a_flat_line_has_no_edges) {
    CHECK_EQ(find_edges(SampleStream::from_patterns({"11111"}, 1e6), 0).size(), std::size_t{0});
    CHECK_EQ(find_edges(SampleStream::from_patterns({"00000"}, 1e6), 0).size(), std::size_t{0});
}

TEST(Edges, a_single_sample_has_no_edges) {
    CHECK_EQ(find_edges(SampleStream::from_patterns({"1"}, 1e6), 0).size(), std::size_t{0});
}

TEST(Edges, next_edge_skips_the_wrong_direction) {
    SampleStream stream = SampleStream::from_patterns({"0011001111"}, 1e6);
    ChannelView view = stream.channel(0);

    CHECK_EQ(next_edge(view, 0, Edge::Rising), SampleIndex{2});
    CHECK_EQ(next_edge(view, 2, Edge::Falling), SampleIndex{4});
    CHECK_EQ(next_edge(view, 2, Edge::Rising), SampleIndex{6});
}

TEST(Edges, next_edge_returns_the_size_when_there_is_none) {
    SampleStream stream = SampleStream::from_patterns({"0000"}, 1e6);
    CHECK_EQ(next_edge(stream.channel(0), 0, Edge::Rising), SampleIndex{4});
}

TEST(Edges, pulse_widths_ignore_the_partial_runs_at_each_end) {
    // A capture rarely starts or ends on an edge; counting those truncated runs
    // makes a steady clock look like it has a short pulse.
    SampleStream stream = SampleStream::from_patterns({"1" "0000" "1111" "0000" "11"}, 1e6);
    PulseWidths widths = measure_pulses(stream, 0);

    REQUIRE(widths.low.size() == 2);
    CHECK_EQ(widths.low[0], std::size_t{4});
    CHECK_EQ(widths.low[1], std::size_t{4});
    REQUIRE(widths.high.size() == 1);
    CHECK_EQ(widths.high[0], std::size_t{4});
}

TEST(Timing, frequency_of_a_clean_square_wave) {
    SampleStream stream = synth::square(1e6, 10e3, 0.5, 20);
    TimingStats stats = measure_timing(stream, 0);

    CHECK(stats.measurable());
    CHECK_NEAR(stats.frequency_hz, 10e3, 1.0);
    CHECK_NEAR(stats.period_mean_s, 1e-4, 1e-9);
    CHECK_EQ(stats.periods, std::size_t{19});
}

TEST(Timing, duty_cycle_is_measured) {
    SampleStream stream = synth::square(1e6, 10e3, 0.25, 20);
    TimingStats stats = measure_timing(stream, 0);

    CHECK_NEAR(stats.duty_cycle, 0.25, 0.01);
}

TEST(Timing, an_asymmetric_wave_still_reports_the_right_period) {
    // Period is measured rising edge to rising edge for exactly this reason:
    // half-period doubling would report 2x the frequency at 10% duty.
    SampleStream stream = synth::square(1e6, 5e3, 0.1, 10);
    TimingStats stats = measure_timing(stream, 0);

    CHECK_NEAR(stats.frequency_hz, 5e3, 1.0);
    CHECK_NEAR(stats.duty_cycle, 0.1, 0.01);
}

TEST(Timing, jitter_is_the_spread_between_periods) {
    SampleStream steady = synth::square(1e6, 10e3, 0.5, 10);
    CHECK_NEAR(measure_timing(steady, 0).jitter_s, 0.0, 1e-12);

    // 100 samples, then 120, then 100: one long period in the middle.
    SampleStream uneven = SampleStream::from_patterns(
        {std::string("0") + std::string(50, '1') + std::string(50, '0') + std::string(60, '1') +
         std::string(60, '0') + std::string(50, '1') + std::string(50, '0')},
        1e6);
    TimingStats stats = measure_timing(uneven, 0);
    CHECK_EQ(stats.period_min_samples, std::size_t{100});
    CHECK_EQ(stats.period_max_samples, std::size_t{120});
    CHECK_NEAR(stats.jitter_s, 20e-6, 1e-9);
}

TEST(Timing, a_stuck_pin_is_not_measurable) {
    SampleStream stream = SampleStream::from_patterns({std::string(100, '1')}, 1e6);
    TimingStats stats = measure_timing(stream, 0);

    CHECK(!stats.measurable());
    CHECK_NEAR(stats.frequency_hz, 0.0, 1e-12);
    // ...but it is still obviously high, which is the useful answer here.
    CHECK_NEAR(high_fraction(stream, 0), 1.0, 1e-12);
}

TEST(Timing, one_period_is_not_enough_to_measure) {
    SampleStream stream = SampleStream::from_patterns({"0011110000"}, 1e6);
    CHECK(!measure_timing(stream, 0).measurable());
}

TEST(Timing, edge_counts_match_the_waveform) {
    SampleStream stream = synth::square(1e6, 10e3, 0.5, 7);
    CHECK_EQ(count_edges(stream, 0, Edge::Rising), std::size_t{7});
    CHECK_EQ(count_edges(stream, 0, Edge::Falling), std::size_t{7});
}

TEST(Timing, high_fraction_of_a_half_duty_wave) {
    SampleStream stream = synth::square(1e6, 10e3, 0.5, 50);
    CHECK_NEAR(high_fraction(stream, 0), 0.5, 0.02);
}
