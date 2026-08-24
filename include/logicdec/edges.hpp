// logicdec - edges.hpp
//
// Edge finding and the timing measurements built on it: frequency, period,
// duty cycle, pulse widths. This is the layer under the protocol decoders and
// also the whole answer for "what is this pin doing", which is most of what a
// handheld instrument gets used for.
//
// Everything here reports in both samples and seconds. Sample counts are exact;
// seconds depend on the capture rate and quietly lose meaning if the rate is
// wrong, so the exact figure stays available.

#ifndef LOGICDEC_EDGES_HPP
#define LOGICDEC_EDGES_HPP

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

#include "logicdec/sample_stream.hpp"

namespace logicdec {

enum class Edge { Rising, Falling };

struct EdgeEvent {
    SampleIndex index = 0;  // the sample at which the new level is first seen
    Edge edge = Edge::Rising;
};

// Every transition on `channel`, in order.
inline std::vector<EdgeEvent> find_edges(const SampleStream& stream, int channel) {
    std::vector<EdgeEvent> edges;
    if (stream.size() < 2) {
        return edges;
    }

    ChannelView view = stream.channel(channel);
    bool previous = view[0];
    for (SampleIndex i = 1; i < view.size(); ++i) {
        const bool current = view[i];
        if (current != previous) {
            edges.push_back({i, current ? Edge::Rising : Edge::Falling});
            previous = current;
        }
    }
    return edges;
}

// Index of the first edge of `edge` at or after `from`, or stream.size() if
// there is none. Decoders use this to advance a clock without re-scanning.
inline SampleIndex next_edge(const ChannelView& view, SampleIndex from, Edge edge) {
    if (from >= view.size()) {
        return view.size();
    }
    bool previous = view[from];
    for (SampleIndex i = from + 1; i < view.size(); ++i) {
        const bool current = view[i];
        if (current != previous) {
            if ((edge == Edge::Rising) == current) {
                return i;
            }
            previous = current;
        }
    }
    return view.size();
}

inline SampleIndex next_change(const ChannelView& view, SampleIndex from) {
    if (from >= view.size()) {
        return view.size();
    }
    const bool level = view[from];
    for (SampleIndex i = from + 1; i < view.size(); ++i) {
        if (view[i] != level) {
            return i;
        }
    }
    return view.size();
}

struct PulseWidths {
    std::vector<std::size_t> high;  // in samples
    std::vector<std::size_t> low;
};

// Widths of every complete pulse. The first and last runs are dropped: a
// capture almost never starts or ends exactly on an edge, so including them
// biases the minimum downward and makes a stable clock look jittery.
inline PulseWidths measure_pulses(const SampleStream& stream, int channel) {
    PulseWidths widths;
    const std::vector<EdgeEvent> edges = find_edges(stream, channel);
    if (edges.size() < 2) {
        return widths;
    }

    for (std::size_t i = 0; i + 1 < edges.size(); ++i) {
        const std::size_t width = edges[i + 1].index - edges[i].index;
        // The level between edge i and i+1 is whatever edge i produced.
        if (edges[i].edge == Edge::Rising) {
            widths.high.push_back(width);
        } else {
            widths.low.push_back(width);
        }
    }
    return widths;
}

struct TimingStats {
    std::size_t rising = 0;
    std::size_t falling = 0;
    std::size_t periods = 0;

    double frequency_hz = 0.0;
    double period_mean_s = 0.0;
    double period_min_s = 0.0;
    double period_max_s = 0.0;
    double jitter_s = 0.0;      // max - min period
    double duty_cycle = 0.0;    // 0..1, mean high time over mean period

    std::size_t period_min_samples = 0;
    std::size_t period_max_samples = 0;

    bool measurable() const { return periods > 0; }
    // A single measured period, in samples, as a double so sub-sample means survive.
    double period_mean_samples = 0.0;
};

// Frequency, period spread and duty cycle for `channel`.
//
// Period is measured rising-edge to rising-edge, which is the convention that
// stays correct for an asymmetric duty cycle. With fewer than two rising edges
// there is nothing to measure and `measurable()` is false rather than the
// result being a plausible-looking zero.
inline TimingStats measure_timing(const SampleStream& stream, int channel) {
    TimingStats stats;
    const std::vector<EdgeEvent> edges = find_edges(stream, channel);

    std::vector<SampleIndex> rising;
    for (const EdgeEvent& event : edges) {
        if (event.edge == Edge::Rising) {
            ++stats.rising;
            rising.push_back(event.index);
        } else {
            ++stats.falling;
        }
    }

    if (rising.size() < 2) {
        return stats;
    }

    std::size_t min_period = std::numeric_limits<std::size_t>::max();
    std::size_t max_period = 0;
    std::size_t total = 0;
    for (std::size_t i = 0; i + 1 < rising.size(); ++i) {
        const std::size_t period = rising[i + 1] - rising[i];
        min_period = std::min(min_period, period);
        max_period = std::max(max_period, period);
        total += period;
    }

    stats.periods = rising.size() - 1;
    stats.period_mean_samples = static_cast<double>(total) / static_cast<double>(stats.periods);
    stats.period_mean_s = stats.period_mean_samples / stream.sample_rate();
    stats.period_min_s = static_cast<double>(min_period) / stream.sample_rate();
    stats.period_max_s = static_cast<double>(max_period) / stream.sample_rate();
    stats.period_min_samples = min_period;
    stats.period_max_samples = max_period;
    stats.jitter_s = stats.period_max_s - stats.period_min_s;
    stats.frequency_hz = stats.period_mean_s > 0.0 ? 1.0 / stats.period_mean_s : 0.0;

    const PulseWidths widths = measure_pulses(stream, channel);
    if (!widths.high.empty() && stats.period_mean_samples > 0.0) {
        std::size_t high_total = 0;
        for (std::size_t width : widths.high) {
            high_total += width;
        }
        const double mean_high = static_cast<double>(high_total) / static_cast<double>(widths.high.size());
        stats.duty_cycle = mean_high / stats.period_mean_samples;
    }

    return stats;
}

// Counts transitions in a window, which is what a frequency counter gate does.
inline std::size_t count_edges(const SampleStream& stream, int channel, Edge edge) {
    std::size_t total = 0;
    for (const EdgeEvent& event : find_edges(stream, channel)) {
        if (event.edge == edge) {
            ++total;
        }
    }
    return total;
}

// Fraction of the capture spent high. Unlike TimingStats::duty_cycle this needs
// no edges at all, so it stays meaningful for a pin that is simply stuck.
inline double high_fraction(const SampleStream& stream, int channel) {
    if (stream.empty()) {
        return 0.0;
    }
    ChannelView view = stream.channel(channel);
    std::size_t high = 0;
    for (SampleIndex i = 0; i < view.size(); ++i) {
        high += view[i] ? 1u : 0u;
    }
    return static_cast<double>(high) / static_cast<double>(view.size());
}

}  // namespace logicdec

#endif  // LOGICDEC_EDGES_HPP
