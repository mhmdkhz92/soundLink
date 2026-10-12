#ifndef SOUNDLINK_RECEIVE_RATE_H
#define SOUNDLINK_RECEIVE_RATE_H

#include <chrono>
#include <cstdint>
#include <deque>

// Valid decoded payload bits per wall-clock second during continuous PRBS lock.
class ReceiveRate {
public:
    using Clock = std::chrono::steady_clock;

    void reset() { samples.clear(); }

    void observe(Clock::time_point now, bool locked, uint64_t bits) {
        if (!locked) {
            reset();
            return;
        }
        if (!samples.empty() && bits < samples.back().bits) reset();
        samples.push_back({now, bits});
        const auto cutoff = now - std::chrono::seconds(1);
        // Retain the boundary sample so both bits and elapsed time match.
        while (samples.size() > 2 && samples[1].time <= cutoff)
            samples.pop_front();
    }

    double value() const {
        if (samples.size() < 2) return -1;
        const double seconds = std::chrono::duration<double>(
            samples.back().time - samples.front().time).count();
        if (seconds < 0.25) return -1; // Allow a short measurement warm-up.
        return double(samples.back().bits - samples.front().bits) / seconds;
    }

private:
    struct Sample { Clock::time_point time; uint64_t bits; };
    std::deque<Sample> samples;
};

#endif
