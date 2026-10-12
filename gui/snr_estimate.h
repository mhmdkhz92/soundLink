#ifndef SOUNDLINK_SNR_ESTIMATE_H
#define SOUNDLINK_SNR_ESTIMATE_H
#include <algorithm>
#include <cmath>

struct RbSnrEstimate {
    bool valid = false;
    double db = 0;
    bool capped = false;
};

// The receiver currently demaps QPSK. Check that stored noise corresponds to
// the current moments: the shared estimator retains old noise on invalid updates.
inline RbSnrEstimate estimateQpskSnr(double m2, double m4, double noise) {
    if (!std::isfinite(m2) || !std::isfinite(m4) || !std::isfinite(noise) ||
        m2 <= 0 || m4 < 0 || noise < 0 || noise >= m2) return {};
    const double discriminant = 2 * m2 * m2 - m4;
    if (discriminant < 0) return {};
    const double momentSignal = std::sqrt(discriminant);
    if (momentSignal > m2) return {};
    if (std::abs((m2 - momentSignal) - noise) > 1e-5 * m2) return {};
    const double signal = m2 - noise;
    // Bound the display at 60 dB, including noiseless numerical estimates.
    const double floor = signal * 1e-6;
    return {true, 10 * std::log10(signal / std::max(noise, floor)), noise <= floor};
}
#endif
