#pragma once
#include <span>
#include <vector>
namespace bike::numerics {
// ride/metrics.py lowpass: odd-padding filtfilt, order=4/cutoff=100 Hz.
// <=3*(order+1) samples and cutoff>=Nyquist return an owned input copy.
[[nodiscard]] std::vector<double> legacy_lowpass(std::span<const double> samples,
    double sample_interval_s, double cutoff_hz = 100., int order = 4);
[[nodiscard]] std::vector<double> legacy_ride_filter(std::span<const double> samples,
    double sample_rate_hz);
} // namespace bike::numerics
