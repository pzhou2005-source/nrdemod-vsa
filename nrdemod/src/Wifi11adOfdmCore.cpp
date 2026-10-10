/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11ad/Wifi11adOfdmCore.hpp"

#include <algorithm>
#include <cmath>

namespace wifi11addemod {

namespace {

typedef std::complex<double> cd;

// Identical windowed-power burst detector used by every other native WiFi core in this
// codebase (wifi/WifiOfdmCore.cpp, wifi11ac/Wifi11acOfdmCore.cpp, etc.) - the one piece of
// this core that needs no knowledge of the 802.11ad-specific Golay STF/CE sequence content.
bool findBurst(const std::vector<cd>& iq, double thresholdDb, int& start, int& length)
{
  const int n = static_cast<int>(iq.size());
  if (n == 0) return false;
  const int window = std::max(8, n / 200);
  std::vector<double> powerDb(n);
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += std::norm(iq[i]);
    if (i >= window) sum -= std::norm(iq[i - window]);
    const int count = std::min(i + 1, window);
    const double mean = count > 0 ? sum / count : 0.0;
    powerDb[i] = 10.0 * std::log10(mean > 0.0 ? mean : 1e-30);
  }
  const double peakDb = *std::max_element(powerDb.begin(), powerDb.end());
  const double threshold = peakDb - std::fabs(thresholdDb);
  int first = -1, last = -1;
  for (int i = 0; i < n; ++i) {
    if (powerDb[i] >= threshold) {
      if (first < 0) first = i;
      last = i;
    }
  }
  if (first < 0) return false;
  start = first;
  length = last - first + 1;
  return length > 0;
}

}

Config::Config() : burstSearchThresholdDb(15.0)
{
}

Result::Result() : ok(false), burstFound(false), burstStart(0), burstLength(0)
{
}

Result demodulate(const std::vector<cd>& iq, const Config& config)
{
  Result result;
  if (iq.empty()) {
    result.error = "empty capture";
    return result;
  }

  int burstStart = 0, burstLength = static_cast<int>(iq.size());
  if (!findBurst(iq, config.burstSearchThresholdDb, burstStart, burstLength)) {
    result.error = "burst not found";
    return result;
  }
  result.burstFound = true;
  result.burstStart = burstStart;
  result.burstLength = burstLength;
  result.ok = true;
  return result;
}

}
