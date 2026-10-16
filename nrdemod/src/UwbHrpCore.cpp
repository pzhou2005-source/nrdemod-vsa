/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "uwb/UwbHrpCore.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uwbdemod {

namespace {

const double TWO_PI = 6.283185307179586476925286766559;

std::vector<double> windowedPowerDb(const std::vector<std::complex<double> >& iq, int window)
{
  const int n = static_cast<int>(iq.size());
  std::vector<double> power(n, 0.0);
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += std::norm(iq[i]);
    if (i >= window) sum -= std::norm(iq[i - window]);
    const int count = std::min(i + 1, window);
    const double mean = count > 0 ? sum / count : 0.0;
    power[i] = 10.0 * std::log10(mean > 0.0 ? mean : 1e-30);
  }
  return power;
}

bool findBurst(const std::vector<std::complex<double> >& iq, double thresholdDb, int& start, int& length)
{
  const int n = static_cast<int>(iq.size());
  if (n == 0) return false;
  const int window = std::max(4, n / 200);
  std::vector<double> powerDb = windowedPowerDb(iq, window);
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

// Normalized periodic autocorrelation of the pulse ENERGY (not raw IQ) at a given lag over
// [start, start+windowLen). SHR repeats the same signed ternary code; energy correlation
// avoids carrier rotation during the blind period search. The code-aware data receiver
// separately checks the complex preamble and SFD before attempting packet decoding.
//
// Takes a precomputed power[] (= |iq|^2) and prefixPowerSq (running sum of power[i]^2) so the
// caller can build both ONCE outside the lag-search loop below instead of this function
// recomputing std::norm() and the energyA/energyB running sums from scratch for every one of the
// ~4000 candidate lags searched - that redundant O(windowLen) work per lag was the dominant cost
// of UwbHrpCore::demodulate() (measured ~2x of a real capture's total native runtime).
double periodicEnergyCorrelation(const std::vector<double>& power, const std::vector<double>& prefixPower,
                                  const std::vector<double>& prefixPowerSq, int start, int windowLen, int lag, int n)
{
  const int end = std::min(start + windowLen, n - lag);
  if (end <= start) return 0.0;
  double sum = 0.0;
  for (int i = start; i < end; ++i) sum += power[i] * power[i + lag];
  // Centred (Pearson) form: the uncentred product of positive power samples is ~0.5 even for pure noise.
  const double count = static_cast<double>(end - start);
  const double meanA = (prefixPower[end] - prefixPower[start]) / count;
  const double meanB = (prefixPower[end + lag] - prefixPower[start + lag]) / count;
  const double covariance = sum - count * meanA * meanB;
  const double varA = (prefixPowerSq[end] - prefixPowerSq[start]) - count * meanA * meanA;
  const double varB = (prefixPowerSq[end + lag] - prefixPowerSq[start + lag]) - count * meanB * meanB;
  const double denominator = std::sqrt(std::max(varA, 0.0) * std::max(varB, 0.0));
  return denominator > 0.0 ? covariance / denominator : 0.0;
}

// Carrier frequency error from the complex phase rotation between the burst start and one
// preamble period later, restricted to samples where both repetitions have significant pulse
// energy (so the comparison is dominated by genuine carrier rotation rather than noise phase).
double residualCarrierPhase(const std::vector<std::complex<double> >& iq, int start, int windowLen, int lag,
    double energyThreshold)
{
  const int n = static_cast<int>(iq.size());
  std::complex<double> sum(0.0, 0.0);
  for (int i = start; i < start + windowLen && i + lag < n; ++i) {
    const double powerA = std::norm(iq[i]);
    const double powerB = std::norm(iq[i + lag]);
    if (powerA > energyThreshold && powerB > energyThreshold) {
      sum += iq[i] * std::conj(iq[i + lag]);
    }
  }
  return std::arg(sum);
}

}

Config::Config()
    : sampleRate(0.0),
      phyMode(BPRF),
      syncLength(64),
      chipRateHz(499.2e6),
      burstSearch(true),
      burstSearchThresholdDb(15.0),
      syncThreshold(0.5)
{
}

Result::Result()
    : ok(false),
      burstFound(false),
      burstStart(0),
      burstLength(0),
      syncFound(false),
      syncRho(0.0),
      syncPositionSamples(0),
      preambleSymbolPeriodSamples(0.0),
      frequencyErrorHz(0.0),
      sfdFound(false),
      phrDecoded(false),
      psduDecoded(false),
      rsOk(false),
      preambleSymbols(0),
      sfdSymbol(-1),
      phrStartSamples(0),
      psduLengthBytes(0),
      dataRateKbps(0),
      rsCorrectedSymbols(0),
      evmDb(std::numeric_limits<double>::quiet_NaN())
{
}

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config)
{
  Result result;
  if (iq.empty() || iq.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    result.error = "empty or oversized capture";
    return result;
  }
  if (!std::isfinite(config.sampleRate) || config.sampleRate <= 0.0 ||
      !std::isfinite(config.chipRateHz) || config.chipRateHz <= 0.0 ||
      !std::isfinite(config.syncThreshold) || config.syncThreshold <= 0.0 || config.syncThreshold > 1.0 ||
      !std::isfinite(config.burstSearchThresholdDb) || config.burstSearchThresholdDb < 0.0 ||
      config.syncLength < 2 || config.syncLength > 4096 ||
      config.phyMode < BPRF || config.phyMode > IEEE802154A ||
      config.sampleRate / config.chipRateHz > 1024.0) {
    result.error = "invalid HRP acquisition configuration";
    return result;
  }
  double inputEnergy = 0.0;
  for (size_t i = 0; i < iq.size(); ++i) {
    if (!std::isfinite(iq[i].real()) || !std::isfinite(iq[i].imag()) ||
        !std::isfinite(std::norm(iq[i]))) {
      result.error = "non-finite IQ sample";
      return result;
    }
    inputEnergy += std::norm(iq[i]);
  }
  if (!(inputEnergy > 0.0) || !std::isfinite(inputEnergy)) {
    result.error = "capture has zero or invalid energy";
    return result;
  }

  int burstStart = 0, burstLength = static_cast<int>(iq.size());
  if (config.burstSearch) {
    if (!findBurst(iq, config.burstSearchThresholdDb, burstStart, burstLength)) {
      result.error = "burst not found";
      return result;
    }
    result.burstFound = true;
  } else {
    result.burstFound = true;
  }
  result.burstStart = burstStart;
  result.burstLength = burstLength;

  const double samplesPerChip = config.sampleRate / config.chipRateHz;
  // Blind acquisition searches an effective repeated-symbol period, including spreading.
  const int minLagSamples = std::max(1, static_cast<int>(std::llround(16 * samplesPerChip)));
  const int maxLagSamples = static_cast<int>(std::llround(1024 * samplesPerChip));

  // Precompute |iq|^2 and a running sum-of-squares (energy prefix sum) ONCE, not per-lag - see
  // periodicEnergyCorrelation()'s comment for why this is the dominant cost of this function.
  const int n = static_cast<int>(iq.size());
  std::vector<double> power(n);
  for (int i = 0; i < n; ++i) power[i] = std::norm(iq[i]);
  std::vector<double> prefixPower(n + 1, 0.0);
  std::vector<double> prefixPowerSq(n + 1, 0.0);
  for (int i = 0; i < n; ++i) {
    prefixPower[i + 1] = prefixPower[i] + power[i];
    prefixPowerSq[i + 1] = prefixPowerSq[i] + power[i] * power[i];
  }

  // The SYNC field is syncLength repetitions of one code period, so the correlation window of a
  // candidate lag is syncLength * lag: a fixed window sized for the longest code runs into the SFD/PHR/
  // PSDU of short-code frames and washes the correlation out.
  double bestMagnitude = -1.0;
  int bestLag = minLagSamples;
  for (int lag = minLagSamples; lag <= maxLagSamples && lag < n-burstStart; ++lag) {
    const int lagWindow = static_cast<int>(std::min(static_cast<double>(burstLength),
        config.syncLength * static_cast<double>(lag)));
    if (lagWindow > n-burstStart-lag) break;
    const double magnitude = periodicEnergyCorrelation(power, prefixPower, prefixPowerSq, burstStart, lagWindow, lag, n);
    if (magnitude > bestMagnitude) {
      bestMagnitude = magnitude;
      bestLag = lag;
    }
  }
  const int windowLen = static_cast<int>(std::min(static_cast<double>(burstLength),
      config.syncLength * static_cast<double>(bestLag)));

  result.preambleSymbolPeriodSamples = bestLag;
  result.syncRho = std::max(0.0, bestMagnitude);
  result.syncFound = result.syncRho >= config.syncThreshold;
  result.syncPositionSamples = burstStart;

  // Carrier frequency error from the residual phase rotation over one preamble period, computed
  // only where both repetitions carry real pulse energy, rather than inter-pulse noise.
  double burstPeakPower = 0.0;
  for (int i = burstStart; i < burstStart + burstLength; ++i) burstPeakPower = std::max(burstPeakPower, std::norm(iq[i]));
  const double energyThreshold = burstPeakPower * 0.1;
  const double phase = bestLag > 0 ? residualCarrierPhase(iq, burstStart, windowLen, bestLag, energyThreshold) : 0.0;
  result.frequencyErrorHz = bestLag > 0 ? -phase * config.sampleRate / (TWO_PI * bestLag) : 0.0;

  result.ok = true;
  return result;
}

}
