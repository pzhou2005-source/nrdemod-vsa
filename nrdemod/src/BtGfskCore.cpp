/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "bt/BtGfskCore.hpp"

#include <algorithm>
#include <cmath>

namespace btdemod {

namespace {

const double TWO_PI = 6.283185307179586476925286766559;

// Bluetooth Core Spec Vol 6 Part F (Direct Test Mode) fixed test-packet Access Address.
const uint32_t DEFAULT_ACCESS_ADDRESS = 0x71764129u;

double symbolRateHz(int phy)
{
  return phy == LE_2M ? 2.0e6 : 1.0e6;
}

// Windowed average power, one value per sample (edge-padded), used for burst start/end search.
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

// Instantaneous frequency via the FM discriminator: angle(x[n] * conj(x[n-1])) * Fs / (2*pi).
std::vector<double> instantaneousFrequencyHz(const std::vector<std::complex<double> >& iq, double sampleRate)
{
  const int n = static_cast<int>(iq.size());
  std::vector<double> freq(n, 0.0);
  for (int i = 1; i < n; ++i) {
    const std::complex<double> rotation = iq[i] * std::conj(iq[i - 1]);
    freq[i] = std::arg(rotation) * sampleRate / TWO_PI;
  }
  if (n > 1) freq[0] = freq[1];
  return freq;
}

// Scores a candidate sample phase by how strongly the resulting symbol stream alternates
// over the standard preamble (Core Spec Vol 6 Part B, 2.3.1: alternating 1-0 pattern used for
// bit/symbol synchronisation on both LE1M and LE2M). Returns the fraction of consecutive
// symbol pairs that differ in sign (1.0 = perfectly alternating).
double preambleAlternationScore(const std::vector<double>& freqHz, int phase, double samplesPerSymbol,
    int probeSymbols)
{
  int matches = 0, total = 0;
  int previousSign = 0;
  for (int symbol = 0; symbol < probeSymbols; ++symbol) {
    const int sampleIndex = phase + static_cast<int>(std::llround(symbol * samplesPerSymbol));
    if (sampleIndex < 0 || sampleIndex >= static_cast<int>(freqHz.size())) break;
    const int sign = freqHz[sampleIndex] >= 0.0 ? 1 : -1;
    if (symbol > 0) {
      ++total;
      if (sign != previousSign) ++matches;
    }
    previousSign = sign;
  }
  return total > 0 ? static_cast<double>(matches) / total : 0.0;
}

// Demodulates one bit per symbol period from the FM-discriminated signal: GFSK maps a binary
// one to positive frequency deviation and a binary zero to negative deviation (Core Spec Vol 6
// Part B, 2.3: "a positive frequency deviation... represents a binary one").
std::vector<int> symbolDecisions(const std::vector<double>& freqHz, int phase, double samplesPerSymbol,
    int burstStart, int burstEnd)
{
  std::vector<int> bits;
  for (int sampleIndex = burstStart + phase; sampleIndex < burstEnd;) {
    bits.push_back(freqHz[sampleIndex] >= 0.0 ? 1 : 0);
    sampleIndex += std::max(1, static_cast<int>(std::llround(samplesPerSymbol)));
  }
  return bits;
}

// Best bit-position match of the Access Address (LSB first on air, Core Spec Vol 6 Part B,
// 2.1: "the LSB... shall be transmitted first") within the demodulated bit stream.
void correlateAccessAddress(const std::vector<int>& bits, uint32_t accessAddress, int& bestPosition,
    double& bestRho, std::vector<int>& bestBits)
{
  int referenceBits[32];
  for (int i = 0; i < 32; ++i) referenceBits[i] = (accessAddress >> i) & 1;

  bestPosition = -1;
  bestRho = 0.0;
  for (int start = 0; start + 32 <= static_cast<int>(bits.size()); ++start) {
    int match = 0;
    for (int i = 0; i < 32; ++i) {
      if (bits[start + i] == referenceBits[i]) ++match;
    }
    const double rho = match / 32.0;
    if (rho > bestRho) {
      bestRho = rho;
      bestPosition = start;
    }
  }
  bestBits.clear();
  if (bestPosition >= 0) {
    bestBits.assign(bits.begin() + bestPosition, bits.begin() + bestPosition + 32);
  }
}

}

Config::Config()
    : sampleRate(0.0),
      phy(LE_1M),
      modulationIndex(0.5),
      burstSearch(true),
      burstSearchThresholdDb(15.0),
      accessAddress(DEFAULT_ACCESS_ADDRESS),
      syncThreshold(0.85)
{
}

Result::Result()
    : ok(false),
      burstFound(false),
      burstStart(0),
      burstLength(0),
      syncWordFound(false),
      syncWordRho(0.0),
      syncPositionSamples(0),
      frequencyErrorHz(0.0),
      deltaFAvgHz(0.0),
      deltaFMaxHz(0.0),
      deltaFMinHz(0.0),
      samplesPerSymbol(0.0),
      symbolTimingPhase(0)
{
}

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config)
{
  Result result;
  if (iq.empty() || config.sampleRate <= 0.0) {
    result.error = "empty capture or invalid sample rate";
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

  const std::vector<double> freqHz = instantaneousFrequencyHz(iq, config.sampleRate);
  result.frequencyDeviationHz.assign(freqHz.begin() + burstStart, freqHz.begin() + burstStart + burstLength);

  const double samplesPerSymbol = config.sampleRate / symbolRateHz(config.phy);
  result.samplesPerSymbol = samplesPerSymbol;

  // Lock the sample phase against the alternating preamble before any bit decisions.
  const int probeSymbols = std::min(16, static_cast<int>(burstLength / samplesPerSymbol));
  int bestPhase = 0;
  double bestScore = -1.0;
  const int phaseSteps = std::max(1, static_cast<int>(std::llround(samplesPerSymbol)));
  for (int phase = 0; phase < phaseSteps; ++phase) {
    const double score =
        preambleAlternationScore(result.frequencyDeviationHz, phase, samplesPerSymbol, probeSymbols);
    if (score > bestScore) {
      bestScore = score;
      bestPhase = phase;
    }
  }
  result.symbolTimingPhase = bestPhase;

  const std::vector<int> bits =
      symbolDecisions(freqHz, bestPhase, samplesPerSymbol, burstStart, burstStart + burstLength);

  int accessAddressPosition = -1;
  double accessAddressRho = 0.0;
  std::vector<int> accessAddressBits;
  correlateAccessAddress(bits, config.accessAddress, accessAddressPosition, accessAddressRho, accessAddressBits);
  result.syncWordRho = accessAddressRho;
  result.syncWordFound = accessAddressRho >= config.syncThreshold;
  result.accessAddressBits = accessAddressBits;
  if (accessAddressPosition >= 0) {
    result.syncPositionSamples =
        burstStart + bestPhase + static_cast<int>(std::llround(accessAddressPosition * samplesPerSymbol));
  }

  // Carrier frequency error: mean instantaneous frequency over the preamble, which by
  // construction has zero mean modulating data (alternating +/-1 symbols), so any residual
  // mean is the carrier offset (same quantity the legacy library reports as F0/Icft).
  const int preambleSamples = static_cast<int>(std::llround(8 * samplesPerSymbol));
  double freqSum = 0.0;
  int freqCount = 0;
  for (int i = burstStart; i < burstStart + std::min(preambleSamples, burstLength); ++i) {
    freqSum += freqHz[i];
    ++freqCount;
  }
  result.frequencyErrorHz = freqCount > 0 ? freqSum / freqCount : 0.0;

  // Frequency deviation statistics over the full burst, carrier offset removed (DeltaFAvg/
  // Max/Min, the modulation-accuracy quantities a VSA reports for GFSK).
  double devSum = 0.0, devMax = -1e300, devMin = 1e300;
  for (int i = burstStart; i < burstStart + burstLength; ++i) {
    const double deviation = std::fabs(freqHz[i] - result.frequencyErrorHz);
    devSum += deviation;
    devMax = std::max(devMax, deviation);
    devMin = std::min(devMin, deviation);
  }
  result.deltaFAvgHz = burstLength > 0 ? devSum / burstLength : 0.0;
  result.deltaFMaxHz = burstLength > 0 ? devMax : 0.0;
  result.deltaFMinHz = burstLength > 0 ? devMin : 0.0;

  result.ok = true;
  return result;
}

}
