/**
 ****************************************************************************
 *
 * Standalone checks for the native Bluetooth LE GFSK core (no test
 * framework), mirroring NrOfdmCoreTest.cpp. Builds a synthetic LE1M/LE2M
 * packet (Gaussian-filtered FSK, Core Spec Vol 6 Part B 2.3) from known
 * ground-truth bits and verifies the receiver recovers them.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "bt/BtGfskCore.hpp"

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

using namespace btdemod;

namespace {

int gFailures = 0;

#define CHECK(cond) \
  do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++gFailures; } } while (0)

#define CHECK_NEAR(a, b, tol) \
  do { const double _a = (a), _b = (b); if (std::fabs(_a - _b) > (tol)) { \
    std::printf("FAIL %s:%d: %s = %g, expected %g (tol %g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); ++gFailures; } } while (0)

const double TWO_PI = 6.283185307179586476925286766559;

// Gaussian pulse shaping filter, BT = 0.5 (Core Spec Vol 6 Part B, 2.3.1 "Modulation
// characteristics" for LE1M/LE2M), truncated to +/-2 symbol periods.
std::vector<double> gaussianFilterTaps(double bt, int samplesPerSymbol, int symbolSpan)
{
  const double sigma = std::sqrt(std::log(2.0) / 2.0) / (bt * samplesPerSymbol);
  const int half = symbolSpan * samplesPerSymbol;
  std::vector<double> taps(2 * half + 1);
  double sum = 0.0;
  for (int i = -half; i <= half; ++i) {
    const double t = static_cast<double>(i);
    const double value = std::exp(-(t * t) / (2.0 * sigma * sigma));
    taps[i + half] = value;
    sum += value;
  }
  for (double& tap : taps) tap /= sum;
  return taps;
}

// Builds samplesPerSymbol-oversampled complex baseband GFSK for the given bits (LSB-first
// air order), modulation index h and symbol rate, then resamples (by repetition/decimation)
// to targetSampleRate. Returns IQ at targetSampleRate.
std::vector<std::complex<double> > modulateGfsk(const std::vector<int>& bits, int phy, double modulationIndex,
    double targetSampleRate, double frequencyErrorHz, double noiseAmplitude, unsigned seed)
{
  const double symbolRate = phy == LE_2M ? 2.0e6 : 1.0e6;
  const int oversample = 16;
  const double shapedRate = symbolRate * oversample;

  std::vector<double> nrz(bits.size() * oversample);
  for (size_t b = 0; b < bits.size(); ++b) {
    const double level = bits[b] ? 1.0 : -1.0;
    for (int s = 0; s < oversample; ++s) nrz[b * oversample + s] = level;
  }

  const std::vector<double> taps = gaussianFilterTaps(0.5, oversample, 2);
  const int half = static_cast<int>(taps.size() / 2);
  std::vector<double> shaped(nrz.size(), 0.0);
  for (size_t n = 0; n < nrz.size(); ++n) {
    double acc = 0.0;
    for (size_t k = 0; k < taps.size(); ++k) {
      const long index = static_cast<long>(n) + static_cast<long>(k) - half;
      if (index >= 0 && index < static_cast<long>(nrz.size())) acc += taps[k] * nrz[index];
    }
    shaped[n] = acc;
  }

  // Instantaneous frequency deviation: +/- h * symbolRate/2 at full deviation (h=0.5 -> +/-
  // symbolRate/4), integrated to phase, matching the receiver's inverse operation.
  const double peakDeviationHz = modulationIndex * symbolRate / 2.0;
  std::vector<std::complex<double> > shapedIq(shaped.size());
  double phase = 0.0;
  unsigned state = seed;
  auto uniform = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0 - 0.5;
  };
  for (size_t n = 0; n < shaped.size(); ++n) {
    const double instFreq = shaped[n] * peakDeviationHz + frequencyErrorHz;
    phase += TWO_PI * instFreq / shapedRate;
    std::complex<double> sample = std::polar(1.0, phase);
    sample += std::complex<double>(uniform() * noiseAmplitude, uniform() * noiseAmplitude);
    shapedIq[n] = sample;
  }

  // Resample shapedRate -> targetSampleRate by nearest-neighbour selection (sufficient for a
  // synthetic test signal; the receiver only needs the correct symbol timing grid).
  const double ratio = shapedRate / targetSampleRate;
  const int outLength = static_cast<int>(shapedIq.size() / ratio);
  std::vector<std::complex<double> > out(outLength);
  for (int n = 0; n < outLength; ++n) {
    const int src = std::min(static_cast<int>(shapedIq.size()) - 1, static_cast<int>(std::llround(n * ratio)));
    out[n] = shapedIq[src];
  }
  return out;
}

std::vector<int> bitsFromUint32LsbFirst(uint32_t value, int count)
{
  std::vector<int> bits(count);
  for (int i = 0; i < count; ++i) bits[i] = (value >> i) & 1;
  return bits;
}

void testLe1mAccessAddressRecovery()
{
  const uint32_t accessAddress = 0x71764129u;
  std::vector<int> bits;
  // Standard alternating preamble (Core Spec Vol 6 Part B, 2.3.1), 8 bits for LE1M.
  for (int i = 0; i < 8; ++i) bits.push_back(i % 2);
  const std::vector<int> aaBits = bitsFromUint32LsbFirst(accessAddress, 32);
  bits.insert(bits.end(), aaBits.begin(), aaBits.end());
  // Synthetic payload: pseudo-random bits after the Access Address.
  unsigned payloadState = 12345u;
  for (int i = 0; i < 200; ++i) {
    payloadState = payloadState * 1103515245u + 12345u;
    bits.push_back((payloadState >> 16) & 1);
  }

  const double sampleRate = 32e6; // typical VSA IF sample rate for a 1 MHz symbol rate capture
  std::vector<std::complex<double> > burst =
      modulateGfsk(bits, LE_1M, 0.5, sampleRate, /*frequencyErrorHz=*/0.0, /*noiseAmplitude=*/0.01, 42u);

  // Pad with low-level noise before/after the burst so burst search has something to find.
  std::vector<std::complex<double> > capture(burst.size() + 2000, std::complex<double>(0.0, 0.0));
  unsigned noiseState = 7u;
  auto noise = [&]() {
    noiseState = noiseState * 1664525u + 1013904223u;
    return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.001;
  };
  for (auto& sample : capture) sample = std::complex<double>(noise(), noise());
  std::copy(burst.begin(), burst.end(), capture.begin() + 1000);

  Config config;
  config.sampleRate = sampleRate;
  config.phy = LE_1M;
  config.accessAddress = accessAddress;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.burstFound);
  CHECK(result.syncWordFound);
  CHECK(result.syncWordRho > 0.9);
  // Burst should be found within the padded region (not spanning the whole capture).
  CHECK(result.burstStart > 500);
  CHECK(result.burstStart < 1500);
}

void testLe2mAccessAddressRecovery()
{
  const uint32_t accessAddress = 0x8e89bed6u; // Bluetooth Core Spec advertising channel AA
  std::vector<int> bits;
  for (int i = 0; i < 16; ++i) bits.push_back(i % 2); // LE2M uses a 16 bit preamble
  const std::vector<int> aaBits = bitsFromUint32LsbFirst(accessAddress, 32);
  bits.insert(bits.end(), aaBits.begin(), aaBits.end());
  unsigned payloadState = 999u;
  for (int i = 0; i < 200; ++i) {
    payloadState = payloadState * 1103515245u + 12345u;
    bits.push_back((payloadState >> 16) & 1);
  }

  const double sampleRate = 64e6; // 2x the LE1M test rate, keeping samples/symbol comparable
  std::vector<std::complex<double> > burst =
      modulateGfsk(bits, LE_2M, 0.5, sampleRate, /*frequencyErrorHz=*/50e3, /*noiseAmplitude=*/0.01, 99u);

  std::vector<std::complex<double> > capture(burst.size() + 2000, std::complex<double>(0.0, 0.0));
  unsigned noiseState = 3u;
  auto noise = [&]() {
    noiseState = noiseState * 1664525u + 1013904223u;
    return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.001;
  };
  for (auto& sample : capture) sample = std::complex<double>(noise(), noise());
  std::copy(burst.begin(), burst.end(), capture.begin() + 1000);

  Config config;
  config.sampleRate = sampleRate;
  config.phy = LE_2M;
  config.accessAddress = accessAddress;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.burstFound);
  CHECK(result.syncWordFound);
  CHECK(result.syncWordRho > 0.9);
  // 50 kHz carrier offset injected above should be recovered within a reasonable tolerance;
  // the preamble-mean estimator is coarse, so allow +/-15 kHz.
  CHECK_NEAR(result.frequencyErrorHz, 50e3, 15e3);
}

void testNoBurstWhenCaptureIsEmpty()
{
  Config config;
  config.sampleRate = 32e6;
  const Result result = demodulate(std::vector<std::complex<double> >(), config);
  CHECK(!result.ok);
  CHECK(!result.error.empty());
}

void testWrongAccessAddressFailsSync()
{
  const uint32_t accessAddress = 0x71764129u;
  std::vector<int> bits;
  for (int i = 0; i < 8; ++i) bits.push_back(i % 2);
  const std::vector<int> aaBits = bitsFromUint32LsbFirst(accessAddress, 32);
  bits.insert(bits.end(), aaBits.begin(), aaBits.end());
  for (int i = 0; i < 100; ++i) bits.push_back(i % 3 == 0);

  const double sampleRate = 32e6;
  std::vector<std::complex<double> > burst =
      modulateGfsk(bits, LE_1M, 0.5, sampleRate, 0.0, 0.01, 11u);
  std::vector<std::complex<double> > capture(burst.size() + 200, std::complex<double>(0.0, 0.0));
  std::copy(burst.begin(), burst.end(), capture.begin() + 100);

  Config config;
  config.sampleRate = sampleRate;
  config.phy = LE_1M;
  config.accessAddress = 0xDEADBEEFu; // deliberately wrong
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(!result.syncWordFound);
}

}

int main()
{
  testLe1mAccessAddressRecovery();
  testLe2mAccessAddressRecovery();
  testNoBurstWhenCaptureIsEmpty();
  testWrongAccessAddressFailsSync();

  if (gFailures == 0) {
    std::printf("All Bluetooth GFSK core tests passed\n");
    return 0;
  }
  std::printf("%d Bluetooth GFSK core test(s) FAILED\n", gFailures);
  return 1;
}
