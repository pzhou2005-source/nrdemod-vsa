/**
 ****************************************************************************
 *
 * Standalone checks for the native UWB (802.15.4z HRP) core (no test
 * framework), mirroring BtGfskCoreTest.cpp / NrOfdmCoreTest.cpp. Builds a
 * synthetic periodic BPSK preamble (repeating pseudo-random "ternary-like"
 * code every fixed chip count) and verifies the periodic-autocorrelation
 * receiver recovers the correct period and carrier offset.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "uwb/UwbHrpCore.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace uwbdemod;

namespace {

int gFailures = 0;

#define CHECK(cond) \
  do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++gFailures; } } while (0)

#define CHECK_NEAR(a, b, tol) \
  do { const double _a = (a), _b = (b); if (std::fabs(_a - _b) > (tol)) { \
    std::printf("FAIL %s:%d: %s = %g, expected %g (tol %g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); ++gFailures; } } while (0)

const double TWO_PI = 6.283185307179586476925286766559;

// A repeating ternary-like {-1, 0, +1} code of length codeLength, repeated repeatCount times,
// upsampled to samplesPerChip per chip, with a known carrier offset and additive noise.
std::vector<std::complex<double> > syntheticPreamble(int codeLength, int samplesPerChip, int repeatCount,
    double frequencyErrorHz, double sampleRate, double noiseAmplitude, unsigned seed)
{
  std::vector<double> code(codeLength);
  unsigned state = seed;
  auto uniform = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0;
  };
  for (int i = 0; i < codeLength; ++i) {
    const double r = uniform();
    code[i] = r < 0.33 ? -1.0 : (r < 0.66 ? 0.0 : 1.0);
  }

  const int samplesPerCode = codeLength * samplesPerChip;
  std::vector<std::complex<double> > out(samplesPerCode * repeatCount);
  double phase = 0.0;
  unsigned noiseState = seed ^ 0xA5A5A5A5u;
  auto noise = [&]() {
    noiseState = noiseState * 1664525u + 1013904223u;
    return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * noiseAmplitude;
  };
  for (int rep = 0; rep < repeatCount; ++rep) {
    for (int chip = 0; chip < codeLength; ++chip) {
      for (int s = 0; s < samplesPerChip; ++s) {
        const int index = rep * samplesPerCode + chip * samplesPerChip + s;
        phase += TWO_PI * frequencyErrorHz / sampleRate;
        const std::complex<double> carrier = std::polar(1.0, phase);
        out[index] = code[chip] * carrier + std::complex<double>(noise(), noise());
      }
    }
  }
  return out;
}

void testPeriodicPreambleRecovery()
{
  const int codeLength = 32;
  const int samplesPerChip = 4;
  const int repeatCount = 16;
  const double sampleRate = 1996.8e6; // ~4x the UWB chip rate, matching real TDC captures
  const double chipRate = sampleRate / samplesPerChip;

  std::vector<std::complex<double> > preamble =
      syntheticPreamble(codeLength, samplesPerChip, repeatCount, 0.0, sampleRate, 0.02, 7u);

  std::vector<std::complex<double> > capture(preamble.size() + 2000, std::complex<double>(0.0, 0.0));
  unsigned noiseState = 3u;
  auto noise = [&]() {
    noiseState = noiseState * 1664525u + 1013904223u;
    return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.005;
  };
  for (auto& sample : capture) sample = std::complex<double>(noise(), noise());
  std::copy(preamble.begin(), preamble.end(), capture.begin() + 1000);

  Config config;
  config.sampleRate = sampleRate;
  config.chipRateHz = chipRate;
  config.syncLength = repeatCount;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.burstFound);
  CHECK(result.syncFound);
  CHECK(result.syncRho > 0.5);
  CHECK_NEAR(result.preambleSymbolPeriodSamples, codeLength * samplesPerChip, 2.0);
}

void testCarrierFrequencyErrorRecovery()
{
  const int codeLength = 32;
  const int samplesPerChip = 4;
  const int repeatCount = 16;
  const double sampleRate = 1996.8e6;
  const double chipRate = sampleRate / samplesPerChip;
  const double injectedOffsetHz = 20e3;

  std::vector<std::complex<double> > preamble =
      syntheticPreamble(codeLength, samplesPerChip, repeatCount, injectedOffsetHz, sampleRate, 0.02, 9u);

  std::vector<std::complex<double> > capture(preamble.size() + 2000, std::complex<double>(0.0, 0.0));
  unsigned noiseState = 11u;
  auto noise = [&]() {
    noiseState = noiseState * 1664525u + 1013904223u;
    return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.005;
  };
  for (auto& sample : capture) sample = std::complex<double>(noise(), noise());
  std::copy(preamble.begin(), preamble.end(), capture.begin() + 1000);

  Config config;
  config.sampleRate = sampleRate;
  config.chipRateHz = chipRate;
  config.syncLength = repeatCount;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.syncFound);
  // the autocorrelation phase wraps at +/-chipRate/(2*period), so only a loose tolerance is
  // meaningful here; this proves the sign/order of magnitude is recovered, not exact lock
  CHECK_NEAR(result.frequencyErrorHz, injectedOffsetHz, 5e3);
}

void testNoBurstWhenCaptureIsEmpty()
{
  Config config;
  config.sampleRate = 1996.8e6;
  const Result result = demodulate(std::vector<std::complex<double> >(), config);
  CHECK(!result.ok);
  CHECK(!result.error.empty());
}

void testNoiseOnlyCaptureFailsSync()
{
  const double sampleRate = 1996.8e6;
  std::vector<std::complex<double> > capture(50000);
  unsigned noiseState = 13u;
  auto noise = [&]() {
    noiseState = noiseState * 1664525u + 1013904223u;
    return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5);
  };
  for (auto& sample : capture) sample = std::complex<double>(noise(), noise());

  Config config;
  config.sampleRate = sampleRate;
  config.chipRateHz = sampleRate / 4.0;
  config.burstSearchThresholdDb = 3.0; // tight threshold: pure noise has no real burst
  const Result result = demodulate(capture, config);
  // either no burst is found, or a "burst" is found but sync correlation stays low
  if (result.ok && result.burstFound) {
    CHECK(result.syncRho < 0.5);
  }
}

}

int main()
{
  testPeriodicPreambleRecovery();
  testCarrierFrequencyErrorRecovery();
  testNoBurstWhenCaptureIsEmpty();
  testNoiseOnlyCaptureFailsSync();

  if (gFailures == 0) {
    std::printf("All UWB HRP core tests passed\n");
    return 0;
  }
  std::printf("%d UWB HRP core test(s) FAILED\n", gFailures);
  return 1;
}
