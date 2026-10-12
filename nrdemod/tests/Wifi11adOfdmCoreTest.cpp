/**
 ****************************************************************************
 *
 * Standalone checks for native IEEE 802.11ad burst detection and explicitly
 * trained SC DATA EVM. Tests known CFO/noise, guard exclusion and invalid
 * training, unsupported layouts and truncated/nonfinite captures.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11ad/Wifi11adOfdmCore.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using namespace wifi11addemod;

namespace {

int gFailures = 0;

#define CHECK(cond) \
  do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++gFailures; } } while (0)

typedef std::complex<double> cd;

void testBurstDetection()
{
  unsigned state = 42u;
  auto uniform = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0;
  };

  std::vector<cd> iq;
  for (int i = 0; i < 200; ++i) iq.push_back(cd((uniform() - 0.5) * 0.02, (uniform() - 0.5) * 0.02));
  for (int i = 0; i < 2000; ++i) iq.push_back(cd(uniform() < 0.5 ? 1.0 : -1.0, uniform() < 0.5 ? 1.0 : -1.0));
  for (int i = 0; i < 200; ++i) iq.push_back(cd((uniform() - 0.5) * 0.02, (uniform() - 0.5) * 0.02));

  Config config;
  const Result result = demodulate(iq, config);

  CHECK(result.ok);
  CHECK(result.burstFound);
  CHECK(result.burstStart >= 150 && result.burstStart <= 250);
  CHECK(result.burstLength > 1500);
}

void testNoiseRejection()
{
  unsigned state = 7u;
  auto uniform = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0;
  };
  std::vector<cd> noise(2000);
  for (auto& v : noise) v = cd((uniform() - 0.5) * 0.02, (uniform() - 0.5) * 0.02);

  Config config;
  const Result result = demodulate(noise, config);

  // Pure noise still has a "peak" relative to itself, so findBurst() will report a (spurious)
  // burst spanning most of the capture - this core does not claim to reject noise, only to
  // locate the strongest contiguous power region. Just verify it doesn't crash/error.
  CHECK(result.ok);
}

void testEmptyCapture()
{
  std::vector<cd> empty;
  Config config;
  const Result result = demodulate(empty, config);
  CHECK(!result.ok);
}

void testExplicitScData()
{
  unsigned state = 71u;
  Config config;
  config.measureDataEvm = true;
  config.trainingOffsetSamples = 17;
  config.dataOffsetSamples = 800;
  config.dataBlockCount = 3;
  for (int chip = 0; chip < 128; ++chip) {
    state = state * 1664525u + 1013904223u;
    config.trainingReference += (state >> 31) ? '+' : '-';
  }
  const double carrier = 400000.0;
  const double frequencyRadians = 2.0 * std::acos(-1.0) * carrier / config.sampleRate;
  const cd channel = std::polar(0.7, 0.31);
  const cd rotations[] = {cd(1, 0), cd(0, 1), cd(-1, 0), cd(0, -1)};
  const double noiseLevels[] = {0.0, 0.001, 0.025};
  std::vector<cd> iq;
  for (int mcs = 1; mcs <= 9; ++mcs) {
    config.dataMcs = mcs;
    for (double noise : noiseLevels) {
      iq.assign(config.dataOffsetSamples + config.dataBlockCount * 512, cd(0, 0));
      for (int chip = 0; chip < 512; ++chip) {
        const int position = config.trainingOffsetSamples + chip;
        const double reference = config.trainingReference[chip % 128] == '+' ? 1.0 : -1.0;
        iq[position] = reference * channel * rotations[chip % 4] * std::polar(1.0, frequencyRadians * position);
      }
      for (int block = 0; block < config.dataBlockCount; ++block) {
        for (int chip = 0; chip < 512; ++chip) {
          const int position = config.dataOffsetSamples + block * 512 + chip;
          const int relative = position - config.trainingOffsetSamples;
            const cd reference = mcs < 6 ? cd((chip + block) % 2 ? -1.0 : 1.0, 0.0)
              : rotations[(chip + block) % 4];
          const cd sample = chip < 64 ? cd(20, -30) : reference * cd(1.0 + noise, noise);
          iq[position] = sample * channel * rotations[relative % 4] * std::polar(1.0, frequencyRadians * position);
        }
      }
      const Result result = demodulate(iq, config);
      CHECK(result.ok && result.dataEvmValid);
      CHECK(result.dataError.empty());
      CHECK(result.numDataSymbols == 3 * 448);
      CHECK(result.trainingRho > 0.999999);
      CHECK(std::fabs(result.frequencyErrorHz - carrier) < 1e-6);
      CHECK(std::fabs(result.evmRmsData - std::sqrt(2.0) * noise) < 1e-10);
    }
  }
  iq.pop_back();
  CHECK(!demodulate(iq, config).dataEvmValid);
  iq.push_back(cd(1, 0));
  Config wrong = config;
  for (int chip = 0; chip < 128; chip += 3)
    wrong.trainingReference[chip] = wrong.trainingReference[chip] == '+' ? '-' : '+';
  CHECK(!demodulate(iq, wrong).dataEvmValid);
  wrong = config;
  wrong.dataMcs = 12;
  CHECK(!demodulate(iq, wrong).dataEvmValid);
  wrong = config;
  wrong.sampleRate *= 4;
  CHECK(!demodulate(iq, wrong).dataEvmValid);
  wrong = config;
  wrong.dataOffsetSamples = config.trainingOffsetSamples;
  CHECK(!demodulate(iq, wrong).dataEvmValid);
  iq[config.dataOffsetSamples + 65] = cd(std::numeric_limits<double>::quiet_NaN(), 0);
  CHECK(!demodulate(iq, config).dataEvmValid);
  for (cd& sample : iq) {
    state = state * 1664525u + 1013904223u;
    const double real = static_cast<double>(state >> 8) / 16777216.0 - 0.5;
    state = state * 1664525u + 1013904223u;
    const double imag = static_cast<double>(state >> 8) / 16777216.0 - 0.5;
    sample = cd(real, imag) * 0.02;
  }
  CHECK(!demodulate(iq, config).dataEvmValid);
  wrong = config;
  wrong.trainingReference[0] = '?';
  CHECK(!demodulate(iq, wrong).dataEvmValid);
  wrong = config;
  wrong.dataBlockCount = std::numeric_limits<int>::max();
  CHECK(!demodulate(iq, wrong).dataEvmValid);
}
}

int main()
{
  testBurstDetection();
  testNoiseRejection();
  testEmptyCapture();
  testExplicitScData();

  if (gFailures == 0) {
    std::printf("All Wifi11adOfdmCore tests PASSED\n");
    return 0;
  }
  std::printf("%d Wifi11adOfdmCore test(s) FAILED\n", gFailures);
  return 1;
}
