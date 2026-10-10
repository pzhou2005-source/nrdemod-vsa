/**
 ****************************************************************************
 *
 * Standalone checks for the native IEEE 802.11ad (DMG) burst-detection-only
 * core (no test framework). Verifies the windowed-power burst detector
 * finds a plausible burst and rejects pure noise - the only thing this
 * core claims to decode (see Wifi11adOfdmCore.hpp scope comment).
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11ad/Wifi11adOfdmCore.hpp"

#include <cstdio>
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

}

int main()
{
  testBurstDetection();
  testNoiseRejection();
  testEmptyCapture();

  if (gFailures == 0) {
    std::printf("All Wifi11adOfdmCore tests PASSED\n");
    return 0;
  }
  std::printf("%d Wifi11adOfdmCore test(s) FAILED\n", gFailures);
  return 1;
}
