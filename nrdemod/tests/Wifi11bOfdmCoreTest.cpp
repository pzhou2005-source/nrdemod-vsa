/**
 ****************************************************************************
 *
 * Standalone checks for the native IEEE 802.11b DSSS/CCK core (no test
 * framework), mirroring Wifi11nOfdmCoreTest.cpp. Builds a synthetic 1 Mb/s
 * DBPSK/Barker PLCP preamble+header (SYNC + SFD + SIGNAL/SERVICE/LENGTH/CRC)
 * and verifies the receiver finds the burst, correlates the Barker-11
 * matched filter, measures a low EVM, and recovers an injected carrier
 * frequency offset.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11b/Wifi11bOfdmCore.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace wifi11bdemod;

namespace {

int gFailures = 0;

#define CHECK(cond) \
  do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++gFailures; } } while (0)

#define CHECK_NEAR(a, b, tol) \
  do { const double _a = (a), _b = (b); if (std::fabs(_a - _b) > (tol)) { \
    std::printf("FAIL %s:%d: %s = %g, expected %g (tol %g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); ++gFailures; } } while (0)

typedef std::complex<double> cd;
const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const int BARKER_LEN = 11;
const double BARKER[BARKER_LEN] = {1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1};

// Builds nSym 1 Mb/s DBPSK/Barker-spread symbols from a bit sequence (differential: bit=1 ->
// phase flip from the previous symbol, bit=0 -> no flip, matching 802.11-2016 15.2.3's DBPSK
// convention), each spread by the 11-chip Barker sequence at the chip rate implied by
// sampleRate.
std::vector<cd> buildDbpskBarkerSymbols(const std::vector<int>& bits, double carrierPhaseStart,
    double frequencyErrorHz, double sampleRate, int& phaseStateInOut)
{
  std::vector<cd> chips;
  int phaseState = phaseStateInOut; // 0 or 1 (quadrant: 0=+1, 1=-1 for BPSK)
  const double chipPeriod = 1.0 / 11.0e6;
  double phase = carrierPhaseStart;
  for (size_t b = 0; b < bits.size(); ++b) {
    if (bits[b]) phaseState ^= 1;
    const double sign = phaseState ? -1.0 : 1.0;
    for (int c = 0; c < BARKER_LEN; ++c) {
      const cd chip(sign * BARKER[c], 0.0);
      chips.push_back(chip * std::polar(1.0, phase));
      phase += TWO_PI * frequencyErrorHz * chipPeriod;
    }
  }
  phaseStateInOut = phaseState;
  return chips;
}

std::vector<cd> buildPreamble(double frequencyErrorHz, double sampleRate, double noiseAmplitude, unsigned seed)
{
  unsigned state = seed;
  auto uniform = [&]() { state = state * 1664525u + 1013904223u; return static_cast<double>(state >> 8) / 16777216.0; };

  // 128 scrambled "1" bits (SYNC), then SFD = 0xF3A0 transmitted LSB-first per 802.11-2016
  // 15.2.3 (bit pattern content is not actually checked by the receiver under test, which
  // only needs genuine Barker-spread DBPSK symbols at the right chip rate - any bit sequence
  // works equally well as a synthetic test signal).
  std::vector<int> bits;
  for (int i = 0; i < 128; ++i) bits.push_back(1);
  for (int i = 0; i < 64; ++i) bits.push_back(uniform() < 0.5 ? 1 : 0); // SFD+SIGNAL+SERVICE+LENGTH+CRC (64 bits)

  int phaseState = 0;
  std::vector<cd> chips = buildDbpskBarkerSymbols(bits, 0.3, frequencyErrorHz, sampleRate, phaseState);

  unsigned noiseState = seed ^ 0x5A5A5A5Au;
  auto noise = [&]() { noiseState = noiseState * 1664525u + 1013904223u; return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * noiseAmplitude; };
  for (auto& s : chips) s += cd(noise(), noise());
  return chips;
}

void testBurstSyncAndEvm()
{
  const double sampleRate = 11.0e6;
  std::vector<cd> preamble = buildPreamble(0.0, sampleRate, 0.01, 1u);
  std::vector<cd> capture(preamble.size() + 400, cd(0, 0));
  unsigned noiseState = 2u;
  auto noise = [&]() { noiseState = noiseState * 1664525u + 1013904223u; return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.005; };
  for (auto& s : capture) s = cd(noise(), noise());
  std::copy(preamble.begin(), preamble.end(), capture.begin() + 200);

  Config config;
  config.sampleRate = sampleRate;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.burstFound);
  CHECK(result.syncFound);
  CHECK(result.syncRho > 0.8);
  CHECK(result.rmsEvmPercent < 10.0);
  CHECK(result.numSyncSymbolsMeasured > 0);
}

void testCarrierFrequencyErrorRecovery()
{
  const double sampleRate = 11.0e6;
  const double injectedOffsetHz = 5e3;
  std::vector<cd> preamble = buildPreamble(injectedOffsetHz, sampleRate, 0.01, 3u);
  std::vector<cd> capture(preamble.size() + 400, cd(0, 0));
  unsigned noiseState = 4u;
  auto noise = [&]() { noiseState = noiseState * 1664525u + 1013904223u; return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.005; };
  for (auto& s : capture) s = cd(noise(), noise());
  std::copy(preamble.begin(), preamble.end(), capture.begin() + 200);

  Config config;
  config.sampleRate = sampleRate;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.syncFound);
  CHECK_NEAR(result.frequencyErrorHz, injectedOffsetHz, 200.0);
}

void testNoBurstWhenCaptureIsEmpty()
{
  Config config;
  config.sampleRate = 11.0e6;
  const Result result = demodulate(std::vector<cd>(), config);
  CHECK(!result.ok);
  CHECK(!result.error.empty());
}

void testNoiseOnlyCaptureFailsSync()
{
  const double sampleRate = 11.0e6;
  std::vector<cd> capture(4000);
  unsigned noiseState = 42u;
  auto noise = [&]() { noiseState = noiseState * 1664525u + 1013904223u; return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 1.0; };
  for (auto& s : capture) s = cd(noise(), noise());

  Config config;
  config.sampleRate = sampleRate;
  config.burstSearch = false;
  const Result result = demodulate(capture, config);
  CHECK(result.ok);
  CHECK(!result.syncFound);
}

}

int main()
{
  testBurstSyncAndEvm();
  testCarrierFrequencyErrorRecovery();
  testNoBurstWhenCaptureIsEmpty();
  testNoiseOnlyCaptureFailsSync();

  if (gFailures == 0) {
    std::printf("All WiFi 802.11b core tests passed\n");
    return 0;
  }
  std::printf("%d check(s) failed\n", gFailures);
  return 1;
}
