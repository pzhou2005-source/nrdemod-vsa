/**
 ****************************************************************************
 *
 * Standalone checks for the native IEEE 802.11be (EHT) burst/sync/CFO-only
 * core (no test framework), mirroring Wifi11axOfdmCoreTest.cpp: this core
 * only decodes the legacy preamble (L-STF/L-LTF), so the synthetic packet
 * only needs those two fields plus enough trailing filler samples to look
 * like a real burst to the power-threshold burst detector.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11be/Wifi11beOfdmCore.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace wifi11bedemod;

namespace {

int gFailures = 0;

#define CHECK(cond) \
  do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++gFailures; } } while (0)

typedef std::complex<double> cd;
const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const int FFT_SIZE = 64;

const int LTF_SYM[53] = {
     1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,
     1,  0,
     1, -1, -1,  1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,  1, -1, -1,  1, -1,  1, -1,  1,  1,  1,  1
};
double ltfValue(int k) { return (k < -26 || k > 26) ? 0.0 : static_cast<double>(LTF_SYM[k + 26]); }
int bin(int k) { return ((k % FFT_SIZE) + FFT_SIZE) % FFT_SIZE; }

void fft64(std::vector<cd>& a, bool inverse)
{
  const size_t n = a.size();
  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t b = n >> 1;
    for (; j & b; b >>= 1) j ^= b;
    j ^= b;
    if (i < j) std::swap(a[i], a[j]);
  }
  for (size_t len = 2; len <= n; len <<= 1) {
    const double ang = TWO_PI / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
    const cd wlen(std::cos(ang), std::sin(ang));
    for (size_t i = 0; i < n; i += len) {
      cd w(1.0, 0.0);
      for (size_t j = 0; j < len / 2; ++j) {
        const cd u = a[i + j];
        const cd v = a[i + j + len / 2] * w;
        a[i + j] = u + v;
        a[i + j + len / 2] = u - v;
        w *= wlen;
      }
    }
  }
}

std::vector<cd> ofdmSymbolFromFreq(const std::vector<cd>& freq)
{
  std::vector<cd> time = freq;
  fft64(time, true);
  const double scale = 1.0 / std::sqrt(static_cast<double>(FFT_SIZE));
  for (auto& v : time) v *= scale;
  return time;
}

// Builds a minimal legacy preamble: L-STF + L-LTF (the only two fields this core decodes),
// optionally with a known carrier frequency offset and additive noise, surrounded by a
// trailing filler burst so the power-threshold burst detector sees a plausible burst length.
std::vector<cd> buildBePreamble(double frequencyErrorHz, double sampleRate, double noiseAmplitude, unsigned seed)
{
  unsigned state = seed;
  auto uniform = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0;
  };

  std::vector<cd> stf;
  std::vector<cd> stfToken(16);
  for (auto& v : stfToken) v = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  for (int r = 0; r < 10; ++r) stf.insert(stf.end(), stfToken.begin(), stfToken.end());

  std::vector<cd> ltfFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) {
    const double v = ltfValue(k);
    if (v != 0.0) ltfFreq[bin(k)] = cd(v, 0);
  }
  std::vector<cd> ltfTime = ofdmSymbolFromFreq(ltfFreq);
  std::vector<cd> ltfWithGi(32 + 2 * FFT_SIZE);
  for (int i = 0; i < 32; ++i) ltfWithGi[i] = ltfTime[FFT_SIZE - 32 + i];
  for (int i = 0; i < FFT_SIZE; ++i) {
    ltfWithGi[32 + i] = ltfTime[i];
    ltfWithGi[32 + FFT_SIZE + i] = ltfTime[i];
  }

  std::vector<cd> packet;
  packet.insert(packet.end(), stf.begin(), stf.end());
  packet.insert(packet.end(), ltfWithGi.begin(), ltfWithGi.end());
  // Filler data (unused by the core, just gives the burst detector a plausible tail length).
  for (int i = 0; i < 4 * FFT_SIZE; ++i) {
    packet.push_back(uniform() < 0.5 ? cd(1, 0) : cd(-1, 0));
  }

  for (size_t i = 0; i < packet.size(); ++i) {
    const double phase = TWO_PI * frequencyErrorHz * static_cast<double>(i) / sampleRate;
    packet[i] *= std::polar(1.0, phase);
  }
  if (noiseAmplitude > 0.0) {
    for (auto& v : packet) {
      v += cd((uniform() - 0.5) * 2.0 * noiseAmplitude, (uniform() - 0.5) * 2.0 * noiseAmplitude);
    }
  }
  return packet;
}

std::vector<cd> withIdlePadding(const std::vector<cd>& burst, int idleBefore, int idleAfter)
{
  std::vector<cd> out(idleBefore, cd(0, 0));
  out.insert(out.end(), burst.begin(), burst.end());
  out.insert(out.end(), idleAfter, cd(0, 0));
  return out;
}

void testBurstSyncRecovery()
{
  const double sampleRate = 20.0e6;
  const std::vector<cd> burst = buildBePreamble(0.0, sampleRate, 0.0, 12345u);
  const std::vector<cd> iq = withIdlePadding(burst, 50, 50);

  Config config;
  config.sampleRate = sampleRate;
  const Result result = demodulate(iq, config);

  CHECK(result.ok);
  CHECK(result.burstFound);
  CHECK(result.syncFound);
  CHECK(result.syncRho > 0.95);
}

void testCarrierFrequencyErrorRecovery()
{
  const double sampleRate = 20.0e6;
  const double injectedOffsetHz = 50000.0;
  const std::vector<cd> burst = buildBePreamble(injectedOffsetHz, sampleRate, 0.0, 999u);
  const std::vector<cd> iq = withIdlePadding(burst, 50, 50);

  Config config;
  config.sampleRate = sampleRate;
  const Result result = demodulate(iq, config);

  CHECK(result.ok);
  CHECK(result.syncFound);
  CHECK(std::fabs(result.frequencyErrorHz - injectedOffsetHz) < 2000.0);
}

void testNoiseRejection()
{
  const double sampleRate = 20.0e6;
  std::vector<cd> noise(2000);
  unsigned state = 42u;
  auto uniform = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0;
  };
  for (auto& v : noise) v = cd((uniform() - 0.5) * 2.0, (uniform() - 0.5) * 2.0);

  Config config;
  config.sampleRate = sampleRate;
  config.burstSearch = false;
  const Result result = demodulate(noise, config);

  CHECK(result.ok);
  CHECK(!result.syncFound);
}

}

int main()
{
  testBurstSyncRecovery();
  testCarrierFrequencyErrorRecovery();
  testNoiseRejection();

  if (gFailures == 0) {
    std::printf("All Wifi11beOfdmCore tests PASSED\n");
    return 0;
  }
  std::printf("%d Wifi11beOfdmCore test(s) FAILED\n", gFailures);
  return 1;
}
