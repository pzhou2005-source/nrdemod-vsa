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

#include "wifi11ax/Wifi11axOfdmCore.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

void testMatchedScoreReuse()
{
  for (unsigned seed : {7u, 23u, 99u}) {
    for (int length : {96, 127, 128, 129, 191, 192, 193, 256, 1024, 2048, 5003}) {
      std::vector<cd> iq = buildBePreamble(50000.0, 20.0e6, 0.001, seed);
      iq.resize(length, cd(0.01, -0.02));
      Config config;
      config.sampleRate = 20.0e6;
      config.burstSearch = false;
      wifi11axdemod::Config referenceConfig;
      referenceConfig.sampleRate = config.sampleRate;
      referenceConfig.burstSearch = config.burstSearch;
      referenceConfig.syncThreshold = config.syncThreshold;
      const Result actual = demodulate(iq, config);
      const wifi11axdemod::Result reference = wifi11axdemod::demodulate(iq, referenceConfig);
      CHECK(actual.ok == reference.ok);
      CHECK(actual.burstFound == reference.burstFound);
      CHECK(actual.burstStart == reference.burstStart);
      CHECK(actual.burstLength == reference.burstLength);
      CHECK(actual.syncFound == reference.syncFound);
      CHECK(actual.timingOffsetSamples == reference.timingOffsetSamples);
      CHECK(actual.syncRho == reference.syncRho);
      CHECK(actual.frequencyErrorHz == reference.frequencyErrorHz);
    }
  }
}

void benchmarkMatchedScoreReuse()
{
  if (!std::getenv("DEMOD_WIFI_SYNC_BENCHMARK"))
    return;
  const std::vector<cd> packet = buildBePreamble(50000.0, 20.0e6, 0.001, 23u);
  std::vector<cd> iq(2097152, cd(0.01, -0.02));
  for (std::size_t index = 0; index < packet.size(); ++index) {
    iq[index * 2] = packet[index];
    iq[index * 2 + 1] = packet[index];
  }
  Config config;
  config.sampleRate = 40.0e6;
  config.burstSearch = false;
  wifi11axdemod::Config referenceConfig;
  referenceConfig.sampleRate = config.sampleRate;
  referenceConfig.burstSearch = config.burstSearch;
  referenceConfig.syncThreshold = config.syncThreshold;
  std::vector<double> referenceTimes, cachedTimes;
  for (int iteration = 0; iteration < 11; ++iteration) {
    for (int order = 0; order < 2; ++order) {
      const bool cached = (iteration + order) % 2 != 0;
      const auto started = std::chrono::steady_clock::now();
      if (cached) {
        const Result result = demodulate(iq, config);
        CHECK(result.ok && result.syncFound);
      } else {
        const wifi11axdemod::Result result = wifi11axdemod::demodulate(iq, referenceConfig);
        CHECK(result.ok && result.syncFound);
      }
      const double elapsed =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      if (iteration >= 2)
        (cached ? cachedTimes : referenceTimes).push_back(elapsed);
    }
  }
  std::sort(referenceTimes.begin(), referenceTimes.end());
  std::sort(cachedTimes.begin(), cachedTimes.end());
  std::printf("WIFI_SYNC_BENCHMARK samples=%zu sampleRateHz=40000000 warmups=2 runs=9 referenceMedianMs=%.6f "
              "cachedMedianMs=%.6f throughputGainPercent=%.3f\n",
      iq.size(), referenceTimes[4], cachedTimes[4], 100.0 * (referenceTimes[4] / cachedTimes[4] - 1.0));
}

void testDataEvm()
{
  const int bitsByMcs[] = {1, 2, 2, 4, 4, 6, 6, 6, 8, 8, 10, 10, 12, 12};
  for (int mcs = 0; mcs < 14; ++mcs) {
    for (double noise : {0.0, 0.001}) {
      std::vector<cd> packet = buildBePreamble(0.0, 20.0e6, 0.0, 12345u);
      packet.resize(800, cd(0.0, 0.0));
      const int bits = bitsByMcs[mcs];
      const int levels = bits == 1 ? 2 : 1 << (bits / 2);
      const double scale = bits == 1 ? 1.0 : std::sqrt(2.0 * (levels * levels - 1) / 3.0);
      double errorPower = 0.0, referencePower = 0.0;
      for (int symbol = -1; symbol < 3; ++symbol) {
        std::vector<cd> spectrum(256, cd(0.0, 0.0));
        for (int tone = -122; tone <= 122; ++tone) {
          if (std::abs(tone) < 2)
            continue;
          const int index = (tone + 256) % 256;
          const bool pilot =
              std::abs(tone) == 22 || std::abs(tone) == 48 || std::abs(tone) == 90 || std::abs(tone) == 116;
          const cd channel = std::polar(0.7 + 0.002 * (tone + 122), 0.003 * tone);
          if (symbol < 0) {
            spectrum[index] = channel * (index % 2 ? -1.0 : 1.0);
          } else {
            const cd reference = pilot ? cd(1.0, 0.0)
                                       : cd(2 * ((index + symbol) % levels) - levels + 1,
                                             bits == 1 ? 0 : 2 * ((index * 3 + symbol) % levels) - levels + 1) /
                                             scale;
            const cd perturbation = pilot ? cd(0.0, 0.0) : cd(noise, -noise);
            spectrum[index] = (reference + perturbation) * channel * std::polar(1.0, 0.1 * (symbol + 1));
            if (!pilot) {
              errorPower += std::norm(perturbation);
              referencePower += std::norm(reference);
            }
          }
        }
        fft64(spectrum, true);
        for (auto& value : spectrum)
          value /= 256.0;
        const int guard = symbol < 0 ? 64 : 16;
        packet.insert(packet.end(), spectrum.end() - guard, spectrum.end());
        packet.insert(packet.end(), spectrum.begin(), spectrum.end());
      }
      for (size_t sample = 0; sample < packet.size(); ++sample)
        packet[sample] *= std::polar(0.8, TWO_PI * 50000.0 * sample / 20.0e6);
      Config config;
      config.sampleRate = 20.0e6;
      config.measureDataEvm = true;
      config.dataMcs = mcs;
      config.ehtLtfFftOffsetSamples = 864;
      config.dataFftOffsetSamples = 1136;
      config.numDataSymbols = 3;
      const Result result = demodulate(withIdlePadding(packet, 50, 50), config);
      CHECK(result.ok);
      CHECK(result.dataEvmValid);
      CHECK(result.numDataSymbols == 3);
      CHECK(std::fabs(result.evmRmsData - std::sqrt(errorPower / referencePower)) < 1e-9);
      wifi11axdemod::Config heConfig;
      heConfig.sampleRate = config.sampleRate;
      heConfig.dataConfig = config;
      const wifi11axdemod::Result heResult = wifi11axdemod::demodulate(withIdlePadding(packet, 50, 50), heConfig);
      CHECK(heResult.ok);
      if (mcs <= 11) {
        CHECK(heResult.dataResult.dataEvmValid);
        CHECK(heResult.dataResult.numDataSymbols == 3);
        CHECK(std::fabs(heResult.dataResult.evmRmsData - std::sqrt(errorPower / referencePower)) < 1e-9);
      } else {
        CHECK(!heResult.dataResult.dataEvmValid);
        CHECK(!heResult.dataResult.dataError.empty());
      }
      Config invalidReuseConfig = config;
      invalidReuseConfig.numSpatialStreams = 2;
      Result reused = result;
      measureDataEvm(withIdlePadding(packet, 50, 50), 50.0, invalidReuseConfig, reused);
      CHECK(!reused.dataEvmValid);
      CHECK(reused.numDataSymbols == 0);
      reused = result;
      measureDataEvm(std::vector<cd>(), 50.0, config, reused);
      CHECK(!reused.dataEvmValid);
      CHECK(!reused.dataError.empty());
      for (int unsupported = 0; unsupported < 3; ++unsupported) {
        Config invalidConfig = config;
        if (unsupported == 0)
          invalidConfig.dataBandwidthMHz = 40;
        if (unsupported == 1)
          invalidConfig.numSpatialStreams = 2;
        if (unsupported == 2)
          invalidConfig.ehtLtfType = 2;
        const Result invalid = demodulate(withIdlePadding(packet, 50, 50), invalidConfig);
        CHECK(invalid.ok);
        CHECK(!invalid.dataEvmValid);
        CHECK(!invalid.dataError.empty());
      }
      packet.resize(packet.size() - 128);
      const Result truncated = demodulate(withIdlePadding(packet, 50, 0), config);
      CHECK(truncated.ok);
      CHECK(!truncated.dataEvmValid);
      CHECK(truncated.dataError == "incomplete DATA field");
      const wifi11axdemod::Result truncatedHe = wifi11axdemod::demodulate(withIdlePadding(packet, 50, 0), heConfig);
      CHECK(!truncatedHe.dataResult.dataEvmValid);
    }
  }
}
}

int main()
{
  Config dataConfig;
  dataConfig.sampleRate = 20.0e6;
  dataConfig.measureDataEvm = true;
  const Result invalidData = demodulate(withIdlePadding(buildBePreamble(0.0, 20.0e6, 0.0, 12345u), 50, 50), dataConfig);
  CHECK(invalidData.ok);
  CHECK(invalidData.syncFound);
  CHECK(!invalidData.dataEvmValid);
  CHECK(!invalidData.dataError.empty());
  testBurstSyncRecovery();
  testCarrierFrequencyErrorRecovery();
  testNoiseRejection();
  testMatchedScoreReuse();
  testDataEvm();
  benchmarkMatchedScoreReuse();

  if (gFailures == 0) {
    std::printf("All Wifi11beOfdmCore tests PASSED\n");
    return 0;
  }
  std::printf("%d Wifi11beOfdmCore test(s) FAILED\n", gFailures);
  return 1;
}
