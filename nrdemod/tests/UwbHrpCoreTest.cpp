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
#include <algorithm>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
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

void testEhprfPreambleRecovery()
{
  CHECK(BPRF == 0);
  CHECK(HPRF == 1);
  CHECK(EHPRF == 2);
  const int codeLength = 508;
  const int repeatCount = 17;
  for (int samplesPerChip : {2, 4}) {
    for (double offsetHz : {-25000.0, 25000.0}) {
      Config config;
      config.phyMode = EHPRF;
      config.sampleRate = config.chipRateHz * samplesPerChip;
      config.syncLength = repeatCount - 1;
      config.burstSearch = false;
      const auto capture = syntheticPreamble(codeLength, samplesPerChip, repeatCount,
          offsetHz, config.sampleRate, 0.002, 37u);
      const Result result = demodulate(capture, config);
      CHECK(result.ok);
      CHECK(result.burstFound);
      CHECK(result.syncFound);
      CHECK(result.syncRho > 0.99);
      CHECK_NEAR(result.preambleSymbolPeriodSamples, codeLength * samplesPerChip, 1.0);
      CHECK(std::isfinite(result.frequencyErrorHz));
      CHECK_NEAR(result.frequencyErrorHz, offsetHz, 100.0);
    }
  }
}

void testUnsupportedPhyMode()
{
  Config config;
  config.sampleRate = 1996.8e6;
  const auto capture = syntheticPreamble(32, 4, 16, 0.0, config.sampleRate, 0.02, 7u);
  for (int mode : {-1, 3}) {
    config.phyMode = mode;
    const Result result = demodulate(capture, config);
    CHECK(!result.ok);
    CHECK(result.error == "unsupported UWB PHY mode");
  }
}

void testNoBurstWhenCaptureIsEmpty()
{
  Config config;
  config.sampleRate = 1996.8e6;
  const Result result = demodulate(std::vector<std::complex<double> >(), config);
  CHECK(!result.ok);
  CHECK(!result.error.empty());
}

void testMacFcsKnownVectors()
{
  std::vector<int> frame16 = {'1','2','3','4','5','6','7','8','9',0x89,0x21};
  std::vector<int> frame32 = {'1','2','3','4','5','6','7','8','9',0x26,0x39,0xf4,0xcb};
  CHECK(validateMacFcs(frame16, 2));
  CHECK(validateMacFcs(frame32, 4));
  frame16[2] ^= 1;
  frame32[2] ^= 1;
  CHECK(!validateMacFcs(frame16, 2));
  CHECK(!validateMacFcs(frame32, 4));
  CHECK(!validateMacFcs({}, 2));
  CHECK(!validateMacFcs({256, 0, 0}, 2));
  CHECK(!validateMacFcs(frame16, 0));
}

void testEmptyFrameIsRejected()
{
  Config config;
  config.sampleRate = 1996.8e6;
  FrameConfig frameConfig;
  const FrameResult result = demodulateFrame({}, config, frameConfig);
  CHECK(!result.ok);
  CHECK(!result.sfdFound);
  CHECK(!result.phrValid);
  CHECK(!result.psduValid);
  CHECK(!result.error.empty());
  for (double chipRate : {0.0, -1.0, std::nan(""), HUGE_VAL}) {
    config.chipRateHz = chipRate;
    const auto rejected = demodulateFrame(std::vector<std::complex<double>>(100, {1, 0}), config, frameConfig);
    CHECK(!rejected.ok);
    CHECK(!rejected.sfdFound);
    CHECK(!rejected.acquisition.burstFound);
    CHECK(!rejected.error.empty());
  }
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

void testCaptureFrame(const std::string& directory, const std::string& stem,
  bool legacy, int codeIndex, bool highRatePhr, int packetConfig, int phyMode = BPRF)
{
  std::ifstream stream(directory + "/" + stem + ".txt");
  CHECK(stream.good());
  std::string line;
  while (std::getline(stream, line)) if (line == "Y" || line == "Y\r") break;
  std::vector<std::complex<double>> iq;
  double real, imaginary;
  while (stream >> real >> imaginary) iq.emplace_back(real, imaginary);
  Config config;
  config.sampleRate = 1996.8e6;
  config.phyMode = phyMode;
  if (stem.find("7M8_LDPC") != std::string::npos) config.sampleRate = 998.4e6;
  config.chipRateHz = config.sampleRate / 4;
  FrameConfig frame;
  frame.legacy = legacy;
  frame.codeIndex = codeIndex;
  frame.highRatePhr = highRatePhr;
  frame.stsPacketConfig = packetConfig;
  if (phyMode == HPRF) frame.sfdIndex = 1;
  if (phyMode == EHPRF) frame.constraintLength = 7;
  if (codeIndex == 27) {
    config.syncLength = 32;
    frame.meanPrfMHz = 124.8;
    frame.constraintLength = 7;
    frame.numStsSegments = 2;
  }
  const auto result = demodulateFrame(iq, config, frame);
  std::printf("%s: ok=%d SFD=%g STS=%g PHR=%d PSDU=%d bytes=%zu error=%s\n",
      stem.c_str(), result.ok, result.sfdRho, result.stsRho, result.phrValid,
      result.psduValid, result.psduBytes.size(), result.error.c_str());
  std::printf("origin=%d payload=%d length=%d\n", result.syncStart, result.psduStart, result.psduLengthBytes);
  CHECK(result.ok);
  CHECK(result.sfdFound);
  CHECK(result.phrValid == (packetConfig != 3));
  CHECK(result.psduValid == (packetConfig != 3));
  CHECK(!packetConfig || result.stsValid);
  if (!result.ok) return;
  std::printf("EVM SHR=%g STS=%g PHR=%g PSDU=%g\n", result.shrEvmRmsPercent,
      result.stsEvmRmsPercent, result.phrEvmRmsPercent, result.psduEvmRmsPercent);
  CHECK(result.evmValid);
  CHECK(result.shrEvmRmsPercent < 0.1);
  CHECK(!result.stsValid || result.stsEvmRmsPercent < 0.1);
  CHECK(!result.phrValid || result.phrEvmRmsPercent < 0.1);
  CHECK(!result.psduValid || result.psduEvmRmsPercent < 0.1);
  if (result.psduValid) {
    auto scaled = iq;
    for (int index = result.psduStart; index < result.psduStart + result.psduLength; ++index) scaled[index] *= 1.1;
    const auto measured = demodulateFrame(scaled, config, frame);
    CHECK(measured.ok);
    CHECK(measured.psduBytes == result.psduBytes);
    CHECK(std::fabs(measured.psduEvmRmsPercent - 10) < 0.1);
    FrameConfig fcsConfig = frame;
    fcsConfig.macFcsLength = 2;
    const auto badFcs = demodulateFrame(iq, config, fcsConfig);
    CHECK(!badFcs.ok);
    CHECK(badFcs.psduValid);
    CHECK(badFcs.macFcsChecked);
    CHECK(!badFcs.macFcsValid);
  }
  if (result.ldpcCoding) {
    auto changed = iq;
    const int symbolSamples = frame.samplesPerPulse * (result.bitRateMbps < 8 ? 64 : 4);
    const int start = result.psduStart + 10 * symbolSamples;
    for (int index = start; index < start + symbolSamples; ++index) changed[index] *= -1;
    const auto corrected = demodulateFrame(changed, config, frame);
    CHECK(corrected.ok);
    CHECK(corrected.ldpcCorrectedBits > 0);
    CHECK(corrected.psduBytes == result.psduBytes);
  }
  if (phyMode == EHPRF) {
    CHECK(result.phr1Valid);
    CHECK(result.ldpcCoding == (stem.find("LDPC") != std::string::npos));
    auto damagedHeader = iq;
    std::fill(damagedHeader.begin() + result.phrStart,
        damagedHeader.begin() + result.phrStart + 2560 * frame.samplesPerPulse, std::complex<double>(0, 0));
    const auto rejectedHeader = demodulateFrame(damagedHeader, config, frame);
    CHECK(!rejectedHeader.ok);
    CHECK(!rejectedHeader.phrValid);
  }
  std::ifstream metadata(directory + "/" + stem + ".para2");
  std::string expectedHex;
  while (std::getline(metadata, line)) if (line.find("PayloadHex,") != std::string::npos) {
    expectedHex = line.substr(line.rfind(',') + 1);
    expectedHex.erase(0, expectedHex.find_first_not_of(" \t"));
    expectedHex.erase(expectedHex.find_last_not_of(" \t\r\n") + 1);
  }
  CHECK(result.psduBytes.size() * 2 == expectedHex.size());
  if (result.psduBytes.size() * 2 == expectedHex.size())
    for (size_t index = 0; index < result.psduBytes.size(); ++index)
      CHECK(result.psduBytes[index] == static_cast<int>(std::stoul(expectedHex.substr(2 * index, 2), nullptr, 16)));
  auto shifted = iq;
  for (size_t index = 0; index < shifted.size(); ++index)
    shifted[index] *= std::polar(1.0, TWO_PI * 25000 * index / config.sampleRate);
  const auto shiftedResult = demodulateFrame(shifted, config, frame);
  CHECK(shiftedResult.ok);
  CHECK(shiftedResult.psduBytes == result.psduBytes);
  if (result.stsPresent) {
    FrameConfig wrongKey = frame;
    wrongKey.stsKeyHex = std::string(32, '0');
    const auto rejected = demodulateFrame(iq, config, wrongKey);
    CHECK(!rejected.ok);
    CHECK(!rejected.stsValid);
    if (frame.numStsSegments > 1) {
      auto damaged = iq;
      const int segmentStart = result.stsStart + (512 + 32768 + 512) * frame.samplesPerPulse;
      std::fill(damaged.begin() + segmentStart,
          damaged.begin() + segmentStart + 32768 * frame.samplesPerPulse, std::complex<double>(0, 0));
      const auto missingSegment = demodulateFrame(damaged, config, frame);
      CHECK(!missingSegment.ok);
      CHECK(!missingSegment.stsValid);
    }
  }
  if (result.psduValid) {
    const auto truncated = demodulateFrame(std::vector<std::complex<double>>(
        iq.begin(), iq.begin() + result.psduStart + 100), config, frame);
    CHECK(!truncated.ok);
    CHECK(!truncated.psduValid);
  }
  if (codeIndex == 10 && result.psduValid) {
    const int symbolSamples = 8 * 8 * frame.samplesPerPulse;
    for (int errors : {1, 4, 5}) {
      auto changed = iq;
      for (int error = 0; error < errors; ++error) {
        const int word = 6 + 6 * error;
        for (int parityWord : {word, word + 2}) {
          const int start = result.psduStart + parityWord * symbolSamples;
          for (int index = start; index < start + symbolSamples; ++index) changed[index] *= -1;
        }
        const int start = result.psduStart + (word + 1) * symbolSamples;
        std::rotate(changed.begin() + start, changed.begin() + start + symbolSamples / 2,
            changed.begin() + start + symbolSamples);
      }
      const auto decoded = demodulateFrame(changed, config, frame);
      CHECK(decoded.ok == (errors <= 4));
      if (errors <= 4) {
        CHECK(decoded.rsCorrectedSymbols == errors);
        CHECK(decoded.psduBytes == result.psduBytes);
      } else {
        CHECK(!decoded.psduValid);
      }
    }
  }
  if (frame.constraintLength == 7 && result.psduValid && !result.ldpcCoding) {
    auto changed = iq;
    const int symbolSamples = result.psduLength / (8 * result.psduLengthBytes + 6);
    const int start = result.psduStart + 10 * symbolSamples;
    for (int index = start; index < start + symbolSamples; ++index) changed[index] *= -1;
    const auto decoded = demodulateFrame(changed, config, frame);
    CHECK(decoded.ok);
    CHECK(decoded.psduBytes == result.psduBytes);
  }
  if (result.sfdFound) {
    auto damaged = iq;
    std::fill(damaged.begin() + std::max(0, result.sfdStart - 16),
        damaged.begin() + std::min(static_cast<int>(iq.size()), result.sfdStart + result.sfdLength + 16),
        std::complex<double>(0, 0));
    const auto rejected = demodulateFrame(damaged, config, frame);
    CHECK(!rejected.ok);
    CHECK(!rejected.sfdFound);
  }
}

}

int main(int argc, char** argv)
{
  testPeriodicPreambleRecovery();
  testCarrierFrequencyErrorRecovery();
  testEhprfPreambleRecovery();
  testUnsupportedPhyMode();
  testNoBurstWhenCaptureIsEmpty();
  testEmptyFrameIsRejected();
  testMacFcsKnownVectors();
  testNoiseOnlyCaptureFailsSync();
  if (argc > 1) {
    testCaptureFrame(argv[1], "UWB_4a_ch9_CI3_PRF15M6_6M81_PSDU100", true, 3, false, 0);
    testCaptureFrame(argv[1], "UWB_4z_BPRF_ch5_CI10_SP1_PSDU127", false, 10, true, 1);
    testCaptureFrame(argv[1], "UWB_4z_BPRF_ch9_CI9_SP0_PSDU127", false, 9, false, 0);
    testCaptureFrame(argv[1], "UWB_4z_BPRF_ch9_CI11_SP3", false, 11, false, 3);
    testCaptureFrame(argv[1], "UWB_4z_HPRF_ch9_CI25_249M6_PSDU127", false, 25, true, 1, HPRF);
    testCaptureFrame(argv[1], "UWB_4z_HPRF_ch5_CI27_124M8_PSDU100", false, 27, false, 1, HPRF);
    testCaptureFrame(argv[1], "UWB_4ab_EHPRF_ch9_62M4_BCC_PSDU256", false, 25, false, 1, EHPRF);
    testCaptureFrame(argv[1], "UWB_4ab_EHPRF_ch5_7M8_LDPC_PSDU127", false, 25, false, 1, EHPRF);
    testCaptureFrame(argv[1], "UWB_4ab_EHPRF_ch9_124M8_LDPC_PSDU1024", false, 25, false, 1, EHPRF);
  }

  if (gFailures == 0) {
    std::printf("All UWB HRP core tests passed\n");
    return 0;
  }
  std::printf("%d UWB HRP core test(s) FAILED\n", gFailures);
  return 1;
}
