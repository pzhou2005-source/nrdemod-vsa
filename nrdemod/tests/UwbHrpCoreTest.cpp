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
#include <limits>
#include <string>
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

std::vector<std::complex<double> > packet(bool legacy, int sp, int rsErrors, int phrErrors,
                                        double cfo, bool damageSfd = false)
{
  const std::string code = legacy ? "-+0++000-+-++00++0+00-0000-0+0-" :
      "+00+000-0--00--+0+0+00-+-++0+0000++-000+00-00--0-+0+0--0-+++0++000+-0+00-0++-0+++00-+00+0+0-0++-+--+000000+00000-+0000-0-000--+";
  // Independent Python/RSCodec golden stages for PSDU octets 01 23 45 67.
  std::string phr = "1000001000001011010";
  const std::string rs = "10000000110001001010001011100110011001110000010000000011100010110000000001111010";
  for (int i = 0; i < phrErrors; ++i) phr[5+6*i] = phr[5+6*i] == '0' ? '1' : '0';
  std::vector<int> input;
  for (size_t i = 0; i < phr.size(); ++i) input.push_back(phr[i]-'0');
  for (size_t i = 0; i < rs.size(); ++i) input.push_back(rs[i]-'0');
  for (int i = 0; i < rsErrors; ++i) input[19+2+6*i] ^= 1;
  input.push_back(0); input.push_back(0);
  std::vector<int> encoded;
  int previous = 0, older = 0;
  for (size_t i = 0; i < input.size(); ++i) {
    encoded.push_back(previous);
    encoded.push_back(input[i]^older);
    older = previous; previous = input[i];
  }
  std::vector<int> pn;
  for (size_t i = 0; i < code.size() && pn.size() < 15; ++i)
    if (code[i] != '0') pn.push_back(code[i] == '+' ? 1 : 0);
  for (int i = 15; i < 10000; ++i) pn.push_back(pn[i-14]^pn[i-15]);
  const int spreading = legacy ? 16 : 4, bursts = legacy ? 32 : 8;
  const int cpbPhr = legacy ? 16 : 64, cpbData = legacy ? 2 : 8;
  const int period = static_cast<int>(code.size())*spreading;
  const int sfd[] = {0,1,0,-1,1,0,0,-1};
  const int skipSts = sp == 1 ? 33792 : 0;
  const int dataChips = sp == 3 ? 0 : 21*bursts*cpbPhr + (static_cast<int>(input.size())-21)*bursts*cpbData;
  std::vector<double> chips(72*period + skipSts + dataChips, 0.0);
  for (int symbol = 0; symbol < 72; ++symbol) {
    const int polarity = symbol < 64 ? 1 : (damageSfd ? 1 : sfd[symbol-64]);
    for (size_t i = 0; i < code.size(); ++i)
      chips[symbol*period+i*spreading] = polarity * (code[i] == '+' ? 1 : (code[i] == '-' ? -1 : 0));
  }
  int offset = 72*period+skipSts, pnOffset = 15;
  for (size_t k = 0; sp != 3 && k < input.size(); ++k) {
    const int cpb = k < 21 ? cpbPhr : cpbData;
    int hop = pn[pnOffset];
    if (legacy) hop += 2*pn[pnOffset+1]+4*pn[pnOffset+2];
    const int begin = offset + (hop+encoded[2*k]*bursts/2)*cpb;
    for (int j = 0; j < cpb; ++j)
      chips[begin+j] = (1-2*encoded[2*k+1])*(1-2*pn[pnOffset+j]);
    offset += bursts*cpb;
    pnOffset += cpb;
  }
  const double b[] = {0.010209480791203138,0.040837923164812551,0.061256884747218826,
                      0.040837923164812551,0.010209480791203138};
  const double a[] = {1,-1.9684277869385185,1.7358607092088867,-0.72447082950736263,0.12038959989624451};
  std::vector<double> x(chips.size()*4+256,0.0), y(x.size(),0.0);
  for (size_t i = 0; i < chips.size(); ++i) x[73+4*i] = chips[i];
  std::vector<std::complex<double> > iq(x.size());
  unsigned noise = 81;
  for (size_t i = 0; i < x.size(); ++i) {
    y[i] = b[0]*x[i];
    for (size_t j = 1; j <= 4 && j <= i; ++j) y[i] += b[j]*x[i-j]-a[j]*y[i-j];
    noise = 1664525u*noise+1013904223u;
    const double nr = (static_cast<double>(noise>>8)/16777216.0-0.5)*0.0001;
    noise = 1664525u*noise+1013904223u;
    const double ni = (static_cast<double>(noise>>8)/16777216.0-0.5)*0.0001;
    iq[i] = y[i]*std::polar(0.7, 0.81+TWO_PI*cfo*i/1996.8e6) + std::complex<double>(nr,ni);
  }
  return iq;
}

void testPacketDecoding()
{
  Config cfg;
  cfg.sampleRate = 1996.8e6;
  DataConfig data;
  const int expected[] = {1,35,69,103};
  for (int legacy = 0; legacy < 2; ++legacy) {
    cfg.phyMode = legacy ? IEEE802154A : BPRF;
    data.codeIndex = legacy ? 3 : 9;
    data.meanPrfMHz = legacy ? 15.6 : 62.4;
    for (int errors = 0; errors <= 4; ++errors) {
      const Result r = demodulateData(packet(legacy != 0,0,errors,0,200e3),cfg,data);
      if (!r.dataError.empty()) std::printf("packet error: %s\n",r.dataError.c_str());
      CHECK(r.ok && r.sfdFound && r.phrDecoded && r.psduDecoded && r.rsOk);
      CHECK(r.psduBytes == std::vector<int>(expected,expected+4));
      CHECK(r.rsCorrectedSymbols == errors);
      CHECK(r.phrStartSamples == 73+72*(legacy ? 496 : 508)*4);
      CHECK_NEAR(r.frequencyErrorHz,200e3,10);
    }
    Result r = demodulateData(packet(legacy != 0,0,5,0,0),cfg,data);
    CHECK(!r.psduDecoded && !r.rsOk && !r.dataError.empty());
    r = demodulateData(packet(legacy != 0,0,0,1,0),cfg,data);
    CHECK(r.psduDecoded && r.psduBytes == std::vector<int>(expected,expected+4));
    r = demodulateData(packet(legacy != 0,0,0,2,0),cfg,data);
    CHECK(!r.phrDecoded && !r.dataError.empty());
    r = demodulateData(packet(legacy != 0,0,0,0,0,true),cfg,data);
    CHECK(!r.sfdFound && !r.psduDecoded && !r.dataError.empty());
    std::vector<std::complex<double> > truncated = packet(legacy != 0,0,0,0,0);
    truncated.resize(truncated.size()-4096);
    r = demodulateData(truncated,cfg,data);
    CHECK(!r.psduDecoded && !r.dataError.empty());
    data.codeIndex = legacy ? 4 : 10;
    r = demodulateData(packet(legacy != 0,0,0,0,0),cfg,data);
    CHECK(!r.psduDecoded && !r.dataError.empty());
  }
  cfg.phyMode = BPRF;
  data = DataConfig();
  for (int sp = 1; sp <= 3; ++sp) {
    data.stsPacketConfiguration = sp;
    const Result r = demodulateData(packet(false,sp,0,0,0),cfg,data);
    CHECK(r.sfdFound && r.dataError.empty());
    CHECK(r.psduDecoded == (sp != 3));
  }
}

void testInvalidInputs()
{
  Config cfg;
  cfg.sampleRate = 1996.8e6;
  std::vector<std::complex<double> > iq(20000, std::complex<double>(0,0));
  CHECK(!demodulate(iq,cfg).ok);
  iq[0] = std::complex<double>(std::numeric_limits<double>::quiet_NaN(),0);
  CHECK(!demodulate(iq,cfg).ok);
  iq[0] = std::complex<double>(1,0);
  cfg.chipRateHz = 0;
  CHECK(!demodulate(iq,cfg).ok);
  cfg.chipRateHz = 499.2e6;
  cfg.syncLength = -1;
  CHECK(!demodulate(iq,cfg).ok);
}

}

int main()
{
  testPeriodicPreambleRecovery();
  testCarrierFrequencyErrorRecovery();
  testNoBurstWhenCaptureIsEmpty();
  testNoiseOnlyCaptureFailsSync();
  testPacketDecoding();
  testInvalidInputs();

  if (gFailures == 0) {
    std::printf("All UWB HRP core tests passed\n");
    return 0;
  }
  std::printf("%d UWB HRP core test(s) FAILED\n", gFailures);
  return 1;
}
