/**
 ****************************************************************************
 *
 * Standalone checks for the native NR CP-OFDM core (no test framework).
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "nr/NrOfdmCore.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace nrdemod;

namespace {

int gFailures = 0;

#define CHECK(cond) \
  do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++gFailures; } } while (0)

#define CHECK_NEAR(a, b, tol) \
  do { const double _a = (a), _b = (b); if (std::fabs(_a - _b) > (tol)) { \
    std::printf("FAIL %s:%d: %s = %g, expected %g (tol %g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); ++gFailures; } } while (0)

void addNoiseAndOffsets(std::vector<std::complex<float> >& x, double noiseRms,
    double cfoFraction, int fftSize, double phase, unsigned seed)
{
  unsigned state = seed;
  auto uniform = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0 - 0.5;
  };
  for (size_t n = 0; n < x.size(); ++n) {
    const std::complex<double> rot = std::polar(1.0,
        2.0 * 3.14159265358979323846 * cfoFraction * static_cast<double>(n) / fftSize + phase);
    std::complex<double> v(x[n].real(), x[n].imag());
    v *= rot;
    v += std::complex<double>(uniform() * noiseRms * 3.4641, uniform() * noiseRms * 3.4641) / std::sqrt(2.0);
    x[n] = std::complex<float>(static_cast<float>(v.real()), static_cast<float>(v.imag()));
  }
}

void testCpLengths()
{
  CHECK(normalCpLength(2048) == 144);
  CHECK(normalCpLength(4096) == 288);
  CHECK(normalCpLength(1024) == 72);
  CHECK(longCpLength(2048, 0) == 160);
  CHECK(longCpLength(2048, 1) == 176);
  CHECK(longCpLength(4096, 1) == 352);
  CHECK(longCpLength(2048, 3) == 272);
  CHECK(extendedCpLength(2048) == 512);
  CHECK(symbolsPerSlot(false) == 14);
  CHECK(symbolsPerSlot(true) == 12);
}

void testFftRoundTrip()
{
  std::vector<std::complex<double> > a(256);
  for (size_t i = 0; i < a.size(); ++i) a[i] = std::complex<double>(std::sin(0.1 * i), std::cos(0.37 * i));
  std::vector<std::complex<double> > b = a;
  fft(b, false);
  fft(b, true);
  for (size_t i = 0; i < a.size(); ++i) {
    CHECK_NEAR(b[i].real() / 256.0, a[i].real(), 1e-9);
    CHECK_NEAR(b[i].imag() / 256.0, a[i].imag(), 1e-9);
  }
}

void testDftArbitraryLength()
{
  // Bluestein DFT against the direct definition for a 12 * 11 = 132 point block
  const size_t m = 132;
  std::vector<std::complex<double> > a(m);
  for (size_t i = 0; i < m; ++i) a[i] = std::complex<double>(std::cos(0.7 * i), std::sin(1.3 * i) * 0.5);
  std::vector<std::complex<double> > direct(m);
  for (size_t k = 0; k < m; ++k) {
    std::complex<double> acc(0.0, 0.0);
    for (size_t n = 0; n < m; ++n) {
      acc += a[n] * std::polar(1.0, -2.0 * 3.14159265358979323846 * static_cast<double>(k * n) / m);
    }
    direct[k] = acc;
  }
  std::vector<std::complex<double> > b = a;
  dft(b, false);
  for (size_t k = 0; k < m; ++k) {
    CHECK_NEAR(b[k].real(), direct[k].real(), 1e-8);
    CHECK_NEAR(b[k].imag(), direct[k].imag(), 1e-8);
  }
  dft(b, true);
  for (size_t i = 0; i < m; ++i) {
    CHECK_NEAR(b[i].real() / m, a[i].real(), 1e-8);
    CHECK_NEAR(b[i].imag() / m, a[i].imag(), 1e-8);
  }
}

void testResamplerTone()
{
  // a 1 MHz tone at 10 MSa/s resampled to 12.5 MSa/s must stay a 1 MHz tone of the same amplitude
  const double fsIn = 10e6, fsOut = 12.5e6, f = 1e6;
  std::vector<std::complex<double> > in(4000);
  for (size_t n = 0; n < in.size(); ++n) in[n] = std::polar(0.8, 2.0 * 3.14159265358979323846 * f * n / fsIn);
  const std::vector<std::complex<double> > out = resample(in, fsIn, fsOut);
  CHECK(out.size() > 4900 && out.size() <= 5000);
  for (size_t n = 200; n + 200 < out.size(); ++n) {
    const std::complex<double> expected = std::polar(0.8, 2.0 * 3.14159265358979323846 * f * n / fsOut);
    CHECK_NEAR(std::abs(out[n] - expected), 0.0, 2e-3);
  }
}

void testCarrierTable()
{
  CHECK(carrierResourceBlocks(FR1, 100, 2) == 135);
  CHECK(carrierResourceBlocks(FR1, 100, 1) == 273);
  CHECK(carrierResourceBlocks(FR1, 20, 0) == 106);
  CHECK(carrierResourceBlocks(FR2, 100, 3) == 66);
  CHECK(carrierResourceBlocks(FR2, 200, 3) == 132);
  CHECK(carrierResourceBlocks(FR1, 100, 0) == 0);
  CHECK(carrierResourceBlocks(FR1, 37, 1) == 0);

  DmrsConfig d;
  std::vector<int> pos = dmrsSymbolsInSlot(d);
  CHECK(pos.size() == 1 && pos[0] == 2);
  d.additionalPositions = 3;
  pos = dmrsSymbolsInSlot(d);
  CHECK(pos.size() == 4 && pos[3] == 11);
  d.mappingTypeB = true;
  d.additionalPositions = 1;
  pos = dmrsSymbolsInSlot(d);
  CHECK(pos.size() == 2 && pos[0] == 0 && pos[1] == 10);
}

void testHardDecision()
{
  const std::complex<double> q = hardDecision(std::complex<double>(0.6, -0.9), MOD_QPSK);
  CHECK_NEAR(q.real(), 1.0 / std::sqrt(2.0), 1e-12);
  CHECK_NEAR(q.imag(), -1.0 / std::sqrt(2.0), 1e-12);
  const std::complex<double> h = hardDecision(std::complex<double>(0.9, 0.05), MOD_QAM16);
  CHECK_NEAR(h.real(), 3.0 / std::sqrt(10.0), 1e-12);
  CHECK_NEAR(h.imag(), 1.0 / std::sqrt(10.0), 1e-12);
  const std::complex<double> big = hardDecision(std::complex<double>(9.0, -9.0), MOD_QAM64);
  CHECK_NEAR(big.real(), 7.0 / std::sqrt(42.0), 1e-12);
}

struct Impairment {
  double clockPpm;        // sample clock offset: symbol timing drifts across the burst
  double amplitudeMod;    // relative gain modulation, slow sine over ~14 symbols
  double phaseModRad;     // phase modulation, slow sine over ~14 symbols
  bool   conjugate;       // spectrally mirrored capture
  int    idleBefore;      // zero samples inserted in front of the burst (TDD / burst mode)
  int    idleAfter;       // zero samples appended after the burst
  double iqGainDb;        // Q branch gain error
  double iqPhaseDeg;      // Q branch phase error
  double iqSkewSamples;   // Q branch delay
  std::complex<double> dcOffset;
  Impairment() : clockPpm(0.0), amplitudeMod(0.0), phaseModRad(0.0), conjugate(false), idleBefore(0), idleAfter(0),
                 iqGainDb(0.0), iqPhaseDeg(0.0), iqSkewSamples(0.0), dcOffset(0.0, 0.0) {}
};

// transmitter model r = s_I + j g exp(j phi) s_Q(t - tau); the receiver DC offset is added
// after the frequency offset by runScenario
void applyIqImpairment(std::vector<std::complex<float> >& x, const Impairment& imp)
{
  if (imp.iqGainDb == 0.0 && imp.iqPhaseDeg == 0.0 && imp.iqSkewSamples == 0.0) return;
  const size_t n = x.size();
  std::vector<double> q(n);
  for (size_t m = 0; m < n; ++m) q[m] = x[m].imag();
  if (imp.iqSkewSamples != 0.0) {
    // windowed sinc fractional delay of the Q branch
    std::vector<double> d(n, 0.0);
    const int half = 24;
    for (size_t m = 0; m < n; ++m) {
      double acc = 0.0;
      for (int k = -half; k <= half; ++k) {
        const long src = static_cast<long>(m) - k;
        if (src < 0 || src >= static_cast<long>(n)) continue;
        const double t = static_cast<double>(k) - imp.iqSkewSamples;
        const double sinc = std::fabs(t) < 1e-12 ? 1.0 : std::sin(3.14159265358979323846 * t) / (3.14159265358979323846 * t);
        const double w = 0.5 + 0.5 * std::cos(3.14159265358979323846 * t / (half + 1));
        acc += q[src] * sinc * w;
      }
      d[m] = acc;
    }
    q.swap(d);
  }
  const std::complex<double> G = std::polar(std::pow(10.0, imp.iqGainDb / 20.0), imp.iqPhaseDeg * 3.14159265358979323846 / 180.0);
  for (size_t m = 0; m < n; ++m) {
    const std::complex<double> v = std::complex<double>(x[m].real(), 0.0) + std::complex<double>(0.0, 1.0) * G * q[m];
    x[m] = std::complex<float>(static_cast<float>(v.real()), static_cast<float>(v.imag()));
  }
}

void applyImpairment(std::vector<std::complex<float> >& x, const Config& cfg, const Impairment& imp)
{
  const double symbolSamples = cfg.sampleRate / (15000.0 * (1 << cfg.numerology)) * 1.07;
  if (imp.clockPpm != 0.0) {
    std::vector<std::complex<double> > d(x.size());
    for (size_t n = 0; n < x.size(); ++n) d[n] = std::complex<double>(x[n].real(), x[n].imag());
    d = resample(d, cfg.sampleRate, cfg.sampleRate * (1.0 + imp.clockPpm * 1e-6));
    x.resize(d.size());
    for (size_t n = 0; n < d.size(); ++n) x[n] = std::complex<float>(static_cast<float>(d[n].real()), static_cast<float>(d[n].imag()));
  }
  for (size_t n = 0; n < x.size(); ++n) {
    std::complex<double> v(x[n].real(), x[n].imag());
    const double t = static_cast<double>(n) / symbolSamples;
    // slow relative to one symbol: the intra symbol variation (ICI) stays negligible and
    // per symbol tracking is the only thing that can remove the error
    v *= 1.0 + imp.amplitudeMod * std::sin(2.0 * 3.14159265358979323846 * t / 14.0);
    v *= std::polar(1.0, imp.phaseModRad * std::sin(2.0 * 3.14159265358979323846 * t / 14.0));
    x[n] = std::complex<float>(static_cast<float>(v.real()), static_cast<float>(v.imag()));
  }
  if (imp.idleBefore > 0 || imp.idleAfter > 0) {
    std::vector<std::complex<float> > padded(static_cast<size_t>(imp.idleBefore), std::complex<float>(0.0f, 0.0f));
    padded.insert(padded.end(), x.begin(), x.end());
    padded.resize(padded.size() + static_cast<size_t>(imp.idleAfter), std::complex<float>(0.0f, 0.0f));
    x.swap(padded);
  }
  applyIqImpairment(x, imp);
}

Result runScenario(const char* title, Config cfg, int numSymbols, int leading,
    double noiseRms, double cfoFraction, bool autoDetect, double evmLimit, int expectedTiming,
    const Impairment& imp = Impairment());

void runLoopback(int mu, int fftSize, Modulation mod, int numSymbols, int leading,
    double noiseRms, double cfoFraction, bool autoDetect, double evmLimit)
{
  Config cfg;
  cfg.numerology = mu;
  cfg.fftSize = fftSize;
  cfg.sampleRate = fftSize * 15000.0 * (1 << mu);
  cfg.modulation = mod;
  cfg.numResourceBlocks = 0;
  runScenario("loopback", cfg, numSymbols, leading, noiseRms, cfoFraction, autoDetect, evmLimit, leading);
}

Result runScenario(const char* title, Config cfg, int numSymbols, int leading,
    double noiseRms, double cfoFraction, bool autoDetect, double evmLimit, int expectedTiming,
    const Impairment& imp)
{
  const Modulation mod = static_cast<Modulation>(cfg.modulation);

  std::vector<std::complex<double> > ref;
  std::vector<std::complex<float> > x = generateTestSignal(cfg, numSymbols, leading, 12345u + cfg.numerology, &ref);
  CHECK(!x.empty());
  applyImpairment(x, cfg, imp);
  // CFO is specified in subcarrier spacings; convert to the input sample rate
  const double scs = 15000.0 * (1 << cfg.numerology);
  addNoiseAndOffsets(x, noiseRms, cfoFraction * scs / cfg.sampleRate, 1, 0.3, 777u);
  if (imp.conjugate) {
    // the adapter handles MirrorFrequencySpectrum by conjugating; emulate it here
    for (size_t n = 0; n < x.size(); ++n) x[n] = std::conj(x[n]);
  }
  if (imp.dcOffset != std::complex<double>(0.0, 0.0)) {
    const std::complex<float> dc(static_cast<float>(imp.dcOffset.real()), static_cast<float>(imp.dcOffset.imag()));
    for (size_t n = 0; n < x.size(); ++n) x[n] += dc;
  }

  if (autoDetect) cfg.modulation = MOD_AUTO;
  const Result r = demodulate(x.data(), x.size(), cfg);
  if (!r.ok) std::printf("  error: %s\n", r.error.c_str());
  CHECK(r.ok);
  if (!r.ok) return r;
  CHECK(r.numSymbols == numSymbols);
  const bool resampled = std::fabs(r.resampleRate - cfg.sampleRate) > 1.0 || imp.clockPpm != 0.0;
  if (expectedTiming >= 0) CHECK(std::abs(r.timingOffset - expectedTiming) <= (resampled ? 4 : 0));
  CHECK(r.detectedModulation == mod);
  const double expectedCfo = imp.conjugate ? -cfoFraction * scs : cfoFraction * scs;
  CHECK_NEAR(r.frequencyErrorHz, expectedCfo, 0.02 * scs);
  CHECK(r.rmsEvmPercent < evmLimit);
  CHECK(r.rmsEvmPercent >= 0.0);
  const size_t numSc = static_cast<size_t>(12 * r.numResourceBlocks);
  CHECK(r.constellation.size() == static_cast<size_t>(numSymbols) * numSc);
  CHECK(ref.size() == r.constellation.size() - r.dmrsSymbols.size() * numSc);

  // extended statistics: consistent sizes and values for every scenario
  const size_t numSym = static_cast<size_t>(numSymbols);
  CHECK(r.errorVector.size() == r.constellation.size());
  CHECK(r.peakEvmPerSymbol.size() == numSym && r.peakEvmPerSubcarrier.size() == numSc);
  CHECK(r.commonPhaseErrorPerSymbol.size() == numSym && r.channelTypePerSymbol.size() == numSym);
  CHECK(r.channelImpulseResponseDb.size() == numSc && r.channelPhaseDifference.size() == numSc);
  CHECK(r.powerPerResourceBlockDb.size() == numSc / 12 && r.evmPerResourceBlock.size() == numSc / 12);
  CHECK(r.numSlots >= 1 && r.evmPerSlot.size() == static_cast<size_t>(r.numSlots) && r.peakEvmPerSlot.size() == r.evmPerSlot.size());
  CHECK(r.peakEvmSymbol >= 0 && r.peakEvmSymbol < numSymbols && r.peakEvmSubcarrier >= 0 && static_cast<size_t>(r.peakEvmSubcarrier) < numSc);
  CHECK(r.peakEvmPercent >= r.rmsEvmPercent);
  for (size_t j = 0; j < numSym; ++j) {
    CHECK(r.peakEvmPerSymbol[j] >= r.evmPerSymbol[j] - 1e-9);
    CHECK(r.errorVector[j * numSc] == r.constellation[j * numSc] - r.reference[j * numSc]);
    CHECK(r.channelTypePerSymbol[j] == (std::find(r.dmrsSymbols.begin(), r.dmrsSymbols.end(), static_cast<int>(j)) != r.dmrsSymbols.end() ? 2 : 1));
  }
  for (int s = 0; s < r.numSlots; ++s) CHECK(r.peakEvmPerSlot[s] >= r.evmPerSlot[s] - 1e-9);
  CHECK(r.syncCorrelation > 0.5 && r.syncCorrelation <= 1.0 + 1e-9);
  CHECK(r.normalizationFactor > 0.0);
  CHECK(r.channelImpulseResponseDb[0] > -6.0); // flat channel: energy in tap 0 (split with a neighbour at most)
  CHECK(r.flatnessRippleRange1Db >= 0.0 && r.flatnessRippleRange2Db >= 0.0);
  if (!r.dmrsSymbols.empty()) {
    // DM-RS symbols carry no decidable data, a sample clock error is not tracked for them
    if (imp.clockPpm == 0.0) CHECK(r.dmrsEvmPercent >= 0.0 && r.dmrsEvmPercent < 3.0 * r.rmsEvmPercent + 0.5);
    CHECK_NEAR(r.dmrsPowerDb, 0.0, 0.5); // generator: unit power DM-RS
  } else {
    CHECK(r.dmrsEvmPercent == 0.0);
  }
  if (imp.iqGainDb == 0.0 && imp.iqPhaseDeg == 0.0 && imp.iqSkewSamples == 0.0 && noiseRms <= 0.005) {
    CHECK(std::fabs(r.iqGainImbalanceDb) < 0.1 && std::fabs(r.iqQuadratureErrorDeg) < 0.6);
  }
  if (imp.dcOffset == std::complex<double>(0.0, 0.0) && !cfg.removeIqOffset) CHECK(r.iqOffsetDb < -25.0);
  std::printf("  %-12s mu=%d N=%d Fs=%.4g mod=%d syms=%d dmrs=%zu noise=%.3f cfo=%.3f -> EVM %.3f %% (peak %.3f %%), fErr %.1f Hz, t0 %d, RB %d/%d k0 %d\n"
              "               rho %.3f iq %.3f dB %.3f deg %.2g s dc %.1f dB dmrs %.3f %% %.2f dB cpe %.2f deg slots %d/%d\n",
      title, cfg.numerology, r.fftSize, cfg.sampleRate, mod, numSymbols, r.dmrsSymbols.size(), noiseRms, cfoFraction,
      r.rmsEvmPercent, r.peakEvmPercent, r.frequencyErrorHz, r.timingOffset, r.numResourceBlocks,
      r.carrierResourceBlocks, r.firstSubcarrierIndex,
      r.syncCorrelation, r.iqGainImbalanceDb, r.iqQuadratureErrorDeg, r.iqTimingSkewSec, r.iqOffsetDb,
      r.dmrsEvmPercent, r.dmrsPowerDb, r.commonPhaseErrorDeg, r.firstSlotIndex, r.numSlots);
  return r;
}

}

int main()
{
  testCpLengths();
  testFftRoundTrip();
  testDftArbitraryLength();
  testResamplerTone();
  testCarrierTable();
  testHardDecision();

  std::printf("ideal loopback\n");
  runLoopback(0, 1024, MOD_QPSK,   14, 0,   0.0,  0.0,  false, 0.1);
  runLoopback(1, 2048, MOD_QAM16,  28, 100, 0.0,  0.0,  false, 0.1);
  runLoopback(2, 1024, MOD_QAM64,  20, 33,  0.0,  0.0,  false, 0.1);
  runLoopback(3, 512,  MOD_QAM256, 60, 7,   0.0,  0.0,  false, 0.1);

  std::printf("noise + cfo, explicit modulation\n");
  runLoopback(1, 2048, MOD_QPSK,   28, 50,  0.05, 0.12, false, 12.0);
  runLoopback(1, 2048, MOD_QAM64,  28, 50,  0.01, -0.2, false, 5.0);

  std::printf("auto modulation detection\n");
  runLoopback(1, 2048, MOD_QPSK,   28, 10,  0.02, 0.05, true, 8.0);
  runLoopback(1, 2048, MOD_QAM16,  28, 10,  0.01, 0.05, true, 5.0);
  runLoopback(1, 2048, MOD_QAM64,  28, 10,  0.005, 0.05, true, 3.0);
  runLoopback(0, 2048, MOD_QAM256, 28, 10,  0.002, 0.02, true, 2.0);

  std::printf("carrier grid from channel bandwidth (legacy para2 style)\n");
  {
    // FR1 100 MHz, mu 2, 135 RB, 122.88 MSa/s: same as 5GNR_FR1_FDD_UL_100MHz_mu2_MCS27_256QAM_CP.para2
    Config cfg;
    cfg.numerology = 2;
    cfg.frequencyRange = FR1;
    cfg.channelBandwidthMHz = 100;
    cfg.sampleRate = 122.88e6;
    cfg.modulation = MOD_QAM256;
    cfg.dcPunctured = true;
    cfg.dmrs.additionalPositions = 0;
    cfg.symbolTimingAdjustmentPercent = -3.125;
    runScenario("fr1-100MHz", cfg, 28, 64, 0.002, 0.03, false, 2.0, 64);

    // noise free replica of the legacy para2 configuration (mapping type A, one slot)
    cfg.maxSymbols = 14;
    runScenario("para2-typeA", cfg, 14, 30, 0.0, 0.0, false, 0.05, 30);
    cfg.maxSymbols = 0;

    // partial allocation with offset inside the carrier
    cfg.numResourceBlocks = 40;
    cfg.resourceBlockOffset = 60;
    cfg.modulation = MOD_QAM64;
    runScenario("rb-offset", cfg, 14, 20, 0.005, -0.1, true, 3.0, 20);
  }
  {
    // FR2 100 MHz, mu 3, 66 RB, captured at 220.000224 MSa/s (not on the FFT grid -> resampling)
    Config cfg;
    cfg.numerology = 3;
    cfg.frequencyRange = FR2;
    cfg.channelBandwidthMHz = 100;
    cfg.sampleRate = 220.000224e6;
    cfg.modulation = MOD_QAM16;
    cfg.dmrs.additionalPositions = 1;
    runScenario("fr2-resample", cfg, 28, 37, 0.005, 0.05, false, 3.0, 37);
  }
  {
    // 100 MHz FR1 mu 2 captured with 150 MSa/s (Marlin EVM task rate): downsampling to 122.88 MSa/s
    Config cfg;
    cfg.numerology = 2;
    cfg.frequencyRange = FR1;
    cfg.channelBandwidthMHz = 100;
    cfg.sampleRate = 150e6;
    cfg.modulation = MOD_QAM64;
    runScenario("downsample", cfg, 14, 200, 0.003, 0.02, true, 3.0, 200);
  }

  std::printf("carrier offset and DFT-s-OFDM\n");
  {
    Config cfg;
    cfg.numerology = 1;
    cfg.fftSize = 1024;
    cfg.sampleRate = 1024 * 30e3;
    cfg.numResourceBlocks = 24;
    cfg.carrierOffsetHz = 3.3e6;
    cfg.modulation = MOD_QAM16;
    runScenario("carrier-off", cfg, 14, 12, 0.005, 0.0, true, 3.0, 12);

    cfg.carrierOffsetHz = 0.0;
    cfg.transformPrecoding = true;
    cfg.numResourceBlocks = 25; // 300 point DFT (not a power of two)
    cfg.modulation = MOD_QPSK;
    runScenario("dft-s-ofdm", cfg, 14, 12, 0.01, 0.04, false, 6.0, 12);
    cfg.modulation = MOD_QAM64;
    runScenario("dft-s-64qam", cfg, 28, 12, 0.003, 0.0, true, 3.0, 12);
  }

  std::printf("per symbol tracking (TrackTiming / TrackAmplitude / TrackPhase)\n");
  {
    Config cfg;
    cfg.numerology = 1;
    cfg.fftSize = 2048;
    cfg.sampleRate = 2048 * 30e3;
    // decision directed tracking: 16QAM decisions stay reliable under the raw impairment,
    // 100 RB keep the allocation inside the flat region of the resampler used for the clock error
    cfg.modulation = MOD_QAM16;
    cfg.numResourceBlocks = 100;
    Impairment clock;
    clock.clockPpm = 3.0; // ~0.18 sample drift over 28 symbols
    const Result withTiming = runScenario("clock-track", cfg, 28, 40, 0.002, 0.0, false, 1.0, 40, clock);
    cfg.timingTracking = false;
    const Result noTiming = runScenario("clock-notrk", cfg, 28, 40, 0.002, 0.0, false, 100.0, 40, clock);
    CHECK(withTiming.rmsEvmPercent < noTiming.rmsEvmPercent * 0.5);
    cfg.timingTracking = true;

    Impairment amp;
    amp.amplitudeMod = 0.04;
    const Result withAmp = runScenario("amp-track", cfg, 28, 40, 0.002, 0.0, false, 1.0, 40, amp);
    cfg.amplitudeTracking = false;
    const Result noAmp = runScenario("amp-notrk", cfg, 28, 40, 0.002, 0.0, false, 100.0, 40, amp);
    CHECK(withAmp.rmsEvmPercent < noAmp.rmsEvmPercent * 0.5);
    cfg.amplitudeTracking = true;

    Impairment phase;
    phase.phaseModRad = 0.03;
    const Result withPhase = runScenario("phase-track", cfg, 28, 40, 0.002, 0.0, false, 1.0, 40, phase);
    cfg.phaseTracking = false;
    const Result noPhase = runScenario("phase-notrk", cfg, 28, 40, 0.002, 0.0, false, 100.0, 40, phase);
    CHECK(withPhase.rmsEvmPercent < noPhase.rmsEvmPercent * 0.5);
  }

  std::printf("burst search (5GNRBurstMode): idle time before and after the transmission\n");
  {
    Config cfg;
    cfg.numerology = 2;
    cfg.fftSize = 2048;
    cfg.sampleRate = 2048 * 60e3;
    cfg.modulation = MOD_QAM256;
    cfg.numResourceBlocks = 32;
    cfg.maxSymbols = 14;
    Impairment idle;
    idle.idleBefore = 2 * (14 * 2192 + 64); // two idle slots (one long CP each), burst in the third
    idle.idleAfter = 14 * 2192;
    // the noise is added to the whole capture, the idle part is noise only
    const Result burst = runScenario("burst", cfg, 14, 25, 0.001, 0.02, false, 0.5, idle.idleBefore + 25, idle);
    CHECK(std::abs(burst.burstStart - idle.idleBefore) <= 2 * 2048 + 32); // one FFT length envelope resolution
    CHECK(burst.burstLength > 12 * 2192 && burst.burstLength < 16 * 2192);
    CHECK(burst.firstSlotIndex == 2);
    CHECK(burst.numSlots == 1);
    // without burst search the timing search is confined to the first symbol period
    cfg.burstSearch = false;
    std::vector<std::complex<float> > x = generateTestSignal(cfg, 14, 25, 12347u, NULL);
    applyImpairment(x, cfg, idle);
    const Result blind = demodulate(x.data(), x.size(), cfg);
    CHECK(!blind.ok || blind.rmsEvmPercent > 10.0);
    cfg.burstSearch = true;
    // a continuous capture is left untouched
    Impairment none;
    const Result cont = runScenario("continuous", cfg, 14, 25, 0.001, 0.02, false, 0.5, 25, none);
    CHECK(cont.burstStart == 0);
    CHECK(cont.burstLength >= 14 * 2192);
    CHECK(cont.firstSlotIndex == 0);
  }

  std::printf("I/Q impairments, DC offset, common phase error statistics\n");
  {
    Config cfg;
    cfg.numerology = 1;
    cfg.fftSize = 1024;
    cfg.sampleRate = 1024 * 30e3;
    cfg.modulation = MOD_QAM16;
    cfg.numResourceBlocks = 60; // symmetric around DC so that every subcarrier has its image
    Impairment iq;
    iq.iqGainDb = 0.5;
    iq.iqPhaseDeg = 2.0;
    const Result gp = runScenario("iq-gain-phs", cfg, 28, 30, 0.002, 0.0, false, 10.0, 30, iq);
    std::printf("    estimated gain %.3f dB, quadrature %.3f deg, skew %.3g s\n", gp.iqGainImbalanceDb, gp.iqQuadratureErrorDeg, gp.iqTimingSkewSec);
    CHECK_NEAR(gp.iqGainImbalanceDb, 0.5, 0.1);
    CHECK_NEAR(gp.iqQuadratureErrorDeg, 2.0, 0.3);
    CHECK(std::fabs(gp.iqTimingSkewSec) < 0.05 / cfg.sampleRate);
    // a frequency error of the transmitter rotates signal and image alike: no influence
    const Result gpCfo = runScenario("iq-with-cfo", cfg, 28, 30, 0.002, 0.04, false, 10.0, 30, iq);
    CHECK_NEAR(gpCfo.iqGainImbalanceDb, 0.5, 0.15);
    CHECK_NEAR(gpCfo.iqQuadratureErrorDeg, 2.0, 0.5);

    // skew: the image grows with tan(pi f tau) towards the band edges, QPSK keeps the decisions safe
    Impairment skew;
    skew.iqSkewSamples = 0.2;
    cfg.modulation = MOD_QPSK;
    const Result sk = runScenario("iq-skew", cfg, 28, 30, 0.002, 0.0, false, 30.0, 30, skew);
    std::printf("    estimated skew %.3f samples (expected 0.2), gain %.3f dB, quadrature %.3f deg\n",
        sk.iqTimingSkewSec * cfg.sampleRate, sk.iqGainImbalanceDb, sk.iqQuadratureErrorDeg);
    CHECK_NEAR(sk.iqTimingSkewSec * cfg.sampleRate, 0.2, 0.04);
    CHECK(std::fabs(sk.iqGainImbalanceDb) < 0.1);
    cfg.modulation = MOD_QAM16;

    // DC offset with a punctured DC subcarrier (as in the libdemod parameter files): the offset
    // leaks into the neighbouring subcarriers through the CFO correction unless it is removed first
    Impairment dc;
    dc.dcOffset = std::complex<double>(0.04, -0.03); // |dc|^2 = 0.0025 vs burst power 720/1024 -> -24.5 dB
    cfg.dcPunctured = true;
    const Result withDc = runScenario("dc-offset", cfg, 14, 30, 0.002, 0.3, false, 20.0, 30, dc);
    std::printf("    IQ offset %.2f dB, EVM %.3f %%\n", withDc.iqOffsetDb, withDc.rmsEvmPercent);
    // the mean of one slot of random data is itself a random variable, so the estimate scatters
    // by about +-1 dB around the nominal -24.5 dB from seed to seed
    CHECK_NEAR(withDc.iqOffsetDb, -24.5, 1.2);
    cfg.removeIqOffset = true;
    const Result noDc = runScenario("dc-removed", cfg, 14, 30, 0.002, 0.3, false, 1.0, 30, dc);
    CHECK_NEAR(noDc.iqOffsetDb, -24.5, 1.2); // reported before removal
    CHECK(noDc.rmsEvmPercent < withDc.rmsEvmPercent * 0.5);
    cfg.removeIqOffset = false;
    cfg.dcPunctured = false;

    // 0.03 rad sine over the burst -> ~1.2 deg RMS common phase error, removed by the tracking
    Impairment phase;
    phase.phaseModRad = 0.03;
    const Result cpe = runScenario("cpe-stat", cfg, 28, 30, 0.002, 0.0, false, 1.0, 30, phase);
    std::printf("    common phase error %.3f deg rms\n", cpe.commonPhaseErrorDeg);
    CHECK(cpe.commonPhaseErrorDeg > 0.8 && cpe.commonPhaseErrorDeg < 1.6);
    for (size_t j = 0; j < cpe.commonPhaseErrorPerSymbol.size(); ++j) {
      CHECK_NEAR(std::abs(cpe.commonPhaseErrorPerSymbol[j]), 1.0, 1e-9);
    }
    // DFT-s-OFDM: no image based I/Q estimate (pairs share the spread data)
    cfg.transformPrecoding = true;
    const Result dfts = runScenario("iq-dfts", cfg, 28, 30, 0.002, 0.0, false, 10.0, 30, iq);
    CHECK(dfts.iqGainImbalanceDb == 0.0 && dfts.iqQuadratureErrorDeg == 0.0 && dfts.iqTimingSkewSec == 0.0);
  }

  std::printf("mirrored spectrum, extended CP, DM-RS variants, multi slot\n");
  {
    Config cfg;
    cfg.numerology = 1;
    cfg.fftSize = 1024;
    cfg.sampleRate = 1024 * 30e3;
    cfg.modulation = MOD_QAM16;
    Impairment mirror;
    mirror.conjugate = true;
    runScenario("mirror", cfg, 14, 25, 0.005, 0.03, true, 3.0, 25, mirror);

    // three slots: long CP at 0, 14, 28 (mu 1), DM-RS type A pos 3 + 1 additional position
    cfg.dmrs.typeAPosition = 3;
    cfg.dmrs.additionalPositions = 1;
    const Result slots = runScenario("3-slots", cfg, 42, 9, 0.005, -0.02, false, 3.0, 9);
    CHECK(slots.dmrsSymbols.size() == 6);
    if (slots.dmrsSymbols.size() == 6) {
      const int expected[] = { 3, 11, 17, 25, 31, 39 };
      for (size_t i = 0; i < 6; ++i) CHECK(slots.dmrsSymbols[i] == expected[i]);
    }
    CHECK(slots.slotStartSymbol == 0);
    CHECK(slots.numSlots == 3 && slots.firstSlotIndex == 0);
    for (int s = 0; s < slots.numSlots; ++s) CHECK(slots.evmPerSlot[s] > 0.0 && slots.evmPerSlot[s] < 3.0);

    // capture starting mid slot: the long CP of the second slot marks the slot boundary
    Config mid = cfg;
    mid.dmrs.additionalPositions = 0;
    mid.dmrs.typeAPosition = 2;
    std::vector<std::complex<double> > refIgnored;
    std::vector<std::complex<float> > full = generateTestSignal(mid, 28, 0, 99u, &refIgnored);
    const int cpN = normalCpLength(1024), cpL = longCpLength(1024, 1);
    const size_t skip = static_cast<size_t>(cpL + 1024 + 4 * (cpN + 1024)); // drop the first 5 symbols
    std::vector<std::complex<float> > tail(full.begin() + skip, full.end());
    const Result midResult = demodulate(tail.data(), tail.size(), mid);
    CHECK(midResult.ok);
    if (midResult.ok) {
      CHECK(midResult.numSymbols == 23);
      CHECK(midResult.slotStartSymbol == 9);              // burst symbol 14 (long CP) = tail symbol 9
      CHECK(midResult.dmrsSymbols.size() == 1);            // burst symbol 16 = tail symbol 11 (2 is cut off)
      if (midResult.dmrsSymbols.size() == 1) CHECK(midResult.dmrsSymbols[0] == 11);
      CHECK(midResult.numSlots == 2); // 9 symbols of the first slot + one full slot
      std::printf("  mid-slot start: symbols=%d slotStart=%d dmrs=%zu evm=%.3f %%\n", midResult.numSymbols,
          midResult.slotStartSymbol, midResult.dmrsSymbols.size(), midResult.rmsEvmPercent);
      CHECK(midResult.rmsEvmPercent < 0.1);
    }

    // mapping type B with double symbol DM-RS
    Config typeB = cfg;
    typeB.dmrs.mappingTypeB = true;
    typeB.dmrs.doubleSymbol = true;
    typeB.dmrs.additionalPositions = 1;
    const Result b = runScenario("typeB-dbl", typeB, 14, 5, 0.005, 0.0, false, 3.0, 5);
    CHECK(b.dmrsSymbols.size() == 4);
    if (b.dmrsSymbols.size() == 4) {
      CHECK(b.dmrsSymbols[0] == 0 && b.dmrsSymbols[1] == 1 && b.dmrsSymbols[2] == 10 && b.dmrsSymbols[3] == 11);
    }

    // extended CP (mu 2 only): 12 symbols per slot, no long CP
    Config ext;
    ext.numerology = 2;
    ext.fftSize = 1024;
    ext.sampleRate = 1024 * 60e3;
    ext.extendedCp = true;
    ext.modulation = MOD_QAM64;
    runScenario("extended-cp", ext, 24, 17, 0.003, 0.05, true, 3.0, 17);
    ext.numerology = 1;
    const std::vector<std::complex<float> > dummy(4096, std::complex<float>(1.0f, 0.0f));
    const Result invalid = demodulate(dummy.data(), dummy.size(), ext); // extended CP invalid for mu 1
    CHECK(!invalid.ok && invalid.error.find("Extended cyclic prefix") != std::string::npos);

    // sync options: no timing search (capture starts at a CP), 1 and 12 symbol accumulation, equalizer off
    Config sync = cfg;
    sync.dmrs.typeAPosition = 2;
    sync.dmrs.additionalPositions = 0;
    sync.timingSearch = false;
    runScenario("no-search", sync, 14, 0, 0.005, 0.02, false, 3.0, 0);
    sync.timingSearch = true;
    sync.syncSearchSymbols = 1;
    runScenario("sync-1sym", sync, 14, 33, 0.02, 0.1, false, 8.0, 33);
    sync.syncSearchSymbols = 12;
    runScenario("sync-12sym", sync, 14, 33, 0.02, 0.1, false, 8.0, 33);
    sync.syncSearchSymbols = 4;
    sync.equalize = false;
    runScenario("no-equalizer", sync, 14, 33, 0.005, 0.0, false, 3.0, 33);
    sync.equalize = true;
    sync.maxSymbols = 5;
    std::vector<std::complex<float> > burst = generateTestSignal(sync, 14, 33, 5u, NULL);
    const Result limited = demodulate(burst.data(), burst.size(), sync);
    CHECK(limited.ok && limited.numSymbols == 5 && limited.evmPerSymbol.size() == 5 && limited.rmsEvmPercent < 0.1);
  }

  if (gFailures) {
    std::printf("%d check(s) failed\n", gFailures);
    return EXIT_FAILURE;
  }
  std::printf("all checks passed\n");
  return EXIT_SUCCESS;
}
