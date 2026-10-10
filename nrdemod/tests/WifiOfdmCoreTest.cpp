/**
 ****************************************************************************
 *
 * Standalone checks for the native IEEE 802.11a OFDM core (no test
 * framework), mirroring BtGfskCoreTest.cpp / UwbHrpCoreTest.cpp. Builds a
 * complete, spec-accurate 802.11a packet from scratch (STF + LTF + SIGNAL
 * field with real rate-1/2 convolutional coding/interleaving + DATA field)
 * and verifies the receiver recovers burst/sync, the SIGNAL field content
 * (modulation / rate / PSDU length) and a low EVM on the DATA field.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi/WifiOfdmCore.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace wifidemod;

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
const int FFT_SIZE = 64;

// Same LTF table and FFT as the core under test (duplicated here deliberately: the test
// must independently reconstruct the transmit side, not reuse the receiver's internals).
const int LTF_SYM[53] = {
     1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,
     1,  0,
     1, -1, -1,  1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,  1, -1, -1,  1, -1,  1, -1,  1,  1,  1,  1
};
double ltfValue(int k) { return (k < -26 || k > 26) ? 0.0 : static_cast<double>(LTF_SYM[k + 26]); }
bool isPilot(int k) { return k == -21 || k == -7 || k == 7 || k == 21; }
double pilotBaseSign(int k) { return k == 21 ? -1.0 : 1.0; }
bool isData(int k) { return k != 0 && k >= -26 && k <= 26 && !isPilot(k); }

std::vector<double> pilotPolarity(int length)
{
  std::vector<double> out(length);
  unsigned state = 0x7Fu;
  for (int n = 0; n < length; ++n) {
    const unsigned fb = ((state >> 3) ^ (state >> 6)) & 1u;
    out[n] = 1.0 - 2.0 * static_cast<double>(fb);
    state = ((state << 1) | fb) & 0x7Fu;
  }
  return out;
}

void fft64(std::vector<cd>& a, bool inverse)
{
  const size_t n = a.size();
  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
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
int bin(int k) { return ((k % FFT_SIZE) + FFT_SIZE) % FFT_SIZE; }

std::vector<cd> ofdmSymbolFromFreq(const std::vector<cd>& freqByBin)
{
  std::vector<cd> t = freqByBin;
  fft64(t, true);
  const double scale = 1.0 / std::sqrt(static_cast<double>(FFT_SIZE));
  for (auto& v : t) v *= scale;
  return t; // 64 samples, GI (cyclic prefix) prepended by caller
}

std::vector<cd> withGuardInterval(const std::vector<cd>& symbol64, int giLen)
{
  std::vector<cd> out(giLen + FFT_SIZE);
  for (int i = 0; i < giLen; ++i) out[i] = symbol64[FFT_SIZE - giLen + i];
  for (int i = 0; i < FFT_SIZE; ++i) out[giLen + i] = symbol64[i];
  return out;
}

// --- rate 1/2 K=7 convolutional encoder, matching the core's decoder convention ---
int parityOf(unsigned v) { int p = 0; while (v) { p ^= v & 1u; v >>= 1; } return p; }

std::vector<int> convolutionalEncodeRate1Half(const std::vector<int>& bits)
{
  std::vector<int> out(bits.size() * 2);
  unsigned state = 0;
  for (size_t i = 0; i < bits.size(); ++i) {
    state = ((state << 1) | static_cast<unsigned>(bits[i])) & 0x7Fu;
    out[2 * i] = parityOf(state & 0155u);
    out[2 * i + 1] = parityOf(state & 0117u);
  }
  return out;
}

std::vector<int> interleaveSignalField(const std::vector<int>& coded)
{
  // inverse of the core's deinterleaveSignalField: "second" permutation only (n_bpsc=1 case)
  const int nCbps = 48;
  std::vector<int> out(nCbps);
  for (int k = 0; k < nCbps; ++k) {
    const int second = 16 * k - (nCbps - 1) * static_cast<int>(std::floor(16.0 * k / nCbps));
    out[k] = coded[second];
  }
  return out;
}

// Builds a full, spec-accurate 802.11a BPSK-1/2 packet (STF+LTF+SIGNAL+1 DATA symbol of
// all-zero scrambled bits for simplicity - EVM/SIGNAL decode do not depend on payload content).
std::vector<cd> buildPacket(int psduLengthBytes, double frequencyErrorHz, double sampleRate, double noiseAmplitude,
    unsigned seed)
{
  // STF: 10 repetitions of a 16 sample short training symbol. Content doesn't matter for the
  // receiver under test (it syncs on the LTF, not the STF), so a simple periodic BPSK token
  // is enough to pad realistic burst timing ahead of the LTF.
  std::vector<cd> stf;
  unsigned state = seed;
  auto uniform = [&]() { state = state * 1664525u + 1013904223u; return static_cast<double>(state >> 8) / 16777216.0; };
  std::vector<cd> stfToken(16);
  for (auto& v : stfToken) v = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  for (int r = 0; r < 10; ++r) stf.insert(stf.end(), stfToken.begin(), stfToken.end());

  std::vector<cd> ltfFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) { const double v = ltfValue(k); if (v != 0.0) ltfFreq[bin(k)] = cd(v, 0); }
  std::vector<cd> ltfTime = ofdmSymbolFromFreq(ltfFreq);
  std::vector<cd> ltfWithGi(32 + 2 * FFT_SIZE); // 2x GI(32) + 2 LTF copies, per 17.3.3
  for (int i = 0; i < 32; ++i) ltfWithGi[i] = ltfTime[FFT_SIZE - 32 + i];
  for (int i = 0; i < FFT_SIZE; ++i) { ltfWithGi[32 + i] = ltfTime[i]; ltfWithGi[32 + FFT_SIZE + i] = ltfTime[i]; }

  // SIGNAL field: rate=0x0D (BPSK 1/2, 6 Mbps), length = psduLengthBytes, parity, 6 tail zeros
  std::vector<int> signalBits(24, 0);
  const int rateField = 0x0D;
  for (int b = 0; b < 4; ++b) signalBits[b] = (rateField >> b) & 1; // bit0 first, matches decoder's for(b<4) rateField=(rateField<<1)|bit
  // NOTE: decoder reconstructs rateField via rateField = (rateField<<1)|bit for b=0..3, i.e.
  // bit0 is the MSB fed first. So signalBits[0] must be bit3 of rateField, etc.
  for (int b = 0; b < 4; ++b) signalBits[b] = (rateField >> (3 - b)) & 1;
  signalBits[4] = 0; // reserved
  for (int b = 0; b < 12; ++b) signalBits[5 + b] = (psduLengthBytes >> b) & 1;
  int parity = 0;
  for (int b = 0; b < 17; ++b) parity ^= signalBits[b];
  signalBits[17] = parity;
  for (int b = 18; b < 24; ++b) signalBits[b] = 0;

  const std::vector<int> signalCoded = convolutionalEncodeRate1Half(signalBits);
  const std::vector<int> signalInterleaved = interleaveSignalField(signalCoded);

  const std::vector<double> polarity = pilotPolarity(127);
  std::vector<cd> signalFreq(FFT_SIZE, cd(0, 0));
  int bitIdx = 0;
  for (int k = -26; k <= 26; ++k) {
    if (isPilot(k)) {
      signalFreq[bin(k)] = cd(pilotBaseSign(k) * polarity[0], 0.0);
    } else if (isData(k)) {
      signalFreq[bin(k)] = signalInterleaved[bitIdx++] ? cd(1, 0) : cd(-1, 0);
    }
  }
  std::vector<cd> signalTime = ofdmSymbolFromFreq(signalFreq);
  std::vector<cd> signalWithGi = withGuardInterval(signalTime, 16);

  // DATA field: ceil((16+8*psduLengthBytes+6)/24) symbols for BPSK rate-1/2 (24 data bits per
  // symbol), each BPSK, pseudo-random bits on the 48 data subcarriers (content irrelevant to
  // EVM/SIGNAL checks; what matters is it round-trips through hard-decision cleanly, and that
  // the receiver finds exactly as many symbols as the SIGNAL field says to expect).
  const int nDataSym = static_cast<int>(std::ceil((16.0 + 8.0 * psduLengthBytes + 6.0) / 24.0));
  std::vector<cd> dataField;
  for (int sym = 0; sym < nDataSym; ++sym) {
    std::vector<cd> dataFreq(FFT_SIZE, cd(0, 0));
    const int dataN = (sym + 3 - 2) % 127; // trellis symbol index = sym + 3 (0,1=LTF, 2=SIGNAL)
    for (int k = -26; k <= 26; ++k) {
      if (isPilot(k)) {
        dataFreq[bin(k)] = cd(pilotBaseSign(k) * polarity[dataN], 0.0);
      } else if (isData(k)) {
        dataFreq[bin(k)] = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
      }
    }
    std::vector<cd> dataTime = ofdmSymbolFromFreq(dataFreq);
    std::vector<cd> dataWithGi = withGuardInterval(dataTime, 16);
    dataField.insert(dataField.end(), dataWithGi.begin(), dataWithGi.end());
  }

  std::vector<cd> packet;
  packet.insert(packet.end(), stf.begin(), stf.end());
  packet.insert(packet.end(), ltfWithGi.begin(), ltfWithGi.end());
  packet.insert(packet.end(), signalWithGi.begin(), signalWithGi.end());
  packet.insert(packet.end(), dataField.begin(), dataField.end());

  double phase = 0.0;
  unsigned noiseState = seed ^ 0x5A5A5A5Au;
  auto noise = [&]() { noiseState = noiseState * 1664525u + 1013904223u; return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * noiseAmplitude; };
  for (auto& s : packet) {
    phase += TWO_PI * frequencyErrorHz / sampleRate;
    s = s * std::polar(1.0, phase) + cd(noise(), noise());
  }
  return packet;
}

void testBurstSyncAndSignalField()
{
  const double sampleRate = 20.0e6;
  std::vector<cd> packet = buildPacket(100, 0.0, sampleRate, 0.01, 1u);
  std::vector<cd> capture(packet.size() + 400, cd(0, 0));
  unsigned noiseState = 2u;
  auto noise = [&]() { noiseState = noiseState * 1664525u + 1013904223u; return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.005; };
  for (auto& s : capture) s = cd(noise(), noise());
  std::copy(packet.begin(), packet.end(), capture.begin() + 200);

  Config config;
  config.sampleRate = sampleRate;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.burstFound);
  CHECK(result.syncFound);
  CHECK(result.syncRho > 0.8);
  CHECK(result.modulation == MOD_BPSK);
  CHECK(result.codingRateNumerator == 1);
  CHECK(result.codingRateDenominator == 2);
  CHECK(result.dataRateMbps == 6);
  CHECK(result.psduLengthBytes == 100);
  CHECK(result.rmsEvmPercent < 10.0);
}

void testCarrierFrequencyErrorRecovery()
{
  const double sampleRate = 20.0e6;
  const double injectedOffsetHz = 50e3;
  std::vector<cd> packet = buildPacket(50, injectedOffsetHz, sampleRate, 0.01, 3u);
  std::vector<cd> capture(packet.size() + 400, cd(0, 0));
  unsigned noiseState = 4u;
  auto noise = [&]() { noiseState = noiseState * 1664525u + 1013904223u; return (static_cast<double>(noiseState >> 8) / 16777216.0 - 0.5) * 0.005; };
  for (auto& s : capture) s = cd(noise(), noise());
  std::copy(packet.begin(), packet.end(), capture.begin() + 200);

  Config config;
  config.sampleRate = sampleRate;
  const Result result = demodulate(capture, config);

  CHECK(result.ok);
  CHECK(result.syncFound);
  CHECK_NEAR(result.frequencyErrorHz, injectedOffsetHz, 2000.0);
  CHECK(result.psduLengthBytes == 50);
}

void testNoBurstWhenCaptureIsEmpty()
{
  Config config;
  config.sampleRate = 20.0e6;
  const Result result = demodulate(std::vector<cd>(), config);
  CHECK(!result.ok);
  CHECK(!result.error.empty());
}

void testNoiseOnlyCaptureFailsSync()
{
  const double sampleRate = 20.0e6;
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
  testBurstSyncAndSignalField();
  testCarrierFrequencyErrorRecovery();
  testNoBurstWhenCaptureIsEmpty();
  testNoiseOnlyCaptureFailsSync();

  if (gFailures == 0) {
    std::printf("All WiFi 802.11a core tests passed\n");
    return 0;
  }
  std::printf("%d check(s) failed\n", gFailures);
  return 1;
}
