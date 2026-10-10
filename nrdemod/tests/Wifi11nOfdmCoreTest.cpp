/**
 ****************************************************************************
 *
 * Standalone checks for the native IEEE 802.11n (HT, 20 MHz, Nss=1) OFDM
 * core (no test framework), mirroring WifiOfdmCoreTest.cpp. Builds a
 * complete, spec-accurate HT mixed-format packet from scratch (L-STF +
 * L-LTF + L-SIG(dummy, unused by the receiver) + HT-SIG1/HT-SIG2 with real
 * rate-1/2 convolutional coding/interleaving + HT-STF/HT-LTF(unused,
 * channel estimate is reused from L-LTF) + DATA field on the wider
 * 56-subcarrier HT grid) and verifies the receiver recovers burst/sync,
 * the HT-SIG field content (MCS / bandwidth / PSDU length) and a low EVM
 * on the DATA field.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11n/Wifi11nOfdmCore.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace wifi11ndemod;

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

// Same L-LTF table and FFT as the core under test (duplicated deliberately: the test must
// independently reconstruct the transmit side, not reuse the receiver's internals).
const int LTF_SYM[53] = {
     1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,
     1,  0,
     1, -1, -1,  1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,  1, -1, -1,  1, -1,  1, -1,  1,  1,  1,  1
};
double ltfValue(int k) { return (k < -26 || k > 26) ? 0.0 : static_cast<double>(LTF_SYM[k + 26]); }
bool isPilot(int k) { return k == -21 || k == -7 || k == 7 || k == 21; }
double pilotBaseSign(int k) { return k == 21 ? -1.0 : 1.0; }
bool isLegacyData(int k) { return k != 0 && k >= -26 && k <= 26 && !isPilot(k); }
bool isHtData(int k) { return k != 0 && k >= -28 && k <= 28 && !isPilot(k); }

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
  return t;
}

std::vector<cd> withGuardInterval(const std::vector<cd>& symbol64, int giLen)
{
  std::vector<cd> out(giLen + FFT_SIZE);
  for (int i = 0; i < giLen; ++i) out[i] = symbol64[FFT_SIZE - giLen + i];
  for (int i = 0; i < FFT_SIZE; ++i) out[giLen + i] = symbol64[i];
  return out;
}

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

// Inverse of the core's deinterleaveSignalField (n_bpsc=1 case, nCbps=48), used both for the
// (unused-by-receiver) L-SIG and for each of the two individually-interleaved HT-SIG symbols.
std::vector<int> interleaveSignalField(const std::vector<int>& coded)
{
  const int nCbps = 48;
  std::vector<int> out(nCbps);
  for (int k = 0; k < nCbps; ++k) {
    const int second = 16 * k - (nCbps - 1) * static_cast<int>(std::floor(16.0 * k / nCbps));
    out[k] = coded[second];
  }
  return out;
}

// Builds a complete HT mixed-format packet: L-STF + L-LTF + L-SIG(dummy BPSK content, never
// decoded by the receiver under test) + HT-SIG1/HT-SIG2 (real rate-1/2 encoding, MCS0/20MHz)
// + HT-STF/HT-LTF(dummy, unused - channel estimate is reused from L-LTF) + DATA field on the
// 56-subcarrier HT grid (52 data + 4 pilot), BPSK content for MCS0.
std::vector<cd> buildHtPacket(int psduLengthBytes, double frequencyErrorHz, double sampleRate,
    double noiseAmplitude, unsigned seed)
{
  unsigned state = seed;
  auto uniform = [&]() { state = state * 1664525u + 1013904223u; return static_cast<double>(state >> 8) / 16777216.0; };

  // L-STF: 10 repetitions of a 16 sample token (content irrelevant - receiver syncs on L-LTF).
  std::vector<cd> stf;
  std::vector<cd> stfToken(16);
  for (auto& v : stfToken) v = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  for (int r = 0; r < 10; ++r) stf.insert(stf.end(), stfToken.begin(), stfToken.end());

  // L-LTF: 2x GI(32) + 2 copies of the known frequency-domain training symbol, per 17.3.3.
  std::vector<cd> ltfFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) { const double v = ltfValue(k); if (v != 0.0) ltfFreq[bin(k)] = cd(v, 0); }
  std::vector<cd> ltfTime = ofdmSymbolFromFreq(ltfFreq);
  std::vector<cd> ltfWithGi(32 + 2 * FFT_SIZE);
  for (int i = 0; i < 32; ++i) ltfWithGi[i] = ltfTime[FFT_SIZE - 32 + i];
  for (int i = 0; i < FFT_SIZE; ++i) { ltfWithGi[32 + i] = ltfTime[i]; ltfWithGi[32 + FFT_SIZE + i] = ltfTime[i]; }

  const std::vector<double> polarity = pilotPolarity(127);

  // L-SIG: never decoded by this receiver core, but must occupy exactly one 80 sample symbol
  // slot ahead of HT-SIG1 to match the core's fixed field offsets. Pseudo-random BPSK content
  // with correctly-signed pilots (so it does not perturb burst detection).
  std::vector<cd> lsigFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) {
    if (isPilot(k)) lsigFreq[bin(k)] = cd(pilotBaseSign(k) * polarity[0], 0.0);
    else if (isLegacyData(k)) lsigFreq[bin(k)] = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  }
  std::vector<cd> lsigWithGi = withGuardInterval(ofdmSymbolFromFreq(lsigFreq), 16);

  // HT-SIG: 48 info bits -> rate-1/2 conv encode -> 96 coded bits -> first 48 interleaved onto
  // HT-SIG1 (plain BPSK), next 48 onto HT-SIG2 (QBPSK: data subcarriers rotated +90 degrees,
  // pilots stay plain BPSK), per 19.3.9.4.3.
  std::vector<int> htSigBits(48, 0);
  const int mcs = 0;
  const int cbw = 0; // 20 MHz
  for (int b = 0; b < 7; ++b) htSigBits[b] = (mcs >> b) & 1;
  htSigBits[7] = cbw;
  for (int b = 0; b < 16; ++b) htSigBits[8 + b] = (psduLengthBytes >> b) & 1;
  // bits 24..47 (HT-SIG2 fields) left as zero: not decoded/verified by this core.

  const std::vector<int> htSigCoded = convolutionalEncodeRate1Half(htSigBits);
  std::vector<int> htSig1Coded(htSigCoded.begin(), htSigCoded.begin() + 48);
  std::vector<int> htSig2Coded(htSigCoded.begin() + 48, htSigCoded.begin() + 96);
  const std::vector<int> htSig1Interleaved = interleaveSignalField(htSig1Coded);
  const std::vector<int> htSig2Interleaved = interleaveSignalField(htSig2Coded);

  std::vector<cd> htSig1Freq(FFT_SIZE, cd(0, 0));
  std::vector<cd> htSig2Freq(FFT_SIZE, cd(0, 0));
  int idx1 = 0, idx2 = 0;
  for (int k = -26; k <= 26; ++k) {
    if (isPilot(k)) {
      htSig1Freq[bin(k)] = cd(pilotBaseSign(k) * polarity[1], 0.0);
      htSig2Freq[bin(k)] = cd(pilotBaseSign(k) * polarity[2], 0.0);
    } else if (isLegacyData(k)) {
      htSig1Freq[bin(k)] = htSig1Interleaved[idx1++] ? cd(1, 0) : cd(-1, 0);
      const double b2 = htSig2Interleaved[idx2++] ? 1.0 : -1.0;
      htSig2Freq[bin(k)] = cd(0.0, b2); // +90 degree QBPSK rotation
    }
  }
  std::vector<cd> htSig1WithGi = withGuardInterval(ofdmSymbolFromFreq(htSig1Freq), 16);
  std::vector<cd> htSig2WithGi = withGuardInterval(ofdmSymbolFromFreq(htSig2Freq), 16);

  // HT-STF/HT-LTF: never decoded by this core (channel estimate is reused from L-LTF), but
  // must occupy exactly one 80 sample symbol slot each to match the core's fixed offsets.
  std::vector<cd> htStfWithGi(80, cd(0, 0));
  for (auto& v : htStfWithGi) v = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  std::vector<cd> htLtfFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) { const double v = ltfValue(k); if (v != 0.0) htLtfFreq[bin(k)] = cd(v, 0); }
  std::vector<cd> htLtfWithGi = withGuardInterval(ofdmSymbolFromFreq(htLtfFreq), 16);

  // DATA field: ceil((16+8*psduLengthBytes+6)/26) symbols for MCS0 (26 data bits per symbol),
  // BPSK on the wider 56-subcarrier (52 data + 4 pilot) HT grid. p_n index continues from
  // HT-SIG2 (idx 2) through HT-STF/HT-LTF (idx 3,4, not pilot-tracked) to the first DATA
  // symbol at idx 5, matching the core's (sym + 5) % 127 convention.
  const int nDataSym = static_cast<int>(std::ceil((16.0 + 8.0 * psduLengthBytes + 6.0) / 26.0));
  std::vector<cd> dataField;
  for (int sym = 0; sym < nDataSym; ++sym) {
    std::vector<cd> dataFreq(FFT_SIZE, cd(0, 0));
    const int dataN = (sym + 5) % 127;
    for (int k = -28; k <= 28; ++k) {
      if (isPilot(k)) dataFreq[bin(k)] = cd(pilotBaseSign(k) * polarity[dataN], 0.0);
      else if (isHtData(k)) dataFreq[bin(k)] = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
    }
    std::vector<cd> dataTime = ofdmSymbolFromFreq(dataFreq);
    std::vector<cd> dataWithGi = withGuardInterval(dataTime, 16);
    dataField.insert(dataField.end(), dataWithGi.begin(), dataWithGi.end());
  }

  std::vector<cd> packet;
  packet.insert(packet.end(), stf.begin(), stf.end());
  packet.insert(packet.end(), ltfWithGi.begin(), ltfWithGi.end());
  packet.insert(packet.end(), lsigWithGi.begin(), lsigWithGi.end());
  packet.insert(packet.end(), htSig1WithGi.begin(), htSig1WithGi.end());
  packet.insert(packet.end(), htSig2WithGi.begin(), htSig2WithGi.end());
  packet.insert(packet.end(), htStfWithGi.begin(), htStfWithGi.end());
  packet.insert(packet.end(), htLtfWithGi.begin(), htLtfWithGi.end());
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

void testBurstSyncAndHtSigField()
{
  const double sampleRate = 20.0e6;
  std::vector<cd> packet = buildHtPacket(100, 0.0, sampleRate, 0.01, 1u);
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
  CHECK(result.mcsIndex == 0);
  CHECK(result.modulation == MOD_BPSK);
  CHECK(result.codingRateNumerator == 1);
  CHECK(result.codingRateDenominator == 2);
  CHECK(result.bandwidthMHz == 20);
  CHECK(result.psduLengthBytes == 100);
  CHECK(result.rmsEvmPercent < 10.0);
}

void testCarrierFrequencyErrorRecovery()
{
  const double sampleRate = 20.0e6;
  const double injectedOffsetHz = 50e3;
  std::vector<cd> packet = buildHtPacket(50, injectedOffsetHz, sampleRate, 0.01, 3u);
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
  testBurstSyncAndHtSigField();
  testCarrierFrequencyErrorRecovery();
  testNoBurstWhenCaptureIsEmpty();
  testNoiseOnlyCaptureFailsSync();

  if (gFailures == 0) {
    std::printf("All WiFi 802.11n core tests passed\n");
    return 0;
  }
  std::printf("%d check(s) failed\n", gFailures);
  return 1;
}
