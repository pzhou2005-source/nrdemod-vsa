/**
 ****************************************************************************
 *
 * Standalone checks for the native IEEE 802.11ac (VHT, 20 MHz, Nsts=1) OFDM
 * core (no test framework), mirroring Wifi11nOfdmCoreTest.cpp. Builds a
 * complete, spec-accurate VHT single-user packet from scratch (L-STF +
 * L-LTF + L-SIG(dummy, unused by the receiver) + VHT-SIG-A1/A2 (uncoded
 * BPSK bit fields, no FEC) + VHT-STF/VHT-LTF/VHT-SIG-B(dummy, unused -
 * channel estimate is reused from L-LTF) + DATA field on the 56-subcarrier
 * VHT grid) and verifies the receiver recovers burst/sync, the VHT-SIG-A
 * field content (bandwidth/NSTS/MCS) and a low EVM on the DATA field.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11ac/Wifi11acOfdmCore.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace wifi11acdemod;

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

const int LTF_SYM[53] = {
     1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,
     1,  0,
     1, -1, -1,  1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,  1, -1, -1,  1, -1,  1, -1,  1,  1,  1,  1
};
double ltfValue(int k) { return (k < -26 || k > 26) ? 0.0 : static_cast<double>(LTF_SYM[k + 26]); }
bool isPilot(int k) { return k == -21 || k == -7 || k == 7 || k == 21; }
double pilotBaseSign(int k) { return k == 21 ? -1.0 : 1.0; }
bool isLegacyData(int k) { return k != 0 && k >= -26 && k <= 26 && !isPilot(k); }
bool isVhtData(int k) { return k != 0 && k >= -28 && k <= 28 && !isPilot(k); }

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

// Inverse of the core's deinterleaveSignalField (n_bpsc=1 case, nCbps=48), used for each of
// the two independently-interleaved VHT-SIG-A symbols.
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

int parityOf(unsigned v) { int p = 0; while (v) { p ^= v & 1u; v >>= 1; } return p; }

// Rate-1/2 K=7 convolutional encode of a self-contained 24-info-bit payload (same taps as the
// core's decoder); VHT-SIG-A1/A2 are each independently encoded this way (21.3.8.3.3).
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

// Builds a complete VHT SU packet: L-STF + L-LTF + L-SIG(dummy) + VHT-SIG-A1/A2 (each
// independently rate-1/2 encoded, BW=20MHz/NSTS=1/given MCS) + VHT-STF/VHT-LTF(x1)/
// VHT-SIG-B(dummy, unused) + DATA field on the 56-subcarrier VHT grid (52 data + 4 pilot).
std::vector<cd> buildVhtPacket(int mcs, int numDataSymbols, double frequencyErrorHz, double sampleRate,
    double noiseAmplitude, unsigned seed)
{
  unsigned state = seed;
  auto uniform = [&]() { state = state * 1664525u + 1013904223u; return static_cast<double>(state >> 8) / 16777216.0; };

  std::vector<cd> stf;
  std::vector<cd> stfToken(16);
  for (auto& v : stfToken) v = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  for (int r = 0; r < 10; ++r) stf.insert(stf.end(), stfToken.begin(), stfToken.end());

  std::vector<cd> ltfFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) { const double v = ltfValue(k); if (v != 0.0) ltfFreq[bin(k)] = cd(v, 0); }
  std::vector<cd> ltfTime = ofdmSymbolFromFreq(ltfFreq);
  std::vector<cd> ltfWithGi(32 + 2 * FFT_SIZE);
  for (int i = 0; i < 32; ++i) ltfWithGi[i] = ltfTime[FFT_SIZE - 32 + i];
  for (int i = 0; i < FFT_SIZE; ++i) { ltfWithGi[32 + i] = ltfTime[i]; ltfWithGi[32 + FFT_SIZE + i] = ltfTime[i]; }

  const std::vector<double> polarity = pilotPolarity(127);

  std::vector<cd> lsigFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) {
    if (isPilot(k)) lsigFreq[bin(k)] = cd(pilotBaseSign(k) * polarity[0], 0.0);
    else if (isLegacyData(k)) lsigFreq[bin(k)] = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  }
  std::vector<cd> lsigWithGi = withGuardInterval(ofdmSymbolFromFreq(lsigFreq), 16);

  // VHT-SIG-A1/A2: 24 info bits each, independently rate-1/2 encoded (-> 48 coded bits each).
  // BW=0(20MHz), STBC=0, Group ID=0(SU), NSTS-1=0(Nsts=1), rest zero/don't-care for this test.
  std::vector<int> sigA1(24, 0);
  sigA1[0] = 0; sigA1[1] = 0;   // BW=00 (20 MHz)
  sigA1[10] = 0; sigA1[11] = 0; sigA1[12] = 0; // NSTS-1 = 0 -> Nsts=1
  std::vector<int> sigA2(24, 0);
  for (int b = 0; b < 4; ++b) sigA2[4 + b] = (mcs >> b) & 1; // MCS

  const std::vector<int> sigA1Coded = convolutionalEncodeRate1Half(sigA1);
  const std::vector<int> sigA2Coded = convolutionalEncodeRate1Half(sigA2);
  const std::vector<int> sigA1Interleaved = interleaveSignalField(sigA1Coded);
  const std::vector<int> sigA2Interleaved = interleaveSignalField(sigA2Coded);

  std::vector<cd> sigA1Freq(FFT_SIZE, cd(0, 0));
  std::vector<cd> sigA2Freq(FFT_SIZE, cd(0, 0));
  int idx1 = 0, idx2 = 0;
  for (int k = -26; k <= 26; ++k) {
    if (isPilot(k)) {
      sigA1Freq[bin(k)] = cd(pilotBaseSign(k) * polarity[1], 0.0);
      sigA2Freq[bin(k)] = cd(pilotBaseSign(k) * polarity[2], 0.0);
    } else if (isLegacyData(k)) {
      sigA1Freq[bin(k)] = sigA1Interleaved[idx1++] ? cd(1, 0) : cd(-1, 0);
      sigA2Freq[bin(k)] = sigA2Interleaved[idx2++] ? cd(0, 1) : cd(0, -1); // VHT-SIG-A2 is QBPSK
    }
  }
  std::vector<cd> sigA1WithGi = withGuardInterval(ofdmSymbolFromFreq(sigA1Freq), 16);
  std::vector<cd> sigA2WithGi = withGuardInterval(ofdmSymbolFromFreq(sigA2Freq), 16);

  // VHT-STF/VHT-LTF(x1)/VHT-SIG-B: never decoded by this core, each occupies exactly one
  // 80 sample symbol slot to match the core's fixed field offsets.
  std::vector<cd> vhtStfWithGi(80, cd(0, 0));
  for (auto& v : vhtStfWithGi) v = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
  std::vector<cd> vhtLtfFreq(FFT_SIZE, cd(0, 0));
  for (int k = -26; k <= 26; ++k) { const double v = ltfValue(k); if (v != 0.0) vhtLtfFreq[bin(k)] = cd(v, 0); }
  std::vector<cd> vhtLtfWithGi = withGuardInterval(ofdmSymbolFromFreq(vhtLtfFreq), 16);
  std::vector<cd> vhtSigBWithGi(80, cd(0, 0));
  for (auto& v : vhtSigBWithGi) v = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);

  // DATA field: BPSK (MCS0) on the 56-subcarrier VHT grid. p_n index continues from
  // VHT-SIG-A2 (idx 2) through VHT-STF/VHT-LTF/VHT-SIG-B (idx 3,4,5, not pilot-tracked) to the
  // first DATA symbol at idx 6, matching the core's (sym + 6) % 127 convention.
  std::vector<cd> dataField;
  for (int sym = 0; sym < numDataSymbols; ++sym) {
    std::vector<cd> dataFreq(FFT_SIZE, cd(0, 0));
    const int dataN = (sym + 6) % 127;
    for (int k = -28; k <= 28; ++k) {
      if (isPilot(k)) {
        // VHT data pilots cycle {1,1,1,-1} by one position per symbol
        const int j = (k == -21) ? 0 : (k == -7) ? 1 : (k == 7) ? 2 : 3;
        const double base[4] = {1.0, 1.0, 1.0, -1.0};
        dataFreq[bin(k)] = cd(base[(j + sym) % 4] * polarity[dataN], 0.0);
      }
      else if (isVhtData(k)) dataFreq[bin(k)] = uniform() < 0.5 ? cd(1, 0) : cd(-1, 0);
    }
    std::vector<cd> dataTime = ofdmSymbolFromFreq(dataFreq);
    std::vector<cd> dataWithGi = withGuardInterval(dataTime, 16);
    dataField.insert(dataField.end(), dataWithGi.begin(), dataWithGi.end());
  }

  std::vector<cd> packet;
  packet.insert(packet.end(), stf.begin(), stf.end());
  packet.insert(packet.end(), ltfWithGi.begin(), ltfWithGi.end());
  packet.insert(packet.end(), lsigWithGi.begin(), lsigWithGi.end());
  packet.insert(packet.end(), sigA1WithGi.begin(), sigA1WithGi.end());
  packet.insert(packet.end(), sigA2WithGi.begin(), sigA2WithGi.end());
  packet.insert(packet.end(), vhtStfWithGi.begin(), vhtStfWithGi.end());
  packet.insert(packet.end(), vhtLtfWithGi.begin(), vhtLtfWithGi.end());
  packet.insert(packet.end(), vhtSigBWithGi.begin(), vhtSigBWithGi.end());
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

void testBurstSyncAndSigAField()
{
  const double sampleRate = 20.0e6;
  const int numDataSymbols = 10;
  std::vector<cd> packet = buildVhtPacket(0, numDataSymbols, 0.0, sampleRate, 0.01, 1u);
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
  CHECK(result.bandwidthMHz == 20);
  CHECK(result.nsts == 1);
  CHECK(result.mcsIndex == 0);
  CHECK(result.modulation == MOD_BPSK);
  CHECK(result.codingRateNumerator == 1);
  CHECK(result.codingRateDenominator == 2);
  CHECK(result.numDataSymbols >= numDataSymbols - 2 && result.numDataSymbols <= numDataSymbols + 2);
  CHECK(result.rmsEvmPercent < 10.0);
}

void testCarrierFrequencyErrorRecovery()
{
  const double sampleRate = 20.0e6;
  const double injectedOffsetHz = 50e3;
  std::vector<cd> packet = buildVhtPacket(0, 8, injectedOffsetHz, sampleRate, 0.01, 3u);
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
  testBurstSyncAndSigAField();
  testCarrierFrequencyErrorRecovery();
  testNoBurstWhenCaptureIsEmpty();
  testNoiseOnlyCaptureFailsSync();

  if (gFailures == 0) {
    std::printf("All WiFi 802.11ac core tests passed\n");
    return 0;
  }
  std::printf("%d check(s) failed\n", gFailures);
  return 1;
}
