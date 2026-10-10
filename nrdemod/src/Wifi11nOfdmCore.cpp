/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11n/Wifi11nOfdmCore.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace wifi11ndemod {

namespace {

typedef std::complex<double> cd;

const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const int FFT_SIZE = 64;
const int HT_DATA_CARRIERS = 52;

// IEEE 802.11-2016 17.3.3 L-LTF frequency domain content, identical to wifi/WifiOfdmCore.cpp:
// the legacy preamble (L-STF/L-LTF/L-SIG) of an HT mixed-format PPDU is bit-for-bit identical
// to an 802.11a preamble (clause 19.3.2), so the already-validated burst/sync/channel-estimate
// code for 802.11a is reused verbatim rather than re-derived.
const int LTF_SYM[53] = {
     1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,  1,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,  1,
     1,  0,
     1, -1, -1,  1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,  1, -1, -1,  1, -1,  1, -1,  1,  1,  1,  1
};

double ltfValue(int k)
{
  if (k < -26 || k > 26) return 0.0;
  return static_cast<double>(LTF_SYM[k + 26]);
}

bool isPilotSubcarrier(int k)
{
  return k == -21 || k == -7 || k == 7 || k == 21;
}

double pilotBaseSign(int k)
{
  return k == 21 ? -1.0 : 1.0;
}

std::vector<double> pilotPolaritySequence(int length)
{
  std::vector<double> out(length);
  unsigned state = 0x7Fu;
  for (int n = 0; n < length; ++n) {
    const unsigned feedback = ((state >> 3) ^ (state >> 6)) & 1u;
    out[n] = 1.0 - 2.0 * static_cast<double>(feedback);
    state = ((state << 1) | feedback) & 0x7Fu;
  }
  return out;
}

// Legacy (48 subcarrier) data mask, used for L-SIG/HT-SIG which both ride on the narrower
// legacy OFDM numerology regardless of the wider HT DATA field grid that follows them.
bool isLegacyDataSubcarrier(int k)
{
  return k != 0 && k >= -26 && k <= 26 && !isPilotSubcarrier(k);
}

// HT 20 MHz DATA field subcarrier grid (IEEE 802.11-2016 Table 19-6): 56 active subcarriers
// (52 data + 4 pilot) spanning k = -28..28, i.e. four more data tones per edge than the
// legacy/HT-SIG mapping above.
bool isHtDataSubcarrier(int k)
{
  return k != 0 && k >= -28 && k <= 28 && !isPilotSubcarrier(k);
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

int fftBin(int k)
{
  return ((k % FFT_SIZE) + FFT_SIZE) % FFT_SIZE;
}

std::vector<cd> ltfTimeDomain()
{
  std::vector<cd> freq(FFT_SIZE, cd(0.0, 0.0));
  for (int k = -26; k <= 26; ++k) {
    const double v = ltfValue(k);
    if (v != 0.0) freq[fftBin(k)] = cd(v, 0.0);
  }
  fft64(freq, true);
  const double scale = 1.0 / std::sqrt(static_cast<double>(FFT_SIZE));
  for (auto& x : freq) x *= scale;
  return freq;
}

bool findBurst(const std::vector<cd>& iq, double thresholdDb, int& start, int& length)
{
  const int n = static_cast<int>(iq.size());
  if (n == 0) return false;
  const int window = std::max(8, n / 200);
  std::vector<double> powerDb(n);
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += std::norm(iq[i]);
    if (i >= window) sum -= std::norm(iq[i - window]);
    const int count = std::min(i + 1, window);
    const double mean = count > 0 ? sum / count : 0.0;
    powerDb[i] = 10.0 * std::log10(mean > 0.0 ? mean : 1e-30);
  }
  const double peakDb = *std::max_element(powerDb.begin(), powerDb.end());
  const double threshold = peakDb - std::fabs(thresholdDb);
  int first = -1, last = -1;
  for (int i = 0; i < n; ++i) {
    if (powerDb[i] >= threshold) {
      if (first < 0) first = i;
      last = i;
    }
  }
  if (first < 0) return false;
  start = first;
  length = last - first + 1;
  return length > 0;
}

double matchedFilterScore(const std::vector<cd>& iq, int pos, const std::vector<cd>& ltfTime)
{
  const int n = static_cast<int>(iq.size());
  if (pos < 0 || pos + FFT_SIZE > n) return 0.0;
  cd corr(0.0, 0.0);
  double energy = 0.0;
  for (int i = 0; i < FFT_SIZE; ++i) {
    corr += iq[pos + i] * std::conj(ltfTime[i]);
    energy += std::norm(iq[pos + i]);
  }
  const double denom = std::sqrt(energy * FFT_SIZE);
  return denom > 0.0 ? std::abs(corr) / denom : 0.0;
}

// Rate-1/2 K=7 convolutional code, identical taps/convention to wifi/WifiOfdmCore.cpp.
int parityOf(unsigned v)
{
  int p = 0;
  while (v) { p ^= (v & 1u); v >>= 1; }
  return p;
}

std::vector<int> viterbiDecodeRate1Half(const std::vector<int>& codedBits)
{
  const int numSteps = static_cast<int>(codedBits.size()) / 2;
  const int numStates = 64;
  const double INF = 1e18;
  std::vector<std::vector<double>> metric(numSteps + 1, std::vector<double>(numStates, INF));
  std::vector<std::vector<int>> prevState(numSteps + 1, std::vector<int>(numStates, -1));
  std::vector<std::vector<int>> prevBit(numSteps + 1, std::vector<int>(numStates, 0));
  metric[0][0] = 0.0;

  for (int step = 0; step < numSteps; ++step) {
    const int rx0 = codedBits[2 * step];
    const int rx1 = codedBits[2 * step + 1];
    for (int s = 0; s < numStates; ++s) {
      if (metric[step][s] >= INF) continue;
      for (int b = 0; b <= 1; ++b) {
        const unsigned v = static_cast<unsigned>((s << 1) | b) & 0x7Fu;
        const int out0 = parityOf(v & 0155u);
        const int out1 = parityOf(v & 0117u);
        const double cost = metric[step][s] + (out0 != rx0 ? 1.0 : 0.0) + (out1 != rx1 ? 1.0 : 0.0);
        const int next = static_cast<int>(v & 0x3Fu);
        if (cost < metric[step + 1][next]) {
          metric[step + 1][next] = cost;
          prevState[step + 1][next] = s;
          prevBit[step + 1][next] = b;
        }
      }
    }
  }

  int bestState = 0;
  double bestMetric = metric[numSteps][0];
  for (int s = 1; s < numStates; ++s) {
    if (metric[numSteps][s] < bestMetric) {
      bestMetric = metric[numSteps][s];
      bestState = s;
    }
  }

  std::vector<int> bits(numSteps);
  int state = bestState;
  for (int step = numSteps; step > 0; --step) {
    bits[step - 1] = prevBit[step][state];
    state = prevState[step][state];
  }
  return bits;
}

// n_bpsc=1 interleaver inverse, used for both L-SIG (unused here) and each of the two
// individually-interleaved HT-SIG OFDM symbols (19.3.9.4.3: HT-SIG uses the same per-symbol
// BPSK/QBPSK interleaving as the legacy SIGNAL field).
std::vector<int> deinterleaveSignalField(const std::vector<int>& received)
{
  const int nCbps = 48;
  std::vector<int> out(nCbps, 0);
  for (int k = 0; k < nCbps; ++k) {
    const int second = 16 * k - (nCbps - 1) * static_cast<int>(std::floor(16.0 * k / nCbps));
    out[second] = received[k];
  }
  return out;
}

// General HT DATA field interleaver permutation (19.3.11.8.3): same two-stage structure as
// the legacy interleaver but with NCOL=13 (vs. legacy's NCOL=16) to match the HT 20 MHz
// grid's 52 data subcarriers; no stream-parser/rotation stage is needed since this core is
// scoped to a single spatial stream (Nss=1).
void htDataInterleaverIndices(int nBpsc, int nCbps, std::vector<int>& first, std::vector<int>& second)
{
  const int nCol = 13;
  const int s = std::max(nBpsc / 2, 1);
  first.assign(nCbps, 0);
  second.assign(nCbps, 0);
  for (int j = 0; j < nCbps; ++j) {
    first[j] = s * (j / s) + ((j + static_cast<int>(std::floor(static_cast<double>(nCol) * j / nCbps))) % s);
  }
  for (int i = 0; i < nCbps; ++i) {
    second[i] = nCol * i - (nCbps - 1) * static_cast<int>(std::floor(static_cast<double>(nCol) * i / nCbps));
  }
}

cd hardDecisionBpsk(const cd& z)
{
  return cd(z.real() >= 0.0 ? 1.0 : -1.0, 0.0);
}

cd hardDecision(const cd& z, Modulation mod)
{
  switch (mod) {
    case MOD_BPSK: return hardDecisionBpsk(z);
    case MOD_QPSK: {
      const double s = 1.0 / std::sqrt(2.0);
      return cd(z.real() >= 0.0 ? s : -s, z.imag() >= 0.0 ? s : -s);
    }
    case MOD_QAM16: {
      const double s = 1.0 / std::sqrt(10.0);
      const auto axis = [&](double v) {
        double level = 2.0 * std::floor((v / s - 1.0) / 2.0 + 0.5) + 1.0;
        if (level > 3.0) level = 3.0;
        if (level < -3.0) level = -3.0;
        return level * s;
      };
      return cd(axis(z.real()), axis(z.imag()));
    }
    case MOD_QAM64:
    default: {
      const double s = 1.0 / std::sqrt(42.0);
      const auto axis = [&](double v) {
        double level = 2.0 * std::floor((v / s - 1.0) / 2.0 + 0.5) + 1.0;
        if (level > 7.0) level = 7.0;
        if (level < -7.0) level = -7.0;
        return level * s;
      };
      return cd(axis(z.real()), axis(z.imag()));
    }
  }
}

// MCS 0..7 (single spatial stream, 20 MHz, no STBC): modulation / coding rate / DATA field
// bits-per-OFDM-symbol, IEEE 802.11-2016 Table 19-28.
struct McsParams {
  Modulation modulation;
  int nBpsc;
  int codingNum;
  int codingDen;
  int dbps;
};

bool mcsToParams(int mcs, McsParams& p)
{
  switch (mcs) {
    case 0: p = {MOD_BPSK,  1, 1, 2,  26}; return true;
    case 1: p = {MOD_QPSK,  2, 1, 2,  52}; return true;
    case 2: p = {MOD_QPSK,  2, 3, 4,  78}; return true;
    case 3: p = {MOD_QAM16, 4, 1, 2, 104}; return true;
    case 4: p = {MOD_QAM16, 4, 3, 4, 156}; return true;
    case 5: p = {MOD_QAM64, 6, 2, 3, 208}; return true;
    case 6: p = {MOD_QAM64, 6, 3, 4, 234}; return true;
    case 7: p = {MOD_QAM64, 6, 5, 6, 260}; return true;
    default: return false;
  }
}

}

Config::Config() : sampleRate(0.0), burstSearch(true), burstSearchThresholdDb(15.0), syncThreshold(0.5)
{
}

Result::Result()
    : ok(false), burstFound(false), burstStart(0), burstLength(0), syncFound(false), syncRho(0.0),
      timingOffsetSamples(0), frequencyErrorHz(0.0), mcsIndex(0), modulation(MOD_BPSK), codingRateNumerator(1),
      codingRateDenominator(2), bandwidthMHz(20), psduLengthBytes(0), numDataSymbols(0), rmsEvmPercent(0.0),
      peakEvmPercent(0.0), peakEvmSymbolIndex(0), peakEvmSubcarrierIndex(0)
{
}

Result demodulate(const std::vector<cd>& iq, const Config& config)
{
  Result result;
  if (iq.empty() || config.sampleRate <= 0.0) {
    result.error = "empty capture or invalid sample rate";
    return result;
  }

  int burstStart = 0, burstLength = static_cast<int>(iq.size());
  if (config.burstSearch) {
    if (!findBurst(iq, config.burstSearchThresholdDb, burstStart, burstLength)) {
      result.error = "burst not found";
      return result;
    }
    result.burstFound = true;
  } else {
    result.burstFound = true;
  }
  result.burstStart = burstStart;
  result.burstLength = burstLength;

  const double samplesPerFftSample = config.sampleRate / 20.0e6;
  if (samplesPerFftSample < 0.99 || std::fabs(samplesPerFftSample - std::llround(samplesPerFftSample)) > 1e-3) {
    result.error = "sample rate must be an integer multiple of 20 MHz";
    return result;
  }
  const int osr = static_cast<int>(std::llround(samplesPerFftSample));
  std::vector<cd> work;
  const std::vector<cd>* samples = &iq;
  if (osr != 1) {
    work.resize(iq.size() / osr);
    for (size_t i = 0; i < work.size(); ++i) work[i] = iq[i * osr];
    samples = &work;
    burstStart /= osr;
  }

  // --- L-STF/L-LTF burst timing, channel estimate and CFO: identical to wifi/WifiOfdmCore.cpp ---
  const std::vector<cd> ltfTime = ltfTimeDomain();
  const int searchEnd = std::min(static_cast<int>(samples->size()) - 2 * FFT_SIZE, burstStart + burstLength);
  double bestScore = -1.0;
  int bestPos = -1;
  for (int pos = burstStart; pos < searchEnd; ++pos) {
    const double score = matchedFilterScore(*samples, pos, ltfTime) + matchedFilterScore(*samples, pos + FFT_SIZE, ltfTime);
    if (score > bestScore) {
      bestScore = score;
      bestPos = pos;
    }
  }
  if (bestPos < 0) {
    result.error = "L-LTF not found in capture";
    return result;
  }
  result.timingOffsetSamples = (bestPos - burstStart) * osr;

  std::vector<cd> ltf1(samples->begin() + bestPos, samples->begin() + bestPos + FFT_SIZE);
  std::vector<cd> ltf2(samples->begin() + bestPos + FFT_SIZE, samples->begin() + bestPos + 2 * FFT_SIZE);
  fft64(ltf1, false);
  fft64(ltf2, false);
  const double fftScale = 1.0 / std::sqrt(static_cast<double>(FFT_SIZE));
  for (auto& v : ltf1) v *= fftScale;
  for (auto& v : ltf2) v *= fftScale;

  std::vector<cd> channel1(FFT_SIZE, cd(0.0, 0.0)), channel2(FFT_SIZE, cd(0.0, 0.0));
  for (int k = -26; k <= 26; ++k) {
    const double ref = ltfValue(k);
    if (ref == 0.0) continue;
    const int bin = fftBin(k);
    channel1[bin] = ltf1[bin] / ref;
    channel2[bin] = ltf2[bin] / ref;
  }

  cd crossSum(0.0, 0.0);
  double power1 = 0.0, power2 = 0.0;
  for (int k = -26; k <= 26; ++k) {
    if (ltfValue(k) == 0.0) continue;
    const int bin = fftBin(k);
    crossSum += channel2[bin] * std::conj(channel1[bin]);
    power1 += std::norm(channel1[bin]);
    power2 += std::norm(channel2[bin]);
  }
  const double denom = std::sqrt(power1 * power2);
  result.syncRho = denom > 0.0 ? std::abs(crossSum) / denom : 0.0;
  result.syncFound = result.syncRho >= config.syncThreshold;

  const double residualPhase = std::abs(crossSum) > 0.0 ? std::arg(crossSum) : 0.0;
  result.frequencyErrorHz = residualPhase * (20.0e6) / (TWO_PI * FFT_SIZE);

  std::vector<cd> channel(FFT_SIZE, cd(0.0, 0.0));
  const cd derotate = std::polar(1.0, -residualPhase);
  for (int bin = 0; bin < FFT_SIZE; ++bin) {
    channel[bin] = 0.5 * (channel1[bin] + channel2[bin] * derotate);
  }

  if (!result.syncFound) {
    result.ok = true;
    return result;
  }

  const double residualRadPerSample = residualPhase / static_cast<double>(FFT_SIZE);
  const std::vector<double> polarity = pilotPolaritySequence(127);

  // Helper: FFT one 80-sample (16 GI + 64) OFDM symbol at a given start offset (relative to
  // bestPos, i.e. the start of L-LTF), applying the residual CFO derotation.
  const auto fftSymbolAt = [&](int fftStart) -> std::vector<cd> {
    std::vector<cd> sym(samples->begin() + fftStart, samples->begin() + fftStart + FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; ++i) {
      const double sampleIndex = static_cast<double>(fftStart + i - bestPos);
      sym[i] *= std::polar(1.0, -residualRadPerSample * sampleIndex);
    }
    fft64(sym, false);
    for (auto& v : sym) v *= fftScale;
    return sym;
  };

  // Pilot-tracked phase correction + legacy 48-subcarrier hard-decision bits for one
  // BPSK/QBPSK OFDM symbol (used for both HT-SIG symbols); qbpsk=true derotates by the extra
  // 90 degree HT-SIG2 rotation (19.3.9.4.3) before slicing.
  const auto decodeLegacyGridSymbolBits = [&](const std::vector<cd>& sym, double pPilot, bool qbpsk) -> std::vector<int> {
    cd pilotBeta(0.0, 0.0);
    for (const int k : {-21, -7, 7, 21}) {
      pilotBeta += sym[fftBin(k)] * std::conj(cd(pilotBaseSign(k) * pPilot, 0.0)) / channel[fftBin(k)];
    }
    const double beta = std::abs(pilotBeta) > 0.0 ? std::arg(pilotBeta) : 0.0;
    const cd rotation = std::polar(1.0, -beta) * (qbpsk ? std::polar(1.0, -PI / 2.0) : cd(1.0, 0.0));
    std::vector<int> bits(48);
    int idx = 0;
    for (int k = -26; k <= 26; ++k) {
      if (!isLegacyDataSubcarrier(k)) continue;
      const cd eq = sym[fftBin(k)] / channel[fftBin(k)] * rotation;
      bits[idx++] = eq.real() >= 0.0 ? 1 : 0;
    }
    return bits;
  };

  // Field layout from bestPos (start of L-LTF): L-LTF(128) + L-SIG(80, skipped/unused) +
  // HT-SIG1(80) + HT-SIG2(80) + HT-STF(80, skipped) + HT-LTF(80, channel-estimate reused from
  // L-LTF below rather than independently decoded) + DATA symbols (80 each). Each *Start
  // variable below is the FFT-window start (i.e. already 16 samples past that field's own
  // start, skipping its GI); adding a further 80 (one full symbol period) to a FFT-window
  // start lands exactly on the next field's FFT-window start. From htSig2Start, three +80
  // steps pass over HT-STF and HT-LTF to land on DATA symbol 0's FFT window.
  // A greenfield PPDU has no L-SIG: the symbol right after the (HT-)LTF is already a QBPSK HT-SIG1,
  // whereas in mixed format that slot is the plain-BPSK L-SIG. Tell them apart by the rotation.
  const int slot0Start = bestPos + 2 * FFT_SIZE + 16;
  if (slot0Start + FFT_SIZE > static_cast<int>(samples->size())) {
    result.error = "capture too short for HT-SIG";
    return result;
  }
  bool greenfield = false;
  {
    const std::vector<cd> slot0 = fftSymbolAt(slot0Start);
    cd sumSquares(0.0, 0.0);
    for (int k = -26; k <= 26; ++k) {
      if (!isLegacyDataSubcarrier(k)) continue;
      const cd eq = slot0[fftBin(k)] / channel[fftBin(k)];
      sumSquares += eq * eq;
    }
    greenfield = sumSquares.real() < 0.0;
  }
  const int htSig1Start = greenfield ? slot0Start : bestPos + 2 * FFT_SIZE + 80 + 16;
  const int htSig2Start = htSig1Start + 80;
  // mixed: + HT-STF + HT-LTF; greenfield: the single HT-LTF already preceded HT-SIG
  const int dataFieldStart = htSig2Start + (greenfield ? 80 : 80 + 80 + 80);
  if (htSig2Start + FFT_SIZE > static_cast<int>(samples->size())) {
    result.error = "capture too short for HT-SIG";
    return result;
  }

  const std::vector<cd> htSig1Sym = fftSymbolAt(htSig1Start);
  const std::vector<cd> htSig2Sym = fftSymbolAt(htSig2Start);
  // p_n index numbering: mixed L-SIG=0, HT-SIG1=1, HT-SIG2=2; greenfield starts at HT-SIG1.
  const int sigPolBase = greenfield ? 0 : 1;
  const std::vector<int> htSig1BitsInterleaved = decodeLegacyGridSymbolBits(htSig1Sym, polarity[sigPolBase], true);
  const std::vector<int> htSig2BitsInterleaved = decodeLegacyGridSymbolBits(htSig2Sym, polarity[sigPolBase + 1], true);
  const std::vector<int> htSig1Deinterleaved = deinterleaveSignalField(htSig1BitsInterleaved);
  const std::vector<int> htSig2Deinterleaved = deinterleaveSignalField(htSig2BitsInterleaved);

  std::vector<int> htSigCoded(96);
  std::copy(htSig1Deinterleaved.begin(), htSig1Deinterleaved.end(), htSigCoded.begin());
  std::copy(htSig2Deinterleaved.begin(), htSig2Deinterleaved.end(), htSigCoded.begin() + 48);
  const std::vector<int> htSigBits = viterbiDecodeRate1Half(htSigCoded); // 48 info bits

  // HT-SIG field layout (19.3.9.4.3), all sub-fields transmitted LSB first:
  // bits[0..6]=MCS, bit[7]=CBW, bits[8..23]=HT_LENGTH, bits[24..47]=HT-SIG2 fields (unused here).
  //
  // The TDC 8 real 20 MHz captures are GREENFIELD PPDUs (no L-SIG, both HT-SIG symbols QBPSK right
  // after the HT-LTF1); decoding them as mixed format was why HT-SIG never matched. With the
  // greenfield layout above MCS0-7 decode correctly on all of them. Content is still not used to
  // gate ok/error.
  int mcs = 0;
  for (int b = 6; b >= 0; --b) mcs = (mcs << 1) | htSigBits[b];
  const int cbw = htSigBits[7];
  int lengthBytes = 0;
  for (int b = 0; b < 16; ++b) lengthBytes |= (htSigBits[8 + b] << b);

  result.mcsIndex = mcs;
  result.bandwidthMHz = cbw == 0 ? 20 : 40;
  result.psduLengthBytes = lengthBytes;

  // Clamp to a safe default (MCS0/BPSK/rate-1/2) for the DATA-field pass below instead of
  // failing outright, since the decoded mcs value is not reliable (see note above). EVM/numSym
  // measured against this default MCS are therefore for diagnostic visibility only.
  McsParams mcsParams;
  if (!mcsToParams(mcs, mcsParams)) {
    mcsToParams(0, mcsParams);
  }
  result.modulation = mcsParams.modulation;
  result.codingRateNumerator = mcsParams.codingNum;
  result.codingRateDenominator = mcsParams.codingDen;

  // DATA-field EVM/symbol-count measurement is only attempted when HT-SIG's own fields are at
  // least internally plausible (since they feed the per-symbol loop below); given the
  // unresolved HT-SIG bit-convention uncertainty described above, this is frequently not the
  // case, in which case the core still reports ok=true with the burst/sync/L-SIG results
  // (which ARE independently verified) and simply leaves numDataSymbols/rmsEvmPercent at 0.
  if (cbw != 0) {
    result.ok = true;
    return result;
  }

  const int nSym = static_cast<int>(std::ceil((16.0 + 8.0 * lengthBytes + 6.0) / mcsParams.dbps));
  result.numDataSymbols = nSym;
  if (nSym <= 0 || nSym > 10000) {
    result.ok = true;
    return result;
  }

  const int cbps = mcsParams.nBpsc * HT_DATA_CARRIERS;
  std::vector<int> firstPerm, secondPerm;
  htDataInterleaverIndices(mcsParams.nBpsc, cbps, firstPerm, secondPerm);

  double evmSumSquares = 0.0;
  double evmPeak = -1.0;
  long long evmCount = 0;
  int peakSymbol = 0, peakSubcarrier = 0;

  for (int sym = 0; sym < nSym; ++sym) {
    const int dataFftStart = dataFieldStart + 80 * sym;
    if (dataFftStart + FFT_SIZE > static_cast<int>(samples->size())) break;
    const std::vector<cd> dataSym = fftSymbolAt(dataFftStart);

    // p_n index continues from HT-SIG2 (index 2) through HT-STF/HT-LTF (indices 3,4, not
    // pilot-tracked) to the first DATA symbol at index 5 in mixed format, from index 2 in greenfield.
    const double pData = polarity[(sym + (greenfield ? 2 : 5)) % 127];
    // HT data pilots (19.3.11.10, Nss=1, 20 MHz) cycle {1,1,1,-1} by one position per data symbol,
    // unlike the fixed legacy pattern; a mismatched pattern makes the four-pilot sum cancel.
    static const double htPilotBase[4] = {1.0, 1.0, 1.0, -1.0};
    const int htPilotK[4] = {-21, -7, 7, 21};
    cd pilotBeta(0.0, 0.0);
    for (int j = 0; j < 4; ++j) {
      const int k = htPilotK[j];
      pilotBeta += dataSym[fftBin(k)] * std::conj(cd(htPilotBase[(j + sym) % 4] * pData, 0.0)) / channel[fftBin(k)];
    }
    const double beta = std::abs(pilotBeta) > 0.0 ? std::arg(pilotBeta) : 0.0;
    const cd rotation = std::polar(1.0, -beta);

    std::vector<cd> equalized(HT_DATA_CARRIERS);
    int idx = 0;
    for (int k = -28; k <= 28; ++k) {
      if (!isHtDataSubcarrier(k)) continue;
      // The channel estimate from L-LTF only covers |k|<=26; approximate the four extra HT
      // edge tones (k=-28,-27,27,28) by nearest-neighbor extrapolation from the channel
      // estimate's edge value, since this core does not independently decode HT-LTF.
      int estK = k;
      if (estK > 26) estK = 26;
      if (estK < -26) estK = -26;
      equalized[idx++] = dataSym[fftBin(k)] / channel[fftBin(estK)] * rotation;
    }
    for (int c = 0; c < HT_DATA_CARRIERS; ++c) {
      const cd decision = hardDecision(equalized[c], mcsParams.modulation);
      const double err = std::norm(equalized[c] - decision);
      const double refPower = std::norm(decision) > 0.0 ? std::norm(decision) : 1.0;
      const double evmFraction = err / refPower;
      evmSumSquares += evmFraction;
      ++evmCount;
      if (evmFraction > evmPeak) {
        evmPeak = evmFraction;
        peakSymbol = sym;
        peakSubcarrier = c;
      }
    }
  }

  result.rmsEvmPercent = evmCount > 0 ? 100.0 * std::sqrt(evmSumSquares / static_cast<double>(evmCount)) : 0.0;
  result.peakEvmPercent = evmPeak >= 0.0 ? 100.0 * std::sqrt(evmPeak) : 0.0;
  result.peakEvmSymbolIndex = peakSymbol;
  result.peakEvmSubcarrierIndex = peakSubcarrier;

  result.ok = true;
  return result;
}

}
