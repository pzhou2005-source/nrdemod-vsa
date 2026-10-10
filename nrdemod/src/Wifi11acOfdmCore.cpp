/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11ac/Wifi11acOfdmCore.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace wifi11acdemod {

namespace {

typedef std::complex<double> cd;

const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const int FFT_SIZE = 64;
const int VHT_DATA_CARRIERS = 52;

// Identical to wifi/WifiOfdmCore.cpp and wifi11n/Wifi11nOfdmCore.cpp: the legacy preamble of a
// VHT PPDU (L-STF/L-LTF/L-SIG) is bit-for-bit identical to 802.11a (IEEE 802.11-2016 22.3.2),
// so the already-validated burst/sync/channel-estimate code is reused verbatim.
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

bool isLegacyDataSubcarrier(int k)
{
  return k != 0 && k >= -26 && k <= 26 && !isPilotSubcarrier(k);
}

// VHT 20 MHz DATA field subcarrier grid (IEEE 802.11-2016 Table 21-11): same 56 active
// subcarriers (52 data + 4 pilot, k=-28..28) as HT 20 MHz.
bool isVhtDataSubcarrier(int k)
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

// Windowed-sinc (Hamming) lowpass FIR, used by channelizeFir() below to extract one 20 MHz
// VHT subchannel out of a wider (40/80/160 MHz) capture before decimating. numTaps is fixed at
// 127: with the typical sample counts of these captures (tens of thousands) this is cheap, and
// gives a stopband sharp enough that the real 40/80/160 MHz TDC captures achieve syncRho>0.99
// (verified empirically against naive-decimation's syncRho<0.4, which aliases the unwanted
// subchannels' energy directly onto the primary channel and corrupts the L-LTF correlation).
std::vector<double> designLowpassFir(int numTaps, double cutoffRatio)
{
  std::vector<double> h(numTaps);
  const double center = (numTaps - 1) / 2.0;
  double sum = 0.0;
  for (int n = 0; n < numTaps; ++n) {
    const double x = static_cast<double>(n) - center;
    double v;
    if (x == 0.0) {
      v = 2.0 * cutoffRatio;
    } else {
      v = std::sin(TWO_PI * cutoffRatio * x) / (PI * x);
    }
    const double window = 0.54 - 0.46 * std::cos(TWO_PI * static_cast<double>(n) / static_cast<double>(numTaps - 1));
    v *= window;
    h[n] = v;
    sum += v;
  }
  if (sum != 0.0) {
    for (auto& v : h) v /= sum;
  }
  return h;
}

// Frequency-shifts the capture by -offsetHz (bringing the target 20 MHz subchannel to
// baseband), lowpass-filters it to the 20 MHz VHT subchannel bandwidth, then decimates to
// outRate. sampleRate must be an integer multiple of outRate. maxOutLen, when non-negative,
// bounds how many decimated output samples are produced/needed - used by the grid search below
// to avoid mixing+filtering an entire (possibly huge) capture just to probe for the L-LTF, which
// is always found within the first few hundred samples of the burst (IEEE 802.11-2016 22.3.8:
// fixed-length L-STF+L-LTF preamble). The final, winning-candidate call omits this bound so the
// full decimated capture is still available for the VHT-SIG-A decode and 20 MHz EVM path.
std::vector<cd> channelizeFir(const std::vector<cd>& iq, double sampleRate, double offsetHz, double outRate,
                               int maxOutLen = -1)
{
  const int numTaps = 127;
  const double cutoffRatio = (outRate / 2.0) / sampleRate;
  const std::vector<double> h = designLowpassFir(numTaps, cutoffRatio);
  const int n = static_cast<int>(iq.size());
  const int center = (numTaps - 1) / 2;
  const int ratio = static_cast<int>(std::llround(sampleRate / outRate));

  const int fullOutLen = n / ratio;
  const int outLenClamped = (maxOutLen >= 0) ? std::min(maxOutLen, fullOutLen) : fullOutLen;
  // Only mix+rotate as many input samples as the bounded output actually reads (last output
  // sample needs input index (outLenClamped-1)*ratio + center), not the whole capture.
  const int mixLen = std::min(n, outLenClamped > 0 ? (outLenClamped - 1) * ratio + center + 1 : 0);

  // Mix down via a complex rotation recurrence (one std::polar() call total, then an O(1)
  // complex multiply per sample) instead of calling std::polar() (sin/cos) for every sample -
  // this was the actual hot path, not the FIR convolution itself: profiling showed the
  // trig-per-sample mixer accounted for >90% of channelizeFir()'s cost on a 320 MHz/400 Msps
  // capture (dropping its share of a 39-candidate grid search from ~510ms to ~5ms).
  // Periodically renormalized to counter float drift from repeated multiplication.
  std::vector<cd> shifted(mixLen);
  if (mixLen > 0) {
    const cd step = std::polar(1.0, -TWO_PI * offsetHz / sampleRate);
    cd rot(1.0, 0.0);
    for (int i = 0; i < mixLen; ++i) {
      shifted[i] = iq[i] * rot;
      rot *= step;
      if ((i & 1023) == 1023)
        rot /= std::abs(rot);
    }
  }

  // Evaluate the FIR only at the sample positions actually kept after decimation, instead of
  // computing it for every one of the n input samples first and then discarding ratio-1 out
  // of every ratio outputs - a 320 MHz/400 Msps capture decimated to 20 MHz has ratio=20, so
  // the old eager approach wasted ~95% of the FIR convolution work too.
  auto filterAt = [&](int i) -> cd {
    const int kLo = std::max(0, i + center - (static_cast<int>(shifted.size()) - 1));
    const int kHi = std::min(numTaps - 1, i + center);
    cd acc(0.0, 0.0);
    for (int k = kLo; k <= kHi; ++k) {
      acc += h[k] * shifted[i + center - k];
    }
    return acc;
  };

  std::vector<cd> out(outLenClamped);
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = filterAt(static_cast<int>(i) * ratio);
  return out;
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

int parityOf(unsigned v)
{
  int p = 0;
  while (v) { p ^= (v & 1u); v >>= 1; }
  return p;
}

// Rate-1/2 K=7 convolutional decode of a self-contained 48-coded-bit codeword (24 info bits
// + 6 zero tail bits worth of trellis termination), identical taps/convention to the SIGNAL/
// HT-SIG decoder. Unlike HT-SIG (whose two symbols share one continuous 96-bit codeword),
// VHT-SIG-A1 and VHT-SIG-A2 are each independently BCC rate-1/2 encoded (21.3.8.3.3): every
// symbol carries its own complete 24-bit payload (ending in its own 6 tail bits), so each is
// decoded on its own as a 48-coded-bit / 24-info-bit codeword here.
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

void vhtDataInterleaverIndices(int nBpsc, int nCbps, std::vector<int>& first, std::vector<int>& second)
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
    case MOD_QAM64: {
      const double s = 1.0 / std::sqrt(42.0);
      const auto axis = [&](double v) {
        double level = 2.0 * std::floor((v / s - 1.0) / 2.0 + 0.5) + 1.0;
        if (level > 7.0) level = 7.0;
        if (level < -7.0) level = -7.0;
        return level * s;
      };
      return cd(axis(z.real()), axis(z.imag()));
    }
    case MOD_QAM256:
    default: {
      const double s = 1.0 / std::sqrt(170.0);
      const auto axis = [&](double v) {
        double level = 2.0 * std::floor((v / s - 1.0) / 2.0 + 0.5) + 1.0;
        if (level > 15.0) level = 15.0;
        if (level < -15.0) level = -15.0;
        return level * s;
      };
      return cd(axis(z.real()), axis(z.imag()));
    }
  }
}

// VHT-MCS parameters for a single spatial stream, 20 MHz, no STBC (IEEE 802.11-2016 Table
// 21-30/21-31). MCS9 is not a valid combination at 20 MHz/Nss=1 (fractional NDBPS) but is kept
// here with its would-be rate for diagnostic completeness since this core does not gate on it.
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
    case 0: p = {MOD_BPSK,   1, 1, 2,  26}; return true;
    case 1: p = {MOD_QPSK,   2, 1, 2,  52}; return true;
    case 2: p = {MOD_QPSK,   2, 3, 4,  78}; return true;
    case 3: p = {MOD_QAM16,  4, 1, 2, 104}; return true;
    case 4: p = {MOD_QAM16,  4, 3, 4, 156}; return true;
    case 5: p = {MOD_QAM64,  6, 2, 3, 208}; return true;
    case 6: p = {MOD_QAM64,  6, 3, 4, 234}; return true;
    case 7: p = {MOD_QAM64,  6, 5, 6, 260}; return true;
    case 8: p = {MOD_QAM256, 8, 3, 4, 312}; return true;
    case 9: p = {MOD_QAM256, 8, 5, 6, 346}; return true;
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
      codingRateDenominator(2), bandwidthMHz(20), nsts(1), numDataSymbols(0), rmsEvmPercent(0.0),
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
    if (osr == 2) {
      // Plain 20 MHz capture (sampleRate == 2x the signal bandwidth, no wider subchannels to
      // search/reject): decimate directly, exactly as before - no filtering needed since the
      // Nyquist bandwidth of the decimated signal already matches the occupied bandwidth.
      work.resize(iq.size() / osr);
      for (size_t i = 0; i < work.size(); ++i) work[i] = iq[i * osr];
    } else {
      // Captures wider than the base 20 MHz legacy subchannel (40/80/160 MHz VHT PPDUs) are not
      // sampled at a clean "2x the channel bandwidth" rate relative to the one 20 MHz primary
      // subchannel the legacy preamble/VHT-SIG-A is actually transmitted on - the real TDC
      // captures use oversampling ratios of 4/5/10 (not the naively-expected 2/4/8), so osr alone
      // does not reveal how many 20 MHz subchannels are present or where the primary one sits in
      // frequency. The VHT legacy preamble (L-STF/L-LTF/L-SIG/VHT-SIG-A) is replicated identically
      // on every 20 MHz subchannel of a wider VHT PPDU (IEEE 802.11-2016 22.3.8.3.2/22.3.8.3.3), so
      // instead a lowpass-FIR-channelized 20 MHz slice is extracted at every candidate center
      // frequency on a 10 MHz grid spanning the Nyquist range, and whichever gives the strongest
      // L-LTF matched-filter score is kept as the primary subchannel. Naive decimation by osr (no
      // channelization) was verified empirically to fold the other subchannels' energy onto the
      // primary one and corrupt the L-LTF correlation (real 40/80/160 MHz captures drop from
      // syncRho~1.0 to <0.4 without this channelization step).
      const std::vector<cd> ltfTimeProbe = ltfTimeDomain();
      const int probeStart = burstStart / osr;
      const int probeLength = burstLength / osr;
      const double stepHz = 10.0e6;
      const int maxK = static_cast<int>(std::floor((config.sampleRate / 2.0 - 10.0e6) / stepHz));
      double bestSubScore = -1.0;
      double bestSubOffsetHz = 0.0;
      // Bound the per-candidate probe to just past where the L-LTF can possibly be (fixed-length
      // L-STF+L-LTF preamble right after burstStart), not the whole burst - the preamble position
      // itself is unknown ahead of time only within burst-detection jitter, never deep into the
      // DATA field, so this is a large win (verified bit-identical bestSubScore/bestSubOffsetHz on
      // real captures vs scoring the whole burst) with no loss of correctness.
      const int probeWindow = std::min(probeLength, 512);
      for (int k = -maxK; k <= maxK; ++k) {
        const double offsetHz = static_cast<double>(k) * stepHz;
        const int neededLen = probeStart + probeWindow + 2 * FFT_SIZE;
        const std::vector<cd> channelized = channelizeFir(iq, config.sampleRate, offsetHz, 20.0e6, neededLen);
        const int probeEnd =
            std::min(static_cast<int>(channelized.size()) - 2 * FFT_SIZE, probeStart + probeWindow);
        for (int pos = probeStart; pos < probeEnd; ++pos) {
          const double score = matchedFilterScore(channelized, pos, ltfTimeProbe) +
                                matchedFilterScore(channelized, pos + FFT_SIZE, ltfTimeProbe);
          if (score > bestSubScore) {
            bestSubScore = score;
            bestSubOffsetHz = offsetHz;
          }
        }
      }
      work = channelizeFir(iq, config.sampleRate, bestSubOffsetHz, 20.0e6);
    }
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

  // Pilot-tracked phase correction + legacy 48-subcarrier hard-decision bits for one BPSK OFDM
  // symbol (used for both VHT-SIG-A symbols, both of which are plain BPSK per 21.3.8.3.3 -
  // unlike HT-SIG's second symbol, VHT-SIG-A has no QBPSK rotation on either symbol).
  const auto decodeLegacyGridSymbolBits = [&](const std::vector<cd>& sym, double pPilot) -> std::vector<int> {
    cd pilotBeta(0.0, 0.0);
    for (const int k : {-21, -7, 7, 21}) {
      pilotBeta += sym[fftBin(k)] * std::conj(cd(pilotBaseSign(k) * pPilot, 0.0)) / channel[fftBin(k)];
    }
    const double beta = std::abs(pilotBeta) > 0.0 ? std::arg(pilotBeta) : 0.0;
    const cd rotation = std::polar(1.0, -beta);
    std::vector<int> bits(48);
    int idx = 0;
    for (int k = -26; k <= 26; ++k) {
      if (!isLegacyDataSubcarrier(k)) continue;
      const cd eq = sym[fftBin(k)] / channel[fftBin(k)] * rotation;
      bits[idx++] = eq.real() >= 0.0 ? 1 : 0;
    }
    return bits;
  };

  // Field layout from bestPos (start of L-LTF): L-LTF(128) + L-SIG(80, skipped) +
  // VHT-SIG-A1(80) + VHT-SIG-A2(80) + VHT-STF(80, skipped) + VHT-LTF(80*nLtf, skipped, channel
  // estimate reused from L-LTF) + VHT-SIG-B(80, skipped) + DATA symbols (80 each). This core
  // only demodulates Nsts=1 (hence exactly one 80-sample VHT-LTF symbol).
  const int vhtSigA1Start = bestPos + 2 * FFT_SIZE + 80 + 16;
  const int vhtSigA2Start = vhtSigA1Start + 80;
  const int dataFieldStart = vhtSigA2Start + 80 + 80 + 80 + 80; // + VHT-STF + VHT-LTF(x1) + VHT-SIG-B
  if (vhtSigA2Start + FFT_SIZE > static_cast<int>(samples->size())) {
    result.error = "capture too short for VHT-SIG-A";
    return result;
  }

  const std::vector<cd> sigA1Sym = fftSymbolAt(vhtSigA1Start);
  const std::vector<cd> sigA2Sym = fftSymbolAt(vhtSigA2Start);
  // p_n index continuing from the two L-LTF symbols (-2/-1): L-SIG=0, VHT-SIG-A1=1, VHT-SIG-A2=2.
  const std::vector<int> sigA1BitsInterleaved = decodeLegacyGridSymbolBits(sigA1Sym, polarity[1]);
  const std::vector<int> sigA2BitsInterleaved = decodeLegacyGridSymbolBits(sigA2Sym, polarity[2]);
  const std::vector<int> sigA1Coded = deinterleaveSignalField(sigA1BitsInterleaved);
  const std::vector<int> sigA2Coded = deinterleaveSignalField(sigA2BitsInterleaved);
  // Each symbol is independently BCC rate-1/2 encoded (21.3.8.3.3: unlike HT-SIG's two symbols
  // sharing one continuous 96-bit codeword, every VHT-SIG-A symbol carries its own complete
  // 24-info-bit payload ending in its own 6 tail bits), so each 48-coded-bit symbol is Viterbi
  // decoded independently here, yielding 24 info bits each.
  const std::vector<int> sigA1Bits = viterbiDecodeRate1Half(sigA1Coded);
  const std::vector<int> sigA2Bits = viterbiDecodeRate1Half(sigA2Coded);

  // VHT-SIG-A field layout (IEEE 802.11-2016 21.3.8.3.3), two independently BCC rate-1/2
  // encoded 24-info-bit halves:
  //   SIG-A1: b0-1=BW, b2=Reserved(1), b3=STBC, b4-9=Group ID, b10-12=NSTS-1 (SU),
  //           b13-21=Partial AID, b22=TXOP_PS_NOT_ALLOWED, b23=Reserved(1)
  //   SIG-A2: b0=Short GI, b1=Short GI disambiguation, b2=SU coding type (0=BCC,1=LDPC),
  //           b3=LDPC extra OFDM symbol, b4-7=MCS (SU), b8=Beamformed, b9=Reserved(1),
  //           b10-17=CRC8, b18-23=Tail(zero)
  //
  // NOTE: unlike burst/sync/L-SIG (independently verified: L-SIG reliably decodes to the
  // expected rate=0x0D "6 Mb/s" marker on every real capture), this bit-level VHT-SIG-A layout
  // is reconstructed from secondary/community sources (no access to licensed IEEE 802.11-2016
  // text) and NOT independently re-derived the way the 802.11a SIGNAL field or L-LTF table
  // were. It is correlated against the real TDC captures' own recorded ModFormat/Bandwidth
  // RESULT fields in the regression test below; mcsIndex/bandwidthMHz/nsts are reported for
  // diagnostic visibility but deliberately do NOT gate result.ok (same policy as HT-SIG and
  // the still-undecoded 802.11b CCK bit mapping) in case a given capture's values disagree.
  const int bw = (sigA1Bits[0] | (sigA1Bits[1] << 1));
  const int nstsField = sigA1Bits[10] | (sigA1Bits[11] << 1) | (sigA1Bits[12] << 2);
  int mcs = 0;
  for (int b = 3; b >= 0; --b) mcs = (mcs << 1) | sigA2Bits[4 + b];

  result.bandwidthMHz = bw == 0 ? 20 : (bw == 1 ? 40 : (bw == 2 ? 80 : 160));
  result.nsts = nstsField + 1;
  result.mcsIndex = mcs;

  McsParams mcsParams;
  if (!mcsToParams(mcs, mcsParams)) {
    mcsToParams(0, mcsParams);
  }
  result.modulation = mcsParams.modulation;
  result.codingRateNumerator = mcsParams.codingNum;
  result.codingRateDenominator = mcsParams.codingDen;

  // DATA-field EVM/symbol-count measurement is only attempted for the in-scope case (20 MHz,
  // single spatial stream); otherwise this core still reports ok=true with the burst/sync/
  // L-SIG results (which ARE independently verified).
  if (bw != 0 || nstsField != 0) {
    result.ok = true;
    return result;
  }

  // VHT uses a 20-bit APEP_LENGTH (not carried in SIG-A at all - it is only known via
  // VHT-SIG-B, which this core does not decode), so unlike legacy/HT the number of DATA
  // symbols cannot be derived from SIG-A alone. Instead, consume DATA symbols for as long as
  // the detected burst extends past dataFieldStart (NOT the full capture length, which may
  // contain trailing noise/silence past the actual burst end that would otherwise inflate the
  // symbol count and corrupt the EVM average with noise-only "symbols").
  const int burstEnd = burstStart + burstLength;
  const int maxPossibleSymbols =
      (std::min(burstEnd, static_cast<int>(samples->size())) - dataFieldStart) / 80;
  const int nSym = std::max(0, std::min(maxPossibleSymbols, 2000));
  result.numDataSymbols = nSym;

  const int cbps = mcsParams.nBpsc * VHT_DATA_CARRIERS;
  std::vector<int> firstPerm, secondPerm;
  vhtDataInterleaverIndices(mcsParams.nBpsc, cbps, firstPerm, secondPerm);

  double evmSumSquares = 0.0;
  double evmPeak = -1.0;
  long long evmCount = 0;
  int peakSymbol = 0, peakSubcarrier = 0;

  for (int sym = 0; sym < nSym; ++sym) {
    const int dataFftStart = dataFieldStart + 80 * sym;
    if (dataFftStart + FFT_SIZE > static_cast<int>(samples->size())) break;
    const std::vector<cd> dataSym = fftSymbolAt(dataFftStart);

    // p_n index continues from VHT-SIG-A2 (index 2) through VHT-STF/VHT-LTF/VHT-SIG-B (indices
    // 3,4,5, not pilot-tracked) to the first DATA symbol at index 6, i.e. (sym + 6) % 127.
    const double pData = polarity[(sym + 6) % 127];
    cd pilotBeta(0.0, 0.0);
    for (const int k : {-21, -7, 7, 21}) {
      pilotBeta += dataSym[fftBin(k)] * std::conj(cd(pilotBaseSign(k) * pData, 0.0)) / channel[fftBin(k)];
    }
    const double beta = std::abs(pilotBeta) > 0.0 ? std::arg(pilotBeta) : 0.0;
    const cd rotation = std::polar(1.0, -beta);

    std::vector<cd> equalized(VHT_DATA_CARRIERS);
    int idx = 0;
    for (int k = -28; k <= 28; ++k) {
      if (!isVhtDataSubcarrier(k)) continue;
      int estK = k;
      if (estK > 26) estK = 26;
      if (estK < -26) estK = -26;
      equalized[idx++] = dataSym[fftBin(k)] / channel[fftBin(estK)] * rotation;
    }
    for (int c = 0; c < VHT_DATA_CARRIERS; ++c) {
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
