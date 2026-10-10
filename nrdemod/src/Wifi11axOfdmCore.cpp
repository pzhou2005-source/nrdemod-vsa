/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11ax/Wifi11axOfdmCore.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wifi11axdemod {

namespace {

typedef std::complex<double> cd;

const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const int FFT_SIZE = 64;

// Identical to wifi/WifiOfdmCore.cpp, wifi11n/Wifi11nOfdmCore.cpp and
// wifi11ac/Wifi11acOfdmCore.cpp: the legacy preamble of an HE PPDU
// (L-STF/L-LTF/L-SIG) is bit-for-bit identical to 802.11a (IEEE 802.11-2021
// 27.3.4), so the already-validated burst/sync/channel-estimate code is
// reused verbatim.
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

// Windowed-sinc (Hamming) lowpass FIR, identical to wifi11ac/Wifi11acOfdmCore.cpp's
// channelizeFir() helper: used to extract one 20 MHz HE subchannel out of a wider
// (40/80/160 MHz) capture before decimating/correlating against the legacy L-LTF.
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
  // of every ratio outputs - a 160 MHz/200 Msps capture decimated to 20 MHz has ratio=10, so
  // the old eager approach wasted ~90% of the FIR convolution work too.
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

}

Config::Config() : sampleRate(0.0), burstSearch(true), burstSearchThresholdDb(15.0), syncThreshold(0.5)
{
}

Result::Result()
    : ok(false), burstFound(false), burstStart(0), burstLength(0), syncFound(false), syncRho(0.0),
      timingOffsetSamples(0), frequencyErrorHz(0.0)
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
      // Plain 20 MHz capture: decimate directly, no filtering needed since the Nyquist
      // bandwidth of the decimated signal already matches the occupied bandwidth.
      work.resize(iq.size() / osr);
      for (size_t i = 0; i < work.size(); ++i) work[i] = iq[i * osr];
    } else {
      // Wider captures (40/80/160 MHz HE PPDUs): same subchannel-search strategy as
      // wifi11ac/Wifi11acOfdmCore.cpp - the sample rate is not a clean "2x channel
      // bandwidth" ratio relative to the 20 MHz primary subchannel the legacy preamble is
      // transmitted on, so search every candidate 20 MHz center frequency on a 10 MHz grid
      // spanning the Nyquist range and keep whichever gives the strongest L-LTF match.
      const std::vector<cd> ltfTimeProbe = ltfTimeDomain();
      const int probeStart = burstStart / osr;
      const int probeLength = burstLength / osr;
      const double stepHz = 10.0e6;
      const int maxK = static_cast<int>(std::floor((config.sampleRate / 2.0 - 10.0e6) / stepHz));
      double bestSubScore = -1.0;
      double bestSubOffsetHz = 0.0;
      // Bound the per-candidate probe to just past where the L-LTF can possibly be (fixed-length
      // L-STF+L-LTF preamble right after burstStart), not the whole burst - verified bit-identical
      // bestSubScore/bestSubOffsetHz on real captures vs scoring the whole burst.
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

  result.ok = true;
  return result;
}

}
