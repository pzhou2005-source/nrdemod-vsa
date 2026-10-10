/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11be/Wifi11beOfdmCore.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wifi11bedemod {

namespace {

typedef std::complex<double> cd;

const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const int FFT_SIZE = 64;

// Identical to wifi/WifiOfdmCore.cpp, wifi11n/Wifi11nOfdmCore.cpp, wifi11ac/Wifi11acOfdmCore.cpp
// and wifi11ax/Wifi11axOfdmCore.cpp: the legacy preamble of an EHT PPDU (L-STF/L-LTF/L-SIG) is
// bit-for-bit identical to 802.11a (IEEE 802.11-2024 36.3.4), so the already-validated
// burst/sync/channel-estimate code is reused verbatim.
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

// Windowed-sinc (Hamming) lowpass FIR, identical to wifi11ac/wifi11ax's channelizeFir()
// helper: used to extract one 20 MHz EHT subchannel out of a wider (40/80/160/320 MHz)
// capture before decimating/correlating against the legacy L-LTF.
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
  const double ratio = sampleRate / outRate;
  const size_t fullOutLen = static_cast<size_t>(std::floor(static_cast<double>(n - 1) / ratio)) + 1;
  const size_t outLen = (maxOutLen >= 0) ? std::min(static_cast<size_t>(maxOutLen), fullOutLen) : fullOutLen;
  // Only mix+rotate as many input samples as the bounded output actually reads (plus one extra
  // sample for linear interpolation and the FIR half-width), not the whole capture - see
  // wifi11ac/wifi11ax's channelizeFir() for why this bound exists (grid-search probe calls only
  // need the first few hundred decimated samples where the L-LTF can possibly be).
  const int mixLen =
      outLen > 0 ? std::min(n, static_cast<int>(std::ceil(static_cast<double>(outLen) * ratio)) + center + 2) : 0;

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

  // Evaluate the FIR only at the sample positions actually needed for the output (plus the
  // one extra sample linear interpolation reads), instead of computing it for every one of
  // the n input samples first and then discarding most of them.
  auto filterAt = [&](int i) -> cd {
    if (i < 0 || i >= mixLen)
      return cd(0.0, 0.0);
    const int kLo = std::max(0, i + center - (mixLen - 1));
    const int kHi = std::min(numTaps - 1, i + center);
    cd acc(0.0, 0.0);
    for (int k = kLo; k <= kHi; ++k) {
      acc += h[k] * shifted[i + center - k];
    }
    return acc;
  };

  // Resample at the (possibly non-integer, e.g. 50 Msps/20 MHz=2.5) ratio via linear
  // interpolation between adjacent filtered samples; this is numerically identical to the
  // old exact-decimation code whenever the ratio IS an integer (fractional part is exactly
  // 0 so the interpolation degenerates to picking filtered[i*ratio] directly), so no
  // regression for the already-verified 2/4/5/10/20x cases, while now also supporting the
  // real 802.11be TDC captures that use a 1.25x-of-channel-bandwidth sample clock (e.g.
  // 50 Msps for a 40 MHz EHT PPDU).
  std::vector<cd> out(outLen);
  for (size_t i = 0; i < out.size(); ++i) {
    const double pos = static_cast<double>(i) * ratio;
    const size_t lo = static_cast<size_t>(pos);
    const double frac = pos - static_cast<double>(lo);
    if (frac == 0.0 || lo + 1 >= static_cast<size_t>(n)) {
      out[i] = filterAt(static_cast<int>(lo));
    } else {
      out[i] = filterAt(static_cast<int>(lo)) * (1.0 - frac) + filterAt(static_cast<int>(lo) + 1) * frac;
    }
  }
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
  if (samplesPerFftSample < 0.99) {
    result.error = "sample rate must be at least 20 MHz";
    return result;
  }
  const bool integerOsr = std::fabs(samplesPerFftSample - std::llround(samplesPerFftSample)) <= 1e-3;
  const int osr = integerOsr ? static_cast<int>(std::llround(samplesPerFftSample)) : 0;
  std::vector<cd> work;
  const std::vector<cd>* samples = &iq;
  if (osr != 1) {
    if (osr == 2) {
      // Plain 20 MHz capture: decimate directly, no filtering needed since the Nyquist
      // bandwidth of the decimated signal already matches the occupied bandwidth.
      work.resize(iq.size() / osr);
      for (size_t i = 0; i < work.size(); ++i) work[i] = iq[i * osr];
    } else {
      // Wider captures (40/80/160/320 MHz EHT PPDUs): same subchannel-search strategy as
      // wifi11ac/wifi11ax - the sample rate is not a clean "2x channel bandwidth" ratio
      // relative to the 20 MHz primary subchannel the legacy preamble is transmitted on (and
      // may not even be an integer multiple of 20 MHz at all - some real 40 MHz EHT TDC
      // captures use a 50 Msps clock, i.e. 2.5x - channelizeFir() resamples via linear
      // interpolation to handle this), so search every candidate 20 MHz center frequency on
      // a 10 MHz grid spanning the Nyquist range and keep whichever gives the strongest
      // L-LTF match. Verified against real 320 MHz/400 Msps TDC captures (osr=20): still
      // locates the correct subchannel.
      const std::vector<cd> ltfTimeProbe = ltfTimeDomain();
      const int probeStart = static_cast<int>(static_cast<double>(burstStart) / samplesPerFftSample);
      const int probeLength = static_cast<int>(static_cast<double>(burstLength) / samplesPerFftSample);
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
    burstStart = static_cast<int>(static_cast<double>(burstStart) / samplesPerFftSample);
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
