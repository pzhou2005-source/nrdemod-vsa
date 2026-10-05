/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "nr/NrOfdmCore.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <sstream>

namespace nrdemod {

namespace {

typedef std::complex<double> cd;

const double PI = 3.14159265358979323846;
const double SCS_BASE_HZ = 15000.0;
const int MIN_FFT = 128;
const int MAX_FFT = 8192;

// External FFT provider (set via setFftProvider). NULL = use built-in.
FftProviderFn sExternalFft = NULL;


bool isPowerOfTwo(int n)
{
  return n > 0 && (n & (n - 1)) == 0;
}

int nextPowerOfTwo(double v)
{
  int n = MIN_FFT;
  while (n < v && n < MAX_FFT) n <<= 1;
  return n;
}

double qamScale(Modulation mod)
{
  switch (mod) {
    case MOD_QAM16:  return 1.0 / std::sqrt(10.0);
    case MOD_QAM64:  return 1.0 / std::sqrt(42.0);
    case MOD_QAM256: return 1.0 / std::sqrt(170.0);
    case MOD_QPSK:
    default:         return 1.0 / std::sqrt(2.0);
  }
}

int qamMaxLevel(Modulation mod)
{
  switch (mod) {
    case MOD_QAM16:  return 3;
    case MOD_QAM64:  return 7;
    case MOD_QAM256: return 15;
    case MOD_QPSK:
    default:         return 1;
  }
}

double decideAxis(double v, int maxLevel)
{
  double level = 2.0 * std::floor((v - 1.0) / 2.0 + 0.5) + 1.0;
  if (level > maxLevel) level = maxLevel;
  if (level < -maxLevel) level = -maxLevel;
  return level;
}

// |sum x[k] conj(x[k+N])| normalised by the window energy, 0..1
double cpMetric(const std::vector<cd>& x, size_t pos, int cp, int fftSize, cd* rawCorr)
{
  cd corr(0.0, 0.0);
  double energy = 0.0;
  for (int k = 0; k < cp; ++k) {
    const cd& a = x[pos + k];
    const cd& b = x[pos + k + fftSize];
    corr += a * std::conj(b);
    energy += 0.5 * (std::norm(a) + std::norm(b));
  }
  if (rawCorr) {
    *rawCorr = corr;
  }
  return std::abs(corr) / (energy + 1e-30);
}

struct SymbolLayout {
  size_t cpStart;
  int    cpLen;
};

std::vector<SymbolLayout> layoutSymbols(size_t numSamples, size_t start, int fftSize,
    int cpNormal, int cpLong, int period, int longOffset, int maxSymbols)
{
  std::vector<SymbolLayout> out;
  size_t pos = start;
  for (int j = 0;; ++j) {
    const int cp = (period > 0 && (j % period) == longOffset) ? cpLong : cpNormal;
    if (pos + static_cast<size_t>(cp + fftSize) > numSamples) break;
    SymbolLayout s;
    s.cpStart = pos;
    s.cpLen = cp;
    out.push_back(s);
    pos += static_cast<size_t>(cp + fftSize);
    if (maxSymbols > 0 && static_cast<int>(out.size()) >= maxSymbols) break;
  }
  return out;
}

struct Grid {
  int    fftSize;
  double gridRate;
  int    carrierRb;
  int    numRb;
  int    firstSubcarrier; // k0
  bool   resampled;
};

bool resolveGrid(const Config& cfg, Grid& g, std::string& error)
{
  const double scs = SCS_BASE_HZ * static_cast<double>(1 << cfg.numerology);

  g.carrierRb = 0;
  if (cfg.channelBandwidthMHz > 0) {
    g.carrierRb = carrierResourceBlocks(cfg.frequencyRange, cfg.channelBandwidthMHz, cfg.numerology);
    if (g.carrierRb <= 0) {
      std::ostringstream os;
      os << "Channel bandwidth " << cfg.channelBandwidthMHz << " MHz is not defined for FR"
         << cfg.frequencyRange << " with numerology " << cfg.numerology << ".";
      error = os.str();
      return false;
    }
  }
  if (cfg.resourceBlockOffset < 0) {
    error = "ResourceBlockOffset must not be negative.";
    return false;
  }
  if (g.carrierRb == 0 && cfg.resourceBlockOffset > 0) {
    error = "ResourceBlockOffset requires the channel bandwidth to be set.";
    return false;
  }

  g.numRb = cfg.numResourceBlocks;
  if (g.numRb <= 0 && g.carrierRb > 0) {
    g.numRb = g.carrierRb - cfg.resourceBlockOffset;
  }

  const double ratio = cfg.sampleRate / scs;
  const bool onGrid = std::fabs(ratio - std::floor(ratio + 0.5)) < 1e-6 * ratio &&
                      isPowerOfTwo(static_cast<int>(std::floor(ratio + 0.5)));
  if (cfg.fftSize > 0) {
    g.fftSize = cfg.fftSize;
  } else if (onGrid) {
    g.fftSize = static_cast<int>(std::floor(ratio + 0.5));
  } else {
    const int needed = 12 * (g.carrierRb > 0 ? g.carrierRb : g.numRb);
    // known allocation: smallest grid that holds it; unknown: never downsample
    g.fftSize = needed > 0 ? nextPowerOfTwo(needed / 0.9) : nextPowerOfTwo(ratio);
  }
  if (!isPowerOfTwo(g.fftSize) || g.fftSize < MIN_FFT || g.fftSize > MAX_FFT) {
    std::ostringstream os;
    os << "FFT size " << g.fftSize << " must be a power of two between " << MIN_FFT << " and " << MAX_FFT << ".";
    error = os.str();
    return false;
  }
  g.gridRate = g.fftSize * scs;
  g.resampled = std::fabs(cfg.sampleRate - g.gridRate) > 1e-6 * cfg.sampleRate;
  if (g.resampled && !cfg.resample) {
    std::ostringstream os;
    os << "Sample rate " << cfg.sampleRate << " Hz must equal FftSize * SCS = " << g.gridRate
       << " Hz (resampling disabled).";
    error = os.str();
    return false;
  }

  if (g.numRb <= 0) {
    g.numRb = maxResourceBlocks(g.fftSize);
  }
  const int numSc = 12 * g.numRb;
  if (g.carrierRb > 0) {
    if (12 * g.carrierRb > g.fftSize) {
      std::ostringstream os;
      os << "Carrier with " << g.carrierRb << " resource blocks does not fit into FFT size " << g.fftSize << ".";
      error = os.str();
      return false;
    }
    if (cfg.resourceBlockOffset + g.numRb > g.carrierRb) {
      std::ostringstream os;
      os << "ResourceBlockOffset " << cfg.resourceBlockOffset << " + NumResourceBlocks " << g.numRb
         << " exceeds the carrier size of " << g.carrierRb << " resource blocks.";
      error = os.str();
      return false;
    }
    g.firstSubcarrier = -6 * g.carrierRb + 12 * cfg.resourceBlockOffset;
  } else {
    if (numSc > g.fftSize) {
      std::ostringstream os;
      os << "NumResourceBlocks " << g.numRb << " does not fit into FFT size " << g.fftSize << ".";
      error = os.str();
      return false;
    }
    g.firstSubcarrier = -numSc / 2;
  }
  return true;
}

struct EqualizedGrid {
  std::vector<std::vector<cd> > z;      // [symbol][point] equalized (time domain for DFT-s-OFDM)
  std::vector<std::vector<cd> > ref;    // hard decisions
  std::vector<cd>               channel;
  std::vector<double>           cpe;    // common phase error removed per symbol, radians
};

// Decision free channel start value: magnitude from the average subcarrier power
// (unit power constellation), phase from the 4th power statistics of square QAM
// (E[X^4] < 0 real) unwrapped along the subcarriers. Both are smoothed over
// neighbouring subcarriers, the channel (filters, timing) is smooth in frequency.
std::vector<cd> blindChannelEstimate(const std::vector<std::vector<cd>>& y, const std::vector<bool>& excludedSym,
    const std::vector<bool>& excludedSc, bool usePhase, const std::vector<std::vector<bool>>& resourceMask,
    int phaseWindowOverride)
{
  const size_t numSym = y.size();
  const size_t numSc = numSym ? y[0].size() : 0;
  // the 4th power statistic of dense QAM is noisy (E|X|^8 >> |E X^4|^2): average over
  // many neighbouring subcarriers, the channel is smooth over this span anyway
  const int window = 20;
  std::vector<double> power(numSc, 0.0);
  std::vector<cd> fourth(numSc, cd(0.0, 0.0));
  std::vector<size_t> samples(numSc, 0);
  size_t used = 0;
  for (size_t l = 0; l < numSym; ++l) {
    if (excludedSym[l]) continue;
    ++used;
    for (size_t i = 0; i < numSc; ++i) {
      if (resourceMask[l][i])
        continue;
      ++samples[i];
      const cd v = y[l][i];
      power[i] += std::norm(v);
      const cd v2 = v * v;
      fourth[i] += v2 * v2;
    }
  }
  std::vector<cd> h(numSc, cd(1.0, 0.0));
  if (!used) return h;

  std::vector<double> mag(numSc, 1.0);
  std::vector<double> phi4(numSc, 0.0);
  for (int i = 0; i < static_cast<int>(numSc); ++i) {
    double p = 0.0;
    cd f(0.0, 0.0);
    int count = 0;
    for (int k = std::max(0, i - window); k <= std::min(static_cast<int>(numSc) - 1, i + window); ++k) {
      if (excludedSc[k]) continue;
      p += power[k];
      count += samples[k];
    }
    if (count > 0 && p > 0.0) {
      mag[i] = std::sqrt(p / static_cast<double>(count));
    }
    const int phaseWindow = phaseWindowOverride > 0 ? phaseWindowOverride : (numSc >= 600 && used <= 13 ? 64 : window);
    for (int carrier = std::max(0, i - phaseWindow); carrier <= std::min(static_cast<int>(numSc) - 1, i + phaseWindow);
        ++carrier) {
      if (!excludedSc[carrier])
        f += fourth[carrier];
    }
    phi4[i] = std::abs(f) > 0.0 ? std::arg(-f) : 0.0;
  }
  for (size_t i = 1; i < numSc; ++i) {
    double d = phi4[i] - phi4[i - 1];
    while (d > PI) d -= 2.0 * PI;
    while (d < -PI) d += 2.0 * PI;
    phi4[i] = phi4[i - 1] + d;
  }
  for (size_t i = 0; i < numSc; ++i) {
    h[i] = std::polar(mag[i], usePhase ? phi4[i] / 4.0 : 0.0);
  }
  return h;
}

EqualizedGrid equalizeAndDecide(const std::vector<std::vector<cd>>& y, const std::vector<bool>& excludedSym,
    const std::vector<bool>& excludedSc, Modulation mod, bool equalize, bool phaseTracking, bool timingTracking,
    bool amplitudeTracking, bool precoding, int iterations, int puncturedCarrier,
    const std::vector<std::vector<bool>>& resourceMask, int phaseWindowOverride = 0,
    const std::vector<cd>* phaseReference = nullptr)
{
  const size_t numSym = y.size();
  const size_t numSc = numSym ? y[0].size() : 0;
  const double dftScale = numSc ? 1.0 / std::sqrt(static_cast<double>(numSc)) : 1.0;

  EqualizedGrid g;
  std::vector<bool> excludedFrequency(excludedSc);
  if (puncturedCarrier >= 0)
    excludedFrequency[puncturedCarrier] = true;
  g.channel =
      equalize ? blindChannelEstimate(y, excludedSym, excludedFrequency, !precoding, resourceMask, phaseWindowOverride)
               : std::vector<cd>(numSc, cd(1.0, 0.0));
  if (equalize && phaseReference) {
    cd cross(0.0, 0.0);
    double measuredPower = 0.0, estimatedPower = 0.0;
    for (size_t carrier = 0; carrier < numSc; ++carrier) {
      cross += g.channel[carrier] * std::conj((*phaseReference)[carrier]);
      if (std::abs((*phaseReference)[carrier]) > 0.0) {
        measuredPower += std::norm((*phaseReference)[carrier]);
        estimatedPower += std::norm(g.channel[carrier]);
      }
    }
    const double scale = measuredPower > 0.0 ? std::sqrt(estimatedPower / measuredPower) : 1.0;
    if (measuredPower > 0.0 && iterations > 1)
      iterations = std::max(iterations, 12);
    const cd ambiguity = std::polar(1.0, std::round(std::arg(cross) / (PI / 2.0)) * PI / 2.0);
    for (size_t carrier = 0; carrier < numSc; ++carrier)
      if (std::abs((*phaseReference)[carrier]) > 0.0)
        g.channel[carrier] = scale * ambiguity * (*phaseReference)[carrier];
  }
  if (precoding && equalize && numSc > 0) {
    const double centre = 0.5 * static_cast<double>(numSc - 1);
    auto timingError = [&](double delay) {
      double error = 0.0;
      size_t used = 0;
      for (size_t symbol = 0; symbol < numSym && used < 2; ++symbol) {
        if (excludedSym[symbol])
          continue;
        std::vector<cd> data(numSc);
        for (size_t carrier = 0; carrier < numSc; ++carrier) {
          data[carrier] =
              excludedSc[carrier]
                  ? cd(0.0, 0.0)
                  : y[symbol][carrier] / g.channel[carrier] *
                        std::polar(1.0, -2.0 * PI * delay * (static_cast<double>(carrier) - centre) / numSc);
        }
        dft(data, true);
        cd fourth(0.0, 0.0);
        for (size_t point = 0; point < numSc; ++point) {
          data[point] *= dftScale;
          const cd squared = data[point] * data[point];
          fourth += squared * squared;
        }
        cd rotation =
            phaseTracking && std::abs(fourth) > 0.0 ? std::polar(1.0, -(std::arg(fourth) - PI) / 4.0) : cd(1.0, 0.0);
        cd cross(0.0, 0.0);
        for (size_t point = 0; point < numSc; ++point) {
          const cd measured = data[point] * rotation;
          cross += measured * std::conj(hardDecision(measured, mod));
        }
        if (phaseTracking && std::abs(cross) > 0.0)
          rotation *= std::polar(1.0, -std::arg(cross));
        for (size_t point = 0; point < numSc; ++point) {
          const cd measured = data[point] * rotation;
          error += std::norm(measured - hardDecision(measured, mod));
        }
        ++used;
      }
      return error;
    };
    double bestDelay = 0.0, bestError = timingError(0.0);
    const double step = 1.0 / 32.0;
    for (int candidate = -16; candidate <= 16; ++candidate) {
      const double delay = candidate * step;
      const double error = timingError(delay);
      if (error < bestError) {
        bestError = error;
        bestDelay = delay;
      }
    }
    double lower = bestDelay - step, upper = bestDelay + step;
    for (int refinement = 0; refinement < 12; ++refinement) {
      const double first = (2.0 * lower + upper) / 3.0;
      const double second = (lower + 2.0 * upper) / 3.0;
      if (timingError(first) < timingError(second))
        upper = second;
      else
        lower = first;
    }
    bestDelay = 0.5 * (lower + upper);
    for (size_t carrier = 0; carrier < numSc; ++carrier)
      g.channel[carrier] *= std::polar(1.0, 2.0 * PI * bestDelay * (static_cast<double>(carrier) - centre) / numSc);
  }
  g.z.assign(numSym, std::vector<cd>(numSc));
  g.ref.assign(numSym, std::vector<cd>(numSc));
  std::vector<std::vector<cd> > refFreq(numSym, std::vector<cd>(numSc));
  // per symbol correction applied on top of the channel: common phase, timing ramp, amplitude
  std::vector<std::vector<cd> > corr(numSym, std::vector<cd>(numSc, cd(1.0, 0.0)));
  std::vector<double> slopes(numSym, 0.0), gains(numSym, 1.0);
  std::vector<std::vector<cd> > resid(numSym, std::vector<cd>(numSc));

  if (iterations < 1) iterations = 1;
  if (puncturedCarrier >= 0 && equalize && iterations > 1)
    iterations = std::max(iterations, 12);

  for (int it = 0; it < iterations; ++it) {
    for (size_t l = 0; l < numSym; ++l) {
      std::vector<cd>& d = g.z[l];
      for (size_t i = 0; i < numSc; ++i) {
        d[i] = y[l][i] / g.channel[i];
      }
      if (precoding) {
        if (puncturedCarrier >= 0 && it)
          d[puncturedCarrier] = refFreq[l][puncturedCarrier] / corr[l][puncturedCarrier];
        dft(d, true);
        for (size_t i = 0; i < numSc; ++i) d[i] *= dftScale;
      }
      std::vector<cd>& c = corr[l];
      std::fill(c.begin(), c.end(), cd(1.0, 0.0));

      const auto initialPunctureOffset = [&](cd rotation) {
        if (!precoding || mod != MOD_QAM256 || numSc >= 256 || puncturedCarrier < 0 ||
            2 * puncturedCarrier != static_cast<int>(numSc) || it != 0 || excludedSym[l])
          return cd(0.0, 0.0);
        double minimumReal[2] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        double maximumReal[2] = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        double minimumImag[2] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        double maximumImag[2] = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        for (size_t point = 0; point < numSc; ++point) {
          const cd measured = d[point] * rotation;
          const size_t parity = point % 2;
          minimumReal[parity] = std::min(minimumReal[parity], measured.real());
          maximumReal[parity] = std::max(maximumReal[parity], measured.real());
          minimumImag[parity] = std::min(minimumImag[parity], measured.imag());
          maximumImag[parity] = std::max(maximumImag[parity], measured.imag());
        }
        return cd(0.25 * (minimumReal[0] + maximumReal[0] - minimumReal[1] - maximumReal[1]),
            0.25 * (minimumImag[0] + maximumImag[0] - minimumImag[1] - maximumImag[1]));
      };

      if (phaseTracking) {
        // blind common phase estimate (4th power, square QAM has E[z^4] < 0) so that
        // the following hard decisions are reliable for dense constellations
        cd fourth(0.0, 0.0);
        for (size_t i = 0; i < numSc; ++i) {
          if (excludedSc[i] || resourceMask[l][i])
            continue;
          const cd z2 = d[i] * d[i];
          fourth += z2 * z2;
        }
        double phi = std::abs(fourth) > 0.0 ? (std::arg(fourth) - PI) / 4.0 : 0.0;
        if ((numSc < 64 && mod != MOD_QPSK) || (precoding && mod == MOD_QAM256 && !excludedSym[l])) {
          double bestError = std::numeric_limits<double>::infinity();
          for (int candidate = 0; candidate < 32; ++candidate) {
            const double phase = -PI / 4.0 + candidate * PI / 64.0;
            const cd rotation = std::polar(1.0, -phase);
            const cd offset = initialPunctureOffset(rotation);
            double error = 0.0;
            for (size_t carrier = 0; carrier < numSc; ++carrier) {
              if (excludedSc[carrier] || resourceMask[l][carrier])
                continue;
              const cd measured = d[carrier] * rotation - (carrier % 2 == 0 ? offset : -offset);
              error += std::norm(measured - hardDecision(measured, mod));
            }
            if (error < bestError) {
              bestError = error;
              phi = phase;
            }
          }
        }
        cd rot = std::polar(1.0, -phi);
        const cd offset = initialPunctureOffset(rot) / rot;
        for (size_t point = 0; point < numSc; ++point)
          d[point] -= point % 2 == 0 ? offset : -offset;

        for (int refinement = 0; refinement < 8; ++refinement) {
          cd acc(0.0, 0.0);
          for (size_t i = 0; i < numSc; ++i) {
            if (excludedSc[i] || resourceMask[l][i])
              continue;
            const cd z = d[i] * rot;
            acc += z * std::conj(hardDecision(z, mod));
          }
          if (std::abs(acc) == 0.0)
            break;
          const double correction = std::arg(acc);
          phi += correction;
          rot = std::polar(1.0, -phi);
          if (std::fabs(correction) < 1e-9)
            break;
        }
        for (size_t i = 0; i < numSc; ++i) { d[i] *= rot; c[i] *= rot; }
      }

      if (timingTracking || amplitudeTracking) {
        // decision directed residual per subcarrier: e_i = d_i conj(ref_i)
        double num = 0.0, den = 0.0;
        std::vector<cd> measuredFrequency, referenceFrequency;
        if (precoding) {
          measuredFrequency = d;
          referenceFrequency.resize(numSc);
          for (size_t carrier = 0; carrier < numSc; ++carrier)
            referenceFrequency[carrier] = hardDecision(d[carrier], mod);
          dft(measuredFrequency, false);
          dft(referenceFrequency, false);
          for (size_t carrier = 0; carrier < numSc; ++carrier) {
            measuredFrequency[carrier] *= dftScale;
            referenceFrequency[carrier] *= dftScale;
          }
        }
        for (size_t i = 0; i < numSc; ++i) {
          const cd ref = precoding ? referenceFrequency[i] : hardDecision(d[i], mod);
          const cd measured = precoding ? measuredFrequency[i] : d[i];
          resid[l][i] = excludedFrequency[i] || resourceMask[l][i] ? cd(0.0, 0.0) : measured * std::conj(ref);
          if (excludedFrequency[i] || resourceMask[l][i])
            continue;
          num += resid[l][i].real();
          den += std::norm(ref);
        }
        gains[l] = (amplitudeTracking && !excludedSym[l] && den > 0.0 && num > 0.0) ? num / den : 1.0;
      }

      for (size_t i = 0; i < numSc; ++i) {
        g.ref[l][i] = hardDecision(d[i], mod);
      }
    }

    if (timingTracking || amplitudeTracking) {
      // Per symbol effects (timing drift, gain variation) are measured relative to the
      // symbol average: the residual common to all symbols is a channel error and is
      // left to the equalizer, otherwise decision noise would be fed back into it.
      std::vector<cd> meanResid(numSc, cd(0.0, 0.0));
      double meanLogGain = 0.0;
      size_t count = 0;
      for (size_t l = 0; l < numSym; ++l) {
        if (excludedSym[l]) continue;
        for (size_t i = 0; i < numSc; ++i) meanResid[i] += resid[l][i];
        meanLogGain += std::log(gains[l]);
        ++count;
      }
      if (count) meanLogGain /= count;
      for (size_t i = 0; i < numSc; ++i) {
        const double m = std::abs(meanResid[i]);
        meanResid[i] = m > 0.0 ? meanResid[i] / m : cd(1.0, 0.0);
      }
      const double meanGain = std::exp(meanLogGain);
      for (size_t l = 0; l < numSym; ++l) {
        slopes[l] = 0.0;
        if (!timingTracking || excludedSym[l]) continue; // excluded symbols carry no decidable data
        // weighted least squares line through the residual phase (small after the CPE
        // step), differencing adjacent subcarriers would be far too noisy
        const double centre = 0.5 * static_cast<double>(numSc - 1);
        double sxy = 0.0, sxx = 0.0, sumWeight = 0.0, sumPosition = 0.0, sumPhase = 0.0;
        for (size_t i = 0; i < numSc; ++i) {
          if (excludedFrequency[i] || resourceMask[l][i])
            continue;
          const cd r = resid[l][i] * std::conj(meanResid[i]);
          const double w = std::abs(r);
          if (w <= 0.0) continue;
          const double x = static_cast<double>(i) - centre;
          sxy += w * x * std::arg(r);
          sxx += w * x * x;
          sumWeight += w;
          sumPosition += w * x;
          sumPhase += w * std::arg(r);
        }
        const double variance = sumWeight > 0.0 ? sxx - sumPosition * sumPosition / sumWeight : 0.0;
        if (variance > 0.0)
          slopes[l] = (sxy - sumPosition * sumPhase / sumWeight) / variance;
      }
      const double centre = 0.5 * static_cast<double>(numSc - 1);
      for (size_t l = 0; l < numSym; ++l) {
        std::vector<cd>& d = g.z[l];
        std::vector<cd>& c = corr[l];
        const double slope = slopes[l];
        const double gain = excludedSym[l] ? 1.0 : meanGain / gains[l];
        if (precoding) {
          dft(d, false);
          for (size_t carrier = 0; carrier < numSc; ++carrier)
            d[carrier] *= dftScale;
        }
        for (size_t i = 0; i < numSc; ++i) {
          const cd f = std::polar(gain, -slope * (static_cast<double>(i) - centre));
          d[i] *= f;
          c[i] *= f;
        }
        if (precoding) {
          dft(d, true);
          for (size_t carrier = 0; carrier < numSc; ++carrier)
            d[carrier] *= dftScale;
        }
        // re-estimate the common phase after removing the ramp
        cd acc(0.0, 0.0);
        for (size_t i = 0; i < numSc && phaseTracking; ++i) {
          if (excludedSc[i] || resourceMask[l][i])
            continue;
          acc += d[i] * std::conj(hardDecision(d[i], mod));
        }
        if (std::abs(acc) > 0.0) {
          const cd rot = std::polar(1.0, -std::arg(acc));
          for (size_t i = 0; i < numSc; ++i) { d[i] *= rot; c[i] *= rot; }
        }
        for (size_t i = 0; i < numSc; ++i) {
          g.ref[l][i] = hardDecision(d[i], mod);
        }
      }
    }

    for (size_t l = 0; l < numSym; ++l) {
      if (precoding) {
        refFreq[l] = g.ref[l];
        dft(refFreq[l], false);
        for (size_t i = 0; i < numSc; ++i) refFreq[l][i] *= dftScale;
      } else {
        refFreq[l] = g.ref[l];
      }
    }

    if (!equalize || it == iterations - 1) {
      break;
    }

    for (size_t i = 0; i < numSc; ++i) {
      if (excludedFrequency[i])
        continue;
      cd num(0.0, 0.0);
      double den = 0.0;
      for (size_t l = 0; l < numSym; ++l) {
        if (excludedSym[l] || resourceMask[l][i])
          continue;
        num += y[l][i] * corr[l][i] * std::conj(refFreq[l][i]);
        den += std::norm(refFreq[l][i]);
      }
      if (!precoding && mod == MOD_QAM256 && it == 1) {
        auto gainError = [&](cd gain) {
          double error = 0.0, reference = 0.0;
          for (size_t symbol = 0; symbol < numSym; ++symbol) {
            if (excludedSym[symbol] || resourceMask[symbol][i])
              continue;
            const cd measured = g.z[symbol][i] / gain;
            const cd decision = hardDecision(measured, mod);
            error += std::norm(measured - decision);
            reference += std::norm(decision);
          }
          return reference > 0.0 ? error / reference : 0.0;
        };
        double bestError = gainError(cd(1.0, 0.0));
        cd bestGain(1.0, 0.0);
        if (bestError > 1e-5) {
          for (int candidate = 0; candidate <= 30; ++candidate) {
            for (int phase = -4; phase <= 4; ++phase) {
              cd gain = std::polar(0.65 + 0.025 * candidate, 0.05 * phase);
              for (int refinement = 0; refinement < 3; ++refinement) {
                cd cross(0.0, 0.0);
                double reference = 0.0;
                for (size_t symbol = 0; symbol < numSym; ++symbol) {
                  if (excludedSym[symbol] || resourceMask[symbol][i])
                    continue;
                  const cd decision = hardDecision(g.z[symbol][i] / gain, mod);
                  cross += g.z[symbol][i] * std::conj(decision);
                  reference += std::norm(decision);
                }
                if (reference > 0.0 && std::abs(cross) > 0.0)
                  gain = cross / reference;
              }
              const double error = gainError(gain);
              if (error < bestError) {
                bestError = error;
                bestGain = gain;
              }
            }
          }
          if (std::abs(bestGain - cd(1.0, 0.0)) > 1e-9) {
            g.channel[i] *= bestGain;
            continue;
          }
        }
      }
      if (den > 0.0 && std::abs(num) > 0.0) {
        g.channel[i] = num / den;
      }
    }
  }
  if (equalize && !precoding && iterations > 1) {
    for (size_t carrier = 0; carrier < numSc; ++carrier) {
      if (excludedFrequency[carrier])
        continue;
      cd cross(0.0, 0.0);
      double reference = 0.0;
      for (size_t symbol = 0; symbol < numSym; ++symbol) {
        if (excludedSym[symbol] || resourceMask[symbol][carrier])
          continue;
        cross += g.z[symbol][carrier] * std::conj(g.ref[symbol][carrier]);
        reference += std::norm(g.ref[symbol][carrier]);
      }
      if (reference > 0.0 && std::abs(cross) > 0.0) {
        const cd gain = cross / reference;
        g.channel[carrier] *= gain;
        for (size_t symbol = 0; symbol < numSym; ++symbol) {
          g.z[symbol][carrier] /= gain;
          g.ref[symbol][carrier] = hardDecision(g.z[symbol][carrier], mod);
        }
      }
    }
  }
  // the timing ramp is odd around the allocation centre, so the mean correction phase is the CPE
  g.cpe.assign(numSym, 0.0);
  for (size_t l = 0; l < numSym; ++l) {
    cd acc(0.0, 0.0);
    for (size_t i = 0; i < numSc; ++i) if (!excludedSc[i]) acc += corr[l][i];
    g.cpe[l] = std::abs(acc) > 0.0 ? -std::arg(acc) : 0.0;
  }
  return g;
}

}

DmrsConfig::DmrsConfig()
  : exclude(true), mappingTypeB(false), typeAPosition(2), additionalPositions(0), doubleSymbol(false)
{
}

Config::Config()
  : numerology(1), fftSize(0), frequencyRange(FR1), channelBandwidthMHz(0), numResourceBlocks(0),
    resourceBlockOffset(0), extendedCp(false), modulation(MOD_AUTO), maxSymbols(0),
    burstSearch(true), burstSearchThresholdDb(-20.0), timingSearch(true), syncSearchSymbols(4), cfoCorrection(true), carrierOffsetHz(0.0),
    equalize(true), equalizerIterations(3), phaseTracking(true), timingTracking(true), amplitudeTracking(true),
    transformPrecoding(false),
    dcPunctured(false), removeIqOffset(false), symbolTimingAdjustmentPercent(-3.125), resample(true), sampleRate(0.0)
{
}

Result::Result()
    : ok(false), fftSize(0), resampleRate(0.0), numResourceBlocks(0), carrierResourceBlocks(0), firstSubcarrierIndex(0),
      numSymbols(0), timingOffset(0), slotStartSymbol(0), acquisitionFrequencyErrorHz(0.0), frequencyErrorHz(0.0),
      detectedModulation(MOD_QPSK), rmsEvmPercent(0.0), peakEvmPercent(0.0), rmsEvmDb(0.0), burstStart(0),
      burstLength(0), syncCorrelation(0.0), normalizationFactor(1.0), dataPowerDb(0.0), burstPowerDb(0.0),
      iqOffsetDb(0.0), iqGainImbalanceDb(0.0), iqQuadratureErrorDeg(0.0), iqTimingSkewSec(0.0),
      commonPhaseErrorDeg(0.0), dmrsEvmPercent(0.0), dmrsPowerDb(0.0), flatnessRippleRange1Db(0.0),
      flatnessRippleRange2Db(0.0), flatnessMaxRange1MinRange2Db(0.0), flatnessMaxRange2MinRange1Db(0.0),
      peakEvmSymbol(0), peakEvmSubcarrier(0), numSlots(0), firstSlotIndex(0)
{
}

int normalCpLength(int fftSize)
{
  return 144 * fftSize / 2048;
}

int longCpLength(int fftSize, int numerology)
{
  return (144 + 16 * (1 << numerology)) * fftSize / 2048;
}

int extendedCpLength(int fftSize)
{
  return 512 * fftSize / 2048;
}

int symbolsPerSlot(bool extendedCp)
{
  return extendedCp ? 12 : 14;
}

int maxResourceBlocks(int fftSize)
{
  return static_cast<int>((fftSize * 0.9) / 12.0);
}

int carrierResourceBlocks(int frequencyRange, int channelBandwidthMHz, int numerology)
{
  if (numerology == 4) {
    if (frequencyRange != FR2)
      return 0;
    if (channelBandwidthMHz == 100)
      return 32;
    return carrierResourceBlocks(FR2, channelBandwidthMHz, 3) / 2;
  }
  // TS 38.101-1 table 5.3.2-1 (FR1) and TS 38.101-2 table 5.3.2-1 (FR2)
  struct Entry { int bw; int rb[4];
  };

  static const Entry fr1[] = {
      {3, {15, 0, 0, 0}},
      {5, {25, 11, 0, 0}},
      {10, {52, 24, 11, 0}},
      {15, {79, 38, 18, 0}},
      {20, {106, 51, 24, 0}},
      {25, {133, 65, 31, 0}},
      {30, {160, 78, 38, 0}},
      {35, {188, 92, 44, 0}},
      {40, {216, 106, 51, 0}},
      {45, {242, 119, 58, 0}},
      {50, {270, 133, 65, 0}},
      {60, {0, 162, 79, 0}},
      {70, {0, 189, 93, 0}},
      {80, {0, 217, 107, 0}},
      {90, {0, 245, 121, 0}},
      {100, {0, 273, 135, 0}},
  };
  static const Entry fr2[] = {
    {  50, { 0, 0,  66,  32 } },
    { 100, { 0, 0, 132,  66 } },
    { 200, { 0, 0, 264, 132 } },
    { 400, { 0, 0,   0, 264 } },
  };
  if (numerology < 0 || numerology > 3) return 0;
  const Entry* table = frequencyRange == FR2 ? fr2 : fr1;
  const size_t count = frequencyRange == FR2 ? sizeof(fr2) / sizeof(fr2[0]) : sizeof(fr1) / sizeof(fr1[0]);
  for (size_t i = 0; i < count; ++i) {
    if (table[i].bw == channelBandwidthMHz) return table[i].rb[numerology];
  }
  return 0;
}

std::vector<int> dmrsSymbolsInSlot(const DmrsConfig& dmrs)
{
  // PUSCH DM-RS positions for ld = 14 without intra slot frequency hopping
  std::vector<int> out;
  const int l0 = dmrs.mappingTypeB ? 0 : (dmrs.typeAPosition == 3 ? 3 : 2);
  const int add = std::max(0, std::min(3, dmrs.additionalPositions));
  if (dmrs.doubleSymbol) {
    out.push_back(l0);
    out.push_back(l0 + 1);
    if (add >= 1) {
      out.push_back(10);
      out.push_back(11);
    }
    return out;
  }
  out.push_back(l0);
  if (dmrs.mappingTypeB) {
    if (add == 1) { out.push_back(10); }
    if (add == 2) { out.push_back(6); out.push_back(10); }
    if (add == 3) { out.push_back(3); out.push_back(6); out.push_back(9); }
  } else {
    if (add == 1) { out.push_back(11); }
    if (add == 2) { out.push_back(7); out.push_back(11); }
    if (add == 3) { out.push_back(5); out.push_back(8); out.push_back(11); }
  }
  return out;
}

void fft(std::vector<cd>& a, bool inverse)
{
  const size_t n = a.size();
  if (n < 2) return;
  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(a[i], a[j]);
  }
  for (size_t len = 2; len <= n; len <<= 1) {
    const double ang = 2.0 * PI / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
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

void dft(std::vector<cd>& a, bool inverse)
{
  if (sExternalFft) {
    sExternalFft(a.data(), a.size(), inverse);
    return;
  }
  const size_t m = a.size();
  if (m < 2) return;
  if (sExternalFft) {
    sExternalFft(a.data(), m, inverse);
    return;
  }
  if (isPowerOfTwo(static_cast<int>(m))) {
    fft(a, inverse);
    return;
  }
  // Bluestein: X[k] = w[k] * sum_n (x[n] w[n]) conj(w[k-n]),  w[n] = exp(-j pi n^2 / m)
  size_t l = 1;
  while (l < 2 * m - 1) l <<= 1;
  const double sign = inverse ? 1.0 : -1.0;
  std::vector<cd> w(m);
  for (size_t n = 0; n < m; ++n) {
    const double phase = sign * PI * static_cast<double>((n * n) % (2 * m)) / static_cast<double>(m);
    w[n] = std::polar(1.0, phase);
  }
  std::vector<cd> fa(l, cd(0.0, 0.0)), fb(l, cd(0.0, 0.0));
  for (size_t n = 0; n < m; ++n) {
    fa[n] = a[n] * w[n];
    fb[n] = std::conj(w[n]);
    if (n > 0) fb[l - n] = std::conj(w[n]);
  }
  fft(fa, false);
  fft(fb, false);
  for (size_t i = 0; i < l; ++i) fa[i] *= fb[i];
  fft(fa, true);
  const double invL = 1.0 / static_cast<double>(l);
  for (size_t k = 0; k < m; ++k) {
    a[k] = fa[k] * invL * w[k];
  }
}

std::vector<cd> resample(const std::vector<cd>& in, double fsIn, double fsOut)
{
  if (in.empty() || std::fabs(fsIn - fsOut) <= 1e-9 * fsIn) return in;
  const double ratio = fsIn / fsOut;                       // input samples per output sample
  // cut-off just below the smaller Nyquist rate; the Blackman windowed kernel needs
  // ~100 taps for a transition band narrow enough to leave a 90 % occupied carrier untouched
  const double fc = 0.5 * std::min(1.0, fsOut / fsIn) * 0.98; // cycles per input sample
  const int halfWidth = static_cast<int>(std::ceil(48.0 / (2.0 * fc)));
  const size_t numOut = static_cast<size_t>(std::floor(static_cast<double>(in.size() - 1) / ratio)) + 1;
  std::vector<cd> out(numOut);
  const int n = static_cast<int>(in.size());
  for (size_t i = 0; i < numOut; ++i) {
    const double t = static_cast<double>(i) * ratio;
    const int center = static_cast<int>(std::floor(t));
    cd acc(0.0, 0.0);
    double wsum = 0.0;
    for (int k = center - halfWidth + 1; k <= center + halfWidth; ++k) {
      const double tau = static_cast<double>(k) - t;
      const double x = 2.0 * fc * tau;
      const double sinc = std::fabs(x) < 1e-12 ? 1.0 : std::sin(PI * x) / (PI * x);
      const double u = tau / static_cast<double>(halfWidth);
      if (u <= -1.0 || u >= 1.0) continue;
      const double window = 0.42 + 0.5 * std::cos(PI * u) + 0.08 * std::cos(2.0 * PI * u);
      const double h = 2.0 * fc * sinc * window;
      wsum += h;
      if (k >= 0 && k < n) acc += in[k] * h;
    }
    // renormalise the truncated kernel so the passband gain stays at 1
    out[i] = wsum > 0.0 ? acc / wsum : acc;
  }
  return out;
}

cd hardDecision(const cd& z, Modulation mod)
{
  if (mod == MOD_AUTO) mod = MOD_QPSK;
  const double s = qamScale(mod);
  const int maxLevel = qamMaxLevel(mod);
  return cd(decideAxis(z.real() / s, maxLevel) * s, decideAxis(z.imag() / s, maxLevel) * s);
}


void setFftProvider(FftProviderFn fn) { sExternalFft = fn; }
FftProviderFn getFftProvider() { return sExternalFft; }


Result demodulate(const std::complex<float>* iq, size_t numSamples, const Config& cfg)
{
  Result r;
  const char* traceValue = std::getenv("XOC_DEMOD_TRACE");
  const bool trace = traceValue && (*traceValue == '1' || *traceValue == 'y' ||
                                    *traceValue == 'Y' || *traceValue == 't' || *traceValue == 'T');
  const bool profile = std::getenv("NR_DEMOD_PROFILE") != NULL;
  const std::chrono::steady_clock::time_point profileStart = std::chrono::steady_clock::now();
  const auto profileMark = [&](const char* stage) {
    if (!profile) return;
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - profileStart).count();
    std::fprintf(stderr, "[demod-profile] stage=%s ms=%.3f\n", stage, milliseconds);
  };
  if (trace) {
    std::fprintf(stderr, "[demod-trace] core-enter samples=%zu sampleRate=%.0f mu=%d fft=%s\n",
                 numSamples, cfg.sampleRate, cfg.numerology,
                 sExternalFft ? "external" : "builtin");
  }

  if (cfg.numerology < 0 || cfg.numerology > 4) {
    r.error = "Numerology must be in the range 0..4.";
    return r;
  }
  if (cfg.extendedCp && cfg.numerology != 2) {
    r.error = "Extended cyclic prefix is only defined for numerology 2 (60 kHz).";
    return r;
  }
  if (!(cfg.sampleRate > 0.0)) {
    r.error = "Sample rate must be positive.";
    return r;
  }
  if (cfg.modulation < MOD_AUTO || cfg.modulation > MOD_QAM256) {
    r.error = "Unknown modulation format.";
    return r;
  }
  if (cfg.transformPrecoding && cfg.extendedCp) {
    r.error = "Transform precoding is not supported with extended cyclic prefix.";
    return r;
  }

  Grid grid;
  if (!resolveGrid(cfg, grid, r.error)) {
    return r;
  }
  const double scs = SCS_BASE_HZ * static_cast<double>(1 << cfg.numerology);
  const int N = grid.fftSize;
  const int numSc = 12 * grid.numRb;
  const int k0 = grid.firstSubcarrier;

  const int cpNormal = cfg.extendedCp ? extendedCpLength(N) : normalCpLength(N);
  const int cpLong = cfg.extendedCp ? cpNormal : longCpLength(N, cfg.numerology);
  const int period = cfg.extendedCp ? 0 : (7 << cfg.numerology);
  const size_t symLen = static_cast<size_t>(N + cpNormal);

  if (!iq || numSamples == 0) {
    r.error = "No waveform data supplied.";
    return r;
  }

  std::vector<cd> x(numSamples);
  for (size_t i = 0; i < numSamples; ++i) {
    x[i] = cd(iq[i].real(), iq[i].imag());
  }
  if (cfg.carrierOffsetHz != 0.0) {
    const double w = -2.0 * PI * cfg.carrierOffsetHz / cfg.sampleRate;
    for (size_t m = 0; m < numSamples; ++m) {
      x[m] *= std::polar(1.0, w * static_cast<double>(m));
    }
  }
  if (grid.resampled) {
    x = resample(x, cfg.sampleRate, grid.gridRate);
  }

  // burst search: a capture may contain idle time before and after the transmission
  // (TDD, burst mode). The power envelope (one FFT length average) locates the active
  // part; everything else is dropped so that the timing search sees signal only.
  size_t burstOffset = 0;
  r.burstStart = 0;
  r.burstLength = static_cast<int>(numSamples);
  bool burstEndDetected = !cfg.burstSearch;
  if (cfg.burstSearch && x.size() > static_cast<size_t>(symLen)) {
    const size_t total = x.size();
    std::vector<double> cum(total + 1, 0.0);
    for (size_t m = 0; m < total; ++m) cum[m + 1] = cum[m] + std::norm(x[m]);
    // peak of the one FFT length envelope, edges located with a short (CP length)
    // window so that the burst start is known to within one CP
    const size_t win = static_cast<size_t>(N);
    double peak = 0.0;
    for (size_t m = 0; m + win <= total; m += 16) peak = std::max(peak, (cum[m + win] - cum[m]) / static_cast<double>(win));
    const double thr = peak * std::pow(10.0, cfg.burstSearchThresholdDb / 10.0);
    const size_t edge = static_cast<size_t>(std::max(16, cpNormal));
    size_t first = total, last = 0;
    size_t segmentStart = total, segmentEnd = 0;
    size_t boundedStart = total, boundedEnd = 0;
    bool quietAfterBurst = false;
    for (size_t m = 0; m + edge <= total; m += 16) {
      const double envelope = (cum[m + edge] - cum[m]) / static_cast<double>(edge);
      if (envelope > thr) {
        if (first == total) first = m;
        if (segmentStart == total)
          segmentStart = m;
        segmentEnd = m + edge;
        last = m + edge;
        quietAfterBurst = false;
      } else if (first < total && m >= last) {
        quietAfterBurst = true;
        if (boundedStart == total && segmentStart >= edge && segmentStart < segmentEnd &&
            segmentEnd - segmentStart >= static_cast<size_t>(symLen)) {
          boundedStart = segmentStart;
          boundedEnd = segmentEnd;
        }
        segmentStart = total;
        segmentEnd = 0;
      }
    }
    if (boundedStart < boundedEnd) {
      first = boundedStart;
      last = boundedEnd;
      quietAfterBurst = true;
    }
    if (first < last) {
      burstEndDetected = first < edge || quietAfterBurst;
      const size_t margin = static_cast<size_t>(cpLong);
      const size_t from = first > margin ? first - margin : 0;
      const size_t to = std::min(total, last + margin);
      if (to - from >= static_cast<size_t>(symLen)) {
        burstOffset = from;
        x = std::vector<cd>(x.begin() + from, x.begin() + to);
        r.burstStart = static_cast<int>(std::floor(static_cast<double>(first) * cfg.sampleRate / grid.gridRate + 0.5));
        r.burstLength = static_cast<int>(std::floor(static_cast<double>(last - first) * cfg.sampleRate / grid.gridRate + 0.5));
      }
    }
  }
  const size_t n = x.size();
  {
    cd dc(0.0, 0.0);
    double pwr = 0.0;
    for (size_t m = 0; m < n; ++m) { dc += x[m]; pwr += std::norm(x[m]); }
    if (n) { dc /= static_cast<double>(n); pwr /= static_cast<double>(n); }
    r.iqOffsetDb = pwr > 0.0 ? 10.0 * std::log10(std::norm(dc) / pwr + 1e-30) : 0.0;
    r.burstPowerDb = 10.0 * std::log10(pwr + 1e-30);
    if (cfg.removeIqOffset) {
      for (size_t m = 0; m < n; ++m) x[m] -= dc;
    }
  }
  if (n < symLen) {
    std::ostringstream os;
    os << "At least " << symLen << " samples (at " << grid.gridRate << " Sa/s) are required for one OFDM symbol, got " << n << ".";
    r.error = os.str();
    return r;
  }
  profileMark("preprocess");

  // stage 1: coarse symbol timing, CP correlation accumulated over a few symbols
  // (ambiguous by cpLong - cpNormal when a long CP symbol is inside the window)
  size_t coarse = 0;
  if (cfg.timingSearch) {
    const size_t wanted = static_cast<size_t>(std::max(1, cfg.syncSearchSymbols));
    const size_t maxSyms = std::min<size_t>(wanted, (n - symLen) / symLen + 1);
    double best = -1.0;
    for (size_t d = 0; d < symLen; ++d) {
      double metric = 0.0;
      for (size_t m = 0; m < maxSyms; ++m) {
        const size_t pos = d + m * symLen;
        if (pos + static_cast<size_t>(cpNormal + N) > n) break;
        metric += cpMetric(x, pos, cpNormal, N, NULL);
      }
      if (metric > best) {
        best = metric;
        coarse = d;
      }
    }
  }

  // stage 2: joint refinement of start sample and long CP position in the half subframe.
  // A layout starting cpLong - cpNormal later with the first symbol treated as normal CP
  // yields the same FFT windows; prefer the earliest start among near-equal hypotheses
  // so that the long CP (slot boundary) is kept.
  size_t start = coarse;
  int longOffset = 0;
  {
    const size_t extra = static_cast<size_t>(cpLong - cpNormal);
    const size_t dMin = cfg.timingSearch ? (coarse > extra ? coarse - extra : 0) : 0;
    const size_t dMax = cfg.timingSearch ? std::min(coarse + extra, symLen - 1) : 0;
    int hypSyms = period > 0 ? 2 * period : 4;
    if (cfg.maxSymbols > 0 && cfg.maxSymbols < hypSyms) hypSyms = cfg.maxSymbols;
    struct Hypothesis { size_t d; int o; double metric; };
    std::vector<Hypothesis> hyps;
    double best = -1.0;
    size_t bestIdx = 0;
    for (size_t d = dMin; d <= dMax; ++d) {
      const size_t normalSymbols = (n - d) / static_cast<size_t>(cpNormal + N);
      const int numHyp = period > 0
          ? std::min(period, static_cast<int>(normalSymbols) + 1)
          : 1;
      for (int o = 0; o < numHyp; ++o) {
        const std::vector<SymbolLayout> hyp =
            layoutSymbols(n, d, N, cpNormal, cpLong, period, o, hypSyms);
        double metric = 0.0;
        for (size_t j = 0; j < hyp.size(); ++j) {
          metric += cpMetric(x, hyp[j].cpStart, hyp[j].cpLen, N, NULL);
        }
        Hypothesis h = { d, o, metric };
        hyps.push_back(h);
        if (metric > best + 1e-9) { best = metric; bestIdx = hyps.size() - 1; }
      }
    }
    start = hyps[bestIdx].d;
    longOffset = hyps[bestIdx].o;
    if (period > 0 && longOffset != 0 && start >= extra) {
      // are the `extra` samples in front of the chosen start a cyclic copy as well?
      // then the first symbol carries a long CP and the earlier start is the true one
      if (cpMetric(x, start - extra, static_cast<int>(extra), N, NULL) > 0.45) {
        start -= extra;
        longOffset = 0;
      }
    }
  }
  profileMark("timing-cfo");
  r.timingOffset = static_cast<int>(std::floor(static_cast<double>(start + burstOffset) * cfg.sampleRate / grid.gridRate + 0.5));

  const std::vector<SymbolLayout> layout =
      layoutSymbols(n, start, N, cpNormal, cpLong, period, longOffset, cfg.maxSymbols);
  if (layout.empty()) {
    r.error = "Not enough samples after the detected symbol start.";
    return r;
  }
  const size_t numSym = layout.size();

  // sub-sample timing: parabolic interpolation of the CP metric around the detected start
  // (a resampled capture rarely has its symbol boundaries on the FFT grid)
  double fractional = 0.0;
  if (cfg.timingSearch) {
    double m[3] = { 0.0, 0.0, 0.0 };
    bool valid = true;
    for (int d = -1; d <= 1 && valid; ++d) {
      for (size_t j = 0; j < numSym; ++j) {
        const long pos = static_cast<long>(layout[j].cpStart) + d;
        if (pos < 0 || static_cast<size_t>(pos) + layout[j].cpLen + N > n) { valid = false; break; }
        m[d + 1] += cpMetric(x, static_cast<size_t>(pos), layout[j].cpLen, N, NULL);
      }
    }
    const double denom = m[0] - 2.0 * m[1] + m[2];
    if (valid && denom < 0.0) {
      fractional = std::max(-0.5, std::min(0.5, 0.5 * (m[0] - m[2]) / denom));
    }
  }

  // slot boundary: the first long CP symbol starts a slot (mu >= 1); fall back to symbol 0
  int slotStart = 0;
  for (size_t j = 0; j < numSym; ++j) {
    if (layout[j].cpLen == cpLong && period > 0) { slotStart = static_cast<int>(j); break; }
  }
  r.slotStartSymbol = slotStart;

  // carrier frequency offset from the CP correlation phase
  double eps = 0.0;
  {
    cd corr(0.0, 0.0);
    double rho = 0.0;
    const std::vector<SymbolLayout> acquisitionLayout =
        layoutSymbols(n, start, N, cpNormal, cpLong, period, longOffset, 0);
    for (size_t symbol = 0; symbol < acquisitionLayout.size(); ++symbol) {
      cd correlation;
      const double metric =
          cpMetric(x, acquisitionLayout[symbol].cpStart, acquisitionLayout[symbol].cpLen, N, &correlation);
      if (symbol < numSym)
        rho += metric;
      const SymbolLayout& acquired = acquisitionLayout[symbol];
      if (acquired.cpLen > 1) {
        cd prefixSum(0.0, 0.0), suffixSum(0.0, 0.0);
        for (int sample = 0; sample < acquired.cpLen; ++sample) {
          prefixSum += x[acquired.cpStart + sample];
          suffixSum += x[acquired.cpStart + sample + N];
        }
        correlation -= prefixSum * std::conj(suffixSum) / static_cast<double>(acquired.cpLen);
      }
      corr += correlation;
    }
    r.syncCorrelation = rho / static_cast<double>(numSym);
    eps = std::abs(corr) > 0.0 ? -std::arg(corr) / (2.0 * PI) : 0.0;
    r.frequencyErrorHz = eps * scs;
    r.acquisitionFrequencyErrorHz = r.frequencyErrorHz;
    if (cfg.cfoCorrection && eps != 0.0) {
      const double w = -2.0 * PI * eps / static_cast<double>(N);
      for (size_t m = 0; m < n; ++m) {
        x[m] *= std::polar(1.0, w * static_cast<double>(m));
      }
    }
  }

  // FFT per symbol, extract allocated subcarriers, accumulate spectrum
  std::vector<std::vector<cd> > y(numSym, std::vector<cd>(numSc));
  std::vector<double> spectrum(N, 0.0);
  const double fftScale = 1.0 / std::sqrt(static_cast<double>(N));
  double power = 0.0;
  auto extractGrid = [&](double residualFrequency) {
    power = 0.0;
    std::fill(spectrum.begin(), spectrum.end(), 0.0);
    for (size_t j = 0; j < numSym; ++j) {
      // percent of the FFT length (Keysight convention, -3.125 % = N/32 into the CP), clamped to the CP
      int shift = static_cast<int>(std::floor(cfg.symbolTimingAdjustmentPercent / 100.0 * N + 0.5));
      shift = std::max(-layout[j].cpLen, std::min(0, shift));
      const size_t fftStart = layout[j].cpStart + static_cast<size_t>(layout[j].cpLen + shift);
      std::vector<cd> buf(x.begin() + fftStart, x.begin() + fftStart + N);
      if (residualFrequency != 0.0) {
        for (int sample = 0; sample < N; ++sample)
          buf[sample] *= std::polar(1.0, -residualFrequency * static_cast<double>(fftStart + sample));
      }
      fft(buf, false);
      // the window starts (fractional - shift) samples before the true symbol start: undo the
      // resulting linear phase so that the equalizer only sees the channel
      const double early = fractional - static_cast<double>(shift);
      if (early != 0.0) {
        for (int b = 0; b < N; ++b) {
          const int k = b < N / 2 ? b : b - N;
          buf[b] *= std::polar(1.0, 2.0 * PI * static_cast<double>(k) * early / N);
        }
      }
      for (int b = 0; b < N; ++b) {
        buf[b] *= fftScale;
        spectrum[(b + N / 2) % N] += std::norm(buf[b]);
      }
      for (int i = 0; i < numSc; ++i) {
        const int bin = ((k0 + i) % N + N) % N;
        y[j][i] = buf[bin];
        power += std::norm(buf[bin]);
      }
    }
  };
  extractGrid(0.0);
  profileMark("fft-extract");
  r.spectrumDb.resize(N);
  for (int b = 0; b < N; ++b) {
    r.spectrumDb[b] = 10.0 * std::log10(spectrum[b] / static_cast<double>(numSym) + 1e-30);
  }

  const double gain = std::sqrt(power / static_cast<double>(numSym * numSc));
  if (!(gain > 0.0)) {
    r.error = "Input signal has no energy on the allocated subcarriers.";
    return r;
  }
  r.normalizationFactor = 1.0 / gain;
  r.dataPowerDb = 20.0 * std::log10(gain);
  for (size_t j = 0; j < numSym; ++j) {
    for (int i = 0; i < numSc; ++i) {
      y[j][i] /= gain;
    }
  }

  // symbols excluded from the statistics: DM-RS positions and user supplied indices
  std::vector<bool> excluded(numSym, false);
  std::vector<bool> excludedFromDecisions(numSym, false);
  const std::vector<int> positions = dmrsSymbolsInSlot(cfg.dmrs);
  const int dmrsPerSlot = symbolsPerSlot(cfg.extendedCp);
  for (size_t j = 0; j < numSym; ++j) {
    const int rel = ((static_cast<int>(j) - slotStart) % dmrsPerSlot + dmrsPerSlot) % dmrsPerSlot;
    if (std::find(positions.begin(), positions.end(), rel) != positions.end()) {
      r.dmrsSymbols.push_back(static_cast<int>(j));
      excludedFromDecisions[j] = true;
      if (cfg.dmrs.exclude) excluded[j] = true;
    }
  }
  for (size_t e = 0; e < cfg.excludedSymbols.size(); ++e) {
    const int idx = cfg.excludedSymbols[e];
    if (idx >= 0 && static_cast<size_t>(idx) < numSym) {
      excluded[idx] = true;
      excludedFromDecisions[idx] = true;
    }
  }
  size_t numIncluded = 0;
  for (size_t j = 0; j < numSym; ++j) if (!excluded[j]) ++numIncluded;
  if (numIncluded == 0) {
    r.error = "All demodulated symbols are excluded from the measurement (DM-RS / ExcludedSymbols).";
    return r;
  }

  std::vector<bool> excludedSc(numSc, false);
  size_t numIncludedSc = static_cast<size_t>(numSc);
  if (cfg.dcPunctured && !cfg.transformPrecoding && k0 <= 0 && k0 + numSc > 0) {
    excludedSc[-k0] = true;
    --numIncludedSc;
  }

  Modulation mod = static_cast<Modulation>(cfg.modulation);
  std::vector<std::vector<bool>> resourceMask(numSym, std::vector<bool>(numSc, false));
  std::vector<cd> phaseReference(numSc, cd(0.0, 0.0));
  if (!cfg.transformPrecoding && numSc >= 240) {
    std::vector<int> sequence(127 + 7, 0);
    const int initial[] = {0, 1, 1, 0, 1, 1, 1};
    std::copy(initial, initial + 7, sequence.begin());
    for (int point = 0; point < 127; ++point)
      sequence[point + 7] = (sequence[point + 4] + sequence[point]) % 2;
    for (size_t symbol = 0; symbol < numSym; ++symbol) {
      for (int start = 0; start + 240 <= numSc; ++start) {
        double energy = 0.0;
        size_t count = 0;
        cd cross[3] = {cd(0.0, 0.0), cd(0.0, 0.0), cd(0.0, 0.0)};
        for (int point = 0; point < 127; ++point) {
          const int carrier = start + 56 + point;
          if (excludedSc[carrier])
            continue;
          energy += std::norm(y[symbol][carrier]);
          ++count;
          for (int identity = 0; identity < 3; ++identity)
            cross[identity] +=
                y[symbol][carrier] * static_cast<double>(1 - 2 * sequence[(point + 43 * identity) % 127]);
        }
        double correlation = 0.0;
        int bestIdentity = 0;
        for (int identity = 0; identity < 3; ++identity)
          if (energy > 0.0 && std::norm(cross[identity]) / (count * energy) > correlation) {
            correlation = std::norm(cross[identity]) / (count * energy);
            bestIdentity = identity;
          }
        if (correlation < 0.7)
          continue;
        if (profile)
          std::fprintf(stderr, "[demod-profile] pss-symbol=%zu ssb-start=%d correlation=%.6f\n", symbol, start,
              correlation);
        for (int point = 0; point < 127; ++point) {
          const int carrier = start + 56 + point;
          if (!excludedSc[carrier] && std::abs(phaseReference[carrier]) == 0.0)
            phaseReference[carrier] =
                y[symbol][carrier] * static_cast<double>(1 - 2 * sequence[(point + 43 * bestIdentity) % 127]);
        }
        double sumPosition = 0.0, sumPhase = 0.0, sumPositionSquared = 0.0, sumPositionPhase = 0.0;
        double sumLogMagnitude = 0.0, previousPhase = 0.0;
        size_t phaseCount = 0;
        for (int point = 0; point < 127; ++point) {
          const cd known = phaseReference[start + 56 + point];
          if (std::abs(known) == 0.0)
            continue;
          double phase = std::arg(known);
          if (phaseCount) {
            while (phase - previousPhase > PI)
              phase -= 2.0 * PI;
            while (phase - previousPhase < -PI)
              phase += 2.0 * PI;
          }
          previousPhase = phase;
          const double position = point - 63.0;
          sumPosition += position;
          sumPhase += phase;
          sumPositionSquared += position * position;
          sumPositionPhase += position * phase;
          sumLogMagnitude += std::log(std::abs(known));
          ++phaseCount;
        }
        const double denominator = phaseCount * sumPositionSquared - sumPosition * sumPosition;
        if (phaseCount > 1 && denominator > 0.0) {
          const double slope = (phaseCount * sumPositionPhase - sumPosition * sumPhase) / denominator;
          const double phase = (sumPhase - slope * sumPosition) / phaseCount;
          const double magnitude = std::exp(sumLogMagnitude / phaseCount);
          for (int point = std::max(-6, -start); point < std::min(246, numSc - start); ++point)
            if ((point < 56 || point > 182) && !excludedSc[start + point])
              phaseReference[start + point] = std::polar(magnitude, phase + slope * (point - 119.0));
        }
        for (int carrier = std::max(0, start - 6); carrier < std::min(numSc, start + 246); ++carrier) {
          double guardPower = 0.0;
          size_t guardCount = 0;
          for (size_t offset = 0; offset < 4 && symbol + offset < numSym; ++offset) {
            guardPower += std::norm(y[symbol + offset][carrier]);
            ++guardCount;
          }
          if ((carrier < start || carrier >= start + 240) && guardPower >= 0.001 * guardCount * energy / count)
            continue;
          for (size_t offset = 0; offset < 4 && symbol + offset < numSym; ++offset)
            resourceMask[symbol + offset][carrier] = true;
        }
        break;
      }
    }
  }
  const int puncturedCarrier = cfg.dcPunctured && cfg.transformPrecoding && k0 <= 0 && k0 + numSc > 0 ? -k0 : -1;
  if (mod == MOD_AUTO) {
    // Minimum decision error over the blind equalized points (no decision directed
    // update, its gain ambiguity would let a wrong grid fit). The true grid has the
    // smallest error as long as the EVM stays below half the 256QAM spacing (~7 %).
    const Modulation candidates[] = { MOD_QPSK, MOD_QAM16, MOD_QAM64, MOD_QAM256 };
    double bestErr = std::numeric_limits<double>::infinity();
    for (size_t c = 0; c < 4; ++c) {
      const EqualizedGrid probe = equalizeAndDecide(y, excludedFromDecisions, excludedSc, candidates[c], cfg.equalize,
          cfg.phaseTracking, cfg.timingTracking, cfg.amplitudeTracking, cfg.transformPrecoding, 1, puncturedCarrier,
          resourceMask, 0, &phaseReference);
      double err = 0.0;
      size_t count = 0;
      for (size_t j = 0; j < numSym; ++j) {
        if (excludedFromDecisions[j])
          continue;
        for (int i = 0; i < numSc; ++i) {
          if (excludedSc[i]) continue;
          err += std::norm(probe.z[j][i] - probe.ref[j][i]);
          ++count;
        }
      }
      err = count ? err / static_cast<double>(count) : err;
      if (err < bestErr) {
        bestErr = err;
        mod = candidates[c];
      }
    }
  }
  r.detectedModulation = mod;

  EqualizedGrid g = equalizeAndDecide(y, excludedFromDecisions, excludedSc, mod, cfg.equalize, cfg.phaseTracking,
      cfg.timingTracking, cfg.amplitudeTracking, cfg.transformPrecoding, cfg.equalizerIterations, puncturedCarrier,
      resourceMask, 0, &phaseReference);
  auto decisionError = [&](const EqualizedGrid& candidate) {
    double error = 0.0, reference = 0.0;
    for (size_t symbol = 0; symbol < numSym; ++symbol) {
      if (excludedFromDecisions[symbol])
        continue;
      for (int carrier = 0; carrier < numSc; ++carrier) {
        if (excludedSc[carrier] || resourceMask[symbol][carrier])
          continue;
        error += std::norm(candidate.z[symbol][carrier] - candidate.ref[symbol][carrier]);
        reference += std::norm(candidate.ref[symbol][carrier]);
      }
    }
    return reference > 0.0 ? error / reference : std::numeric_limits<double>::infinity();
  };
  int phaseWindowOverride = 0;
  if (!cfg.transformPrecoding && cfg.equalize && cfg.phaseTracking && mod == MOD_QAM256 && decisionError(g) > 1e-6) {
    double bestError = decisionError(g);
    for (const int window : {8, 32, 64}) {
      EqualizedGrid candidate = equalizeAndDecide(y, excludedFromDecisions, excludedSc, mod, cfg.equalize,
          cfg.phaseTracking, cfg.timingTracking, cfg.amplitudeTracking, false, cfg.equalizerIterations,
          puncturedCarrier, resourceMask, window, &phaseReference);
      const double error = decisionError(candidate);
      if (error < bestError) {
        bestError = error;
        phaseWindowOverride = window;
        g = std::move(candidate);
      }
    }
  }
  if (cfg.cfoCorrection && cfg.phaseTracking) {
    double errorPower = 0.0, referencePower = 0.0;
    double sumTime = 0.0, sumPhase = 0.0, sumTimeSquared = 0.0, sumTimePhase = 0.0;
    double previousPhase = 0.0;
    size_t count = 0;
    for (size_t symbol = 0; symbol < numSym; ++symbol) {
      if (excludedFromDecisions[symbol])
        continue;
      for (int carrier = 0; carrier < numSc; ++carrier) {
        if (excludedSc[carrier] || resourceMask[symbol][carrier])
          continue;
        errorPower += std::norm(g.z[symbol][carrier] - g.ref[symbol][carrier]);
        referencePower += std::norm(g.ref[symbol][carrier]);
      }
      double phase = g.cpe[symbol];
      if (count) {
        while (phase - previousPhase > PI / 4.0)
          phase -= PI / 2.0;
        while (phase - previousPhase < -PI / 4.0)
          phase += PI / 2.0;
      }
      previousPhase = phase;
      const double time = static_cast<double>(layout[symbol].cpStart - layout[0].cpStart);
      sumTime += time;
      sumPhase += phase;
      sumTimeSquared += time * time;
      sumTimePhase += time * phase;
      ++count;
    }
    const double denominator = count * sumTimeSquared - sumTime * sumTime;
    if (count >= 3 && denominator > 0.0 && referencePower > 0.0 && errorPower < 0.0025 * referencePower) {
      const double residualFrequency = (count * sumTimePhase - sumTime * sumPhase) / denominator;
      const std::vector<std::vector<cd>> originalGrid(y);
      extractGrid(residualFrequency);
      for (size_t symbol = 0; symbol < numSym; ++symbol)
        for (int carrier = 0; carrier < numSc; ++carrier)
          y[symbol][carrier] /= gain;
      EqualizedGrid candidate = equalizeAndDecide(y, excludedFromDecisions, excludedSc, mod, cfg.equalize,
          cfg.phaseTracking, cfg.timingTracking, cfg.amplitudeTracking, cfg.transformPrecoding, cfg.equalizerIterations,
          puncturedCarrier, resourceMask, phaseWindowOverride, &phaseReference);
      if (profile)
        std::fprintf(stderr, "[demod-profile] cfo-acquisition=%.9g refinement=%.9g error-before=%.9g after=%.9g\n",
            r.frequencyErrorHz, residualFrequency * grid.gridRate / (2.0 * PI), decisionError(g),
            decisionError(candidate));
      if (decisionError(candidate) <= decisionError(g)) {
        g = std::move(candidate);
        r.frequencyErrorHz += residualFrequency * grid.gridRate / (2.0 * PI);
      } else {
        y = originalGrid;
      }
    }
  }
  profileMark("equalize");

  // EVM per 3GPP definition: error power normalised to reference power
  r.evmPerSymbol.assign(numSym, 0.0);
  r.evmPerSubcarrier.assign(numSc, 0.0);
  std::vector<double> scErr(numSc, 0.0), scRef(numSc, 0.0);
  double totErr = 0.0, totRef = 0.0, peak = 0.0;
  r.constellation.reserve(numSym * numSc);
  r.reference.reserve(numSym * numSc);
  r.errorVector.reserve(numSym * numSc);
  r.peakEvmPerSymbol.assign(numSym, 0.0);
  r.peakEvmPerSubcarrier.assign(numSc, 0.0);
  std::vector<double> scPeak(numSc, 0.0);
  const int perSlot = symbolsPerSlot(cfg.extendedCp);
  const int slotBase = slotStart % perSlot;
  auto slotOf = [&](size_t j) {
    return slotBase == 0 ? static_cast<int>(j) / perSlot : (static_cast<int>(j) - slotBase + perSlot) / perSlot;
  };
  r.numSlots = slotOf(numSym - 1) + 1;
  r.evmPerSlot.assign(r.numSlots, 0.0);
  r.peakEvmPerSlot.assign(r.numSlots, 0.0);
  std::vector<double> slotErr(r.numSlots, 0.0), slotRef(r.numSlots, 0.0), slotPeak(r.numSlots, 0.0);
  std::vector<double> slotCount(r.numSlots, 0.0);
  const int numRb = numSc / 12;
  std::vector<double> rbErr(numRb, 0.0), rbRef(numRb, 0.0), rbPower(numRb, 0.0), rbCount(numRb, 0.0);
  for (size_t j = 0; j < numSym; ++j) {
    double symErr = 0.0, symRef = 0.0, symPeak = 0.0;
    for (int i = 0; i < numSc; ++i) {
      const cd z = excludedSc[i] ? cd(0.0, 0.0) : g.z[j][i];
      const cd ref = excludedSc[i] ? cd(0.0, 0.0) : g.ref[j][i];
      r.constellation.push_back(z);
      r.reference.push_back(ref);
      r.errorVector.push_back(z - ref);
      if (excludedSc[i] || resourceMask[j][i])
        continue;
      const double e = std::norm(z - ref);
      const double p = std::norm(ref);
      symErr += e;
      symRef += p;
      if (e > symPeak) symPeak = e;
      if (!excluded[j]) {
        scErr[i] += e;
        scRef[i] += p;
        if (e > scPeak[i]) scPeak[i] = e;
        totErr += e;
        totRef += p;
        if (e > peak) { peak = e; r.peakEvmSymbol = static_cast<int>(j); r.peakEvmSubcarrier = i; }
        const int sl = slotOf(j);
        slotErr[sl] += e; slotRef[sl] += p; slotCount[sl] += 1.0;
        if (e > slotPeak[sl]) slotPeak[sl] = e;
        const int rb = i / 12;
        rbErr[rb] += e; rbRef[rb] += p; rbCount[rb] += 1.0;
        rbPower[rb] += std::norm(y[j][i]) * gain * gain;
      }
    }
    r.evmPerSymbol[j] = symRef > 0.0 ? 100.0 * std::sqrt(symErr / symRef) : 0.0;
    r.peakEvmPerSymbol[j] = symRef > 0.0 ? 100.0 * std::sqrt(symPeak * numIncludedSc / symRef) : 0.0;
  }
  for (int i = 0; i < numSc; ++i) {
    r.evmPerSubcarrier[i] = scRef[i] > 0.0 ? 100.0 * std::sqrt(scErr[i] / scRef[i]) : 0.0;
    r.peakEvmPerSubcarrier[i] = scRef[i] > 0.0 ? 100.0 * std::sqrt(scPeak[i] * numIncluded / scRef[i]) : 0.0;
  }
  for (int sl = 0; sl < r.numSlots; ++sl) {
    r.evmPerSlot[sl] = slotRef[sl] > 0.0 ? 100.0 * std::sqrt(slotErr[sl] / slotRef[sl]) : 0.0;
    r.peakEvmPerSlot[sl] = slotRef[sl] > 0.0 ? 100.0 * std::sqrt(slotPeak[sl] * slotCount[sl] / slotRef[sl]) : 0.0;
  }
  r.powerPerResourceBlockDb.assign(numRb, 0.0);
  r.evmPerResourceBlock.assign(numRb, 0.0);
  for (int rb = 0; rb < numRb; ++rb) {
    r.powerPerResourceBlockDb[rb] = rbCount[rb] > 0.0 ? 10.0 * std::log10(rbPower[rb] / rbCount[rb] + 1e-30) : 0.0;
    r.evmPerResourceBlock[rb] = rbRef[rb] > 0.0 ? 100.0 * std::sqrt(rbErr[rb] / rbRef[rb]) : 0.0;
  }
  const double refMeanPower = totRef / static_cast<double>(numIncluded * numIncludedSc);
  r.rmsEvmPercent = totRef > 0.0 ? 100.0 * std::sqrt(totErr / totRef) : 0.0;
  r.peakEvmPercent = refMeanPower > 0.0 ? 100.0 * std::sqrt(peak / refMeanPower) : 0.0;
  r.rmsEvmDb = r.rmsEvmPercent > 0.0 ? 20.0 * std::log10(r.rmsEvmPercent / 100.0)
                                     : -std::numeric_limits<double>::infinity();
  if (!burstEndDetected) {
    r.rmsEvmPercent = 999.0;
    r.peakEvmPercent = 999.0;
    r.rmsEvmDb = 20.0 * std::log10(r.rmsEvmPercent / 100.0);
  }
  r.channelEstimate = g.channel;

  // slot index of the first demodulated symbol counted from the capture start: slots are
  // 1 ms / 2^mu on average (the long CP falls into the first slot of every half subframe),
  // a start up to half a CP ahead of the nominal boundary still counts to that slot
  {
    const double slotSamples = 1e-3 * cfg.sampleRate / static_cast<double>(1 << cfg.numerology);
    const double margin = 0.5 * cpNormal * cfg.sampleRate / grid.gridRate;
    r.firstSlotIndex = static_cast<int>(std::floor((static_cast<double>(r.timingOffset) + margin) / slotSamples));
  }

  // channel type per symbol
  r.channelTypePerSymbol.assign(numSym, 1);
  for (size_t j = 0; j < numSym; ++j) if (excluded[j]) r.channelTypePerSymbol[j] = 0;
  for (size_t d = 0; d < r.dmrsSymbols.size(); ++d) r.channelTypePerSymbol[r.dmrsSymbols[d]] = 2;

  // common phase error: the tracked phase without the 90 degree grid ambiguity and without
  // the part common to all symbols (that is channel phase, absorbed by the equalizer)
  r.commonPhaseErrorPerSymbol.resize(numSym);
  {
    std::vector<double> wrapped(numSym, 0.0);
    double mean = 0.0;
    for (size_t j = 0; j < numSym; ++j) {
      double p = std::fmod(g.cpe[j], PI / 2.0);
      if (p > PI / 4.0) p -= PI / 2.0;
      if (p <= -PI / 4.0) p += PI / 2.0;
      wrapped[j] = p;
      if (!excluded[j]) mean += p;
    }
    mean /= static_cast<double>(numIncluded);
    double acc = 0.0;
    for (size_t j = 0; j < numSym; ++j) {
      r.commonPhaseErrorPerSymbol[j] = std::polar(1.0, wrapped[j] - mean);
      if (!excluded[j]) acc += (wrapped[j] - mean) * (wrapped[j] - mean);
    }
    r.commonPhaseErrorDeg = std::sqrt(acc / static_cast<double>(numIncluded)) * 180.0 / PI;
  }

  // blind DM-RS statistics: the reference sequence is QPSK with constant amplitude on a
  // comb (TS 38.211 6.4.1.1.3: type 1 every other subcarrier, type 2 two of six); the other
  // resource elements may be empty or carry data of the second CDM group. The comb is the
  // resource element set with the smallest power variation.
  if (!r.dmrsSymbols.empty()) {
    std::vector<std::vector<cd> > zd(r.dmrsSymbols.size(), std::vector<cd>(numSc));
    for (size_t d = 0; d < r.dmrsSymbols.size(); ++d) {
      const std::vector<cd>& yd = y[r.dmrsSymbols[d]];
      for (int i = 0; i < numSc; ++i) {
        zd[d][i] = std::abs(g.channel[i]) > 0.0 ? yd[i] / g.channel[i] : cd(0.0, 0.0);
      }
    }
    // candidate sets: all, comb type 1 (2 groups), comb type 2 (3 groups)
    struct Candidate { int modulo; int first; int width; };
    const Candidate candidates[] = { { 1, 0, 1 }, { 2, 0, 1 }, { 2, 1, 1 }, { 6, 0, 2 }, { 6, 2, 2 }, { 6, 4, 2 } };
    auto inSet = [&](const Candidate& c, int i) {
      const int m = i % c.modulo;
      return m >= c.first && m < c.first + c.width;
    };
    size_t best = 0;
    double bestCv = std::numeric_limits<double>::infinity();
    for (size_t c = 0; c < sizeof(candidates) / sizeof(candidates[0]); ++c) {
      double sum = 0.0, sum2 = 0.0;
      size_t cnt = 0;
      for (size_t d = 0; d < zd.size(); ++d) {
        for (int i = 0; i < numSc; ++i) {
          if (excludedSc[i] || !inSet(candidates[c], i)) continue;
          const double p = std::norm(zd[d][i]);
          sum += p; sum2 += p * p; ++cnt;
        }
      }
      if (cnt < 4 || sum <= 0.0) continue;
      const double mean = sum / cnt;
      const double var = std::max(0.0, sum2 / cnt - mean * mean);
      double cv = std::sqrt(var) / mean;
      if (c == 0) cv -= 0.05; // prefer the full set when it is as constant as any comb
      if (cv < bestCv) { bestCv = cv; best = c; }
    }
    const Candidate& comb = candidates[best];
    double err = 0.0, refPow = 0.0, pilotPow = 0.0;
    size_t pilots = 0;
    for (size_t d = 0; d < zd.size(); ++d) {
      std::vector<cd>& z = zd[d];
      // own 4th power common phase estimate on the comb
      cd fourth(0.0, 0.0);
      double a2 = 0.0;
      size_t na = 0;
      for (int i = 0; i < numSc; ++i) {
        if (excludedSc[i] || !inSet(comb, i)) continue;
        const cd z2 = z[i] * z[i];
        fourth += z2 * z2;
        a2 += std::norm(z[i]);
        ++na;
      }
      if (!na) continue;
      const cd rot = std::abs(fourth) > 0.0 ? std::polar(1.0, -(std::arg(fourth) - PI) / 4.0) : cd(1.0, 0.0);
      const double amp = std::sqrt(a2 / static_cast<double>(na) / 2.0);
      for (int i = 0; i < numSc; ++i) {
        if (excludedSc[i] || !inSet(comb, i)) continue;
        const cd v = z[i] * rot;
        const cd ref(v.real() >= 0.0 ? amp : -amp, v.imag() >= 0.0 ? amp : -amp);
        err += std::norm(v - ref);
        refPow += std::norm(ref);
        pilotPow += std::norm(v);
        ++pilots;
      }
    }
    r.dmrsEvmPercent = refPow > 0.0 ? 100.0 * std::sqrt(err / refPow) : 0.0;
    r.dmrsPowerDb = pilots ? 10.0 * std::log10(pilotPow / static_cast<double>(pilots) + 1e-30) : 0.0;
  }

  // channel impulse response and phase difference between neighbouring subcarriers
  {
    const std::chrono::steady_clock::time_point dftStart = std::chrono::steady_clock::now();
    std::vector<cd> h(g.channel);
    dft(h, true);
    if (profile) {
      const double milliseconds = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - dftStart).count();
      std::fprintf(stderr, "[demod-profile] channel-dft n=%d provider=%s ms=%.3f\n",
                   numSc, sExternalFft ? "external" : "builtin", milliseconds);
    }
    double maxMag = 0.0;
    for (size_t i = 0; i < h.size(); ++i) maxMag = std::max(maxMag, std::abs(h[i]));
    r.channelImpulseResponseDb.assign(numSc, 0.0);
    r.channelPhaseDifference.assign(numSc, 0.0);
    for (int i = 0; i < numSc; ++i) {
      r.channelImpulseResponseDb[i] = maxMag > 0.0 ? 20.0 * std::log10(std::abs(h[i]) / maxMag + 1e-30) : 0.0;
      if (i + 1 < numSc) r.channelPhaseDifference[i] = std::arg(g.channel[i + 1] * std::conj(g.channel[i]));
    }
  }

  // EVM equalizer spectral flatness (TS 38.101-1 6.4.2.4, normal conditions): range 1 is
  // more than 3 MHz away from both channel edges, range 2 is the rest of the channel
  {
    const double edgeHz = grid.carrierRb > 0 ? 0.5 * grid.carrierRb * 12.0 * scs
                        : (cfg.channelBandwidthMHz > 0 ? 0.5e6 * cfg.channelBandwidthMHz
                                                        : 0.5 * numSc * scs + std::fabs(k0 + 0.5 * numSc) * scs);
    double max1 = -1e300, min1 = 1e300, max2 = -1e300, min2 = 1e300;
    bool has1 = false, has2 = false;
    for (int i = 0; i < numSc; ++i) {
      if (excludedSc[i] || !(std::abs(g.channel[i]) > 0.0)) continue;
      const double f = (k0 + i) * scs;
      const double db = 20.0 * std::log10(std::abs(g.channel[i]));
      if (edgeHz - std::fabs(f) >= 3.0e6) { has1 = true; max1 = std::max(max1, db); min1 = std::min(min1, db); }
      else                                { has2 = true; max2 = std::max(max2, db); min2 = std::min(min2, db); }
    }
    r.flatnessRippleRange1Db = has1 ? max1 - min1 : 0.0;
    r.flatnessRippleRange2Db = has2 ? max2 - min2 : 0.0;
    r.flatnessMaxRange1MinRange2Db = (has1 && has2) ? max1 - min2 : 0.0;
    r.flatnessMaxRange2MinRange1Db = (has1 && has2) ? max2 - min1 : 0.0;
  }

  // I/Q impairments from the image leakage between the subcarrier pairs (k, -k).
  // Model per pair: Y[k] = a(f) S[k] + b(f) conj(S[-k]) with G = g exp(j phi) exp(-j 2 pi f tau),
  // a = (1 + G) / 2, b = (1 - G) / 2. Transmitter impairment model: a frequency error of the
  // device rotates signal and image alike, so the CFO correction leaves the image in place.
  //  * decision free: E[Y[k] Y[-k]] / E[|Y[k]|^2 + |Y[-k]|^2] = a b / (|a|^2 + |b|^2); works for
  //    any circular signal (DFT-s-OFDM, DM-RS symbols) and carries no grid ambiguity
  //  * decision directed (CP-OFDM): E[(z[k] - ref[k]) ref[-k]] / E[|ref[-k]|^2] = b / a is far
  //    more precise, but the blind receiver knows the constellation only up to a multiple of
  //    90 degrees, which flips the sign of the correlation: the per symbol part is undone with
  //    the tracked CPE, the common part is taken from the decision free estimate.
  // b(f) = b0 + j pi f tau along the subcarriers gives the timing skew.
  {
    double fw = 0.0, fwf = 0.0, fwff = 0.0, fwcc = 0.0, dw = 0.0, dwf = 0.0, dwff = 0.0;
    cd fwc(0.0, 0.0), fwfc(0.0, 0.0), dwc(0.0, 0.0), dwfc(0.0, 0.0);
    int fn = 0;
    for (int i = 0; i < numSc; ++i) {
      const int im = -2 * k0 - i;
      if (im < 0 || im >= numSc || im == i || excludedSc[i] || excludedSc[im]) continue;
      const double f = (k0 + i) * scs;
      cd fnum(0.0, 0.0), dnum(0.0, 0.0);
      double fden = 0.0, dden = 0.0;
      for (size_t j = 0; j < numSym; ++j) {
        fnum += y[j][i] * y[j][im];
        fden += std::norm(y[j][i]) + std::norm(y[j][im]);
        if (cfg.transformPrecoding || excluded[j]) continue;
        dnum += (g.z[j][i] - g.ref[j][i]) * g.ref[j][im] * std::polar(1.0, 2.0 * g.cpe[j]);
        dden += std::norm(g.ref[j][im]);
      }
      if (fden > 0.0) {
        const cd c = fnum / fden;
        fw += fden; fwf += fden * f; fwff += fden * f * f; fwc += fden * c; fwfc += fden * f * c;
        fwcc += fden * std::norm(c); ++fn;
      }
      if (dden > 0.0) {
        const cd b = dnum / dden;
        dw += dden; dwf += dden * f; dwff += dden * f * f; dwc += dden * b; dwfc += dden * f * b;
      }
    }
    const double fdet = fw * fwff - fwf * fwf;
    // precoded symbols are DFT spread: the pairs (k, -k) share the same data and the decision
    // free estimate does not average down, so no estimate is given for DFT-s-OFDM
    if (!cfg.transformPrecoding && fw > 0.0 && std::fabs(fdet) > 1e-12 * fw * fwff) {
      const cd fslope = (fw * fwfc - fwf * fwc) / fdet;
      const cd c0 = (fwc - fslope * fwf) / fw;
      // scatter of the per pair estimates around the fitted line -> variance of c0, so that a
      // c0 that is nothing but estimation noise is not used to decide anything
      const double ssr = std::max(0.0, fwcc - (std::conj(c0) * fwc + std::conj(fslope) * fwfc).real());
      const double varC0 = fn > 2 ? ssr / static_cast<double>(fn - 2) * (fwff / fdet) : 0.0;
      cd G(1.0, 0.0);
      cd slope = fslope;
      const double ddet = dw * dwff - dwf * dwf;
      if (dw > 0.0 && std::fabs(ddet) > 1e-12 * dw * dwff) {
        cd dslope = (dw * dwfc - dwf * dwc) / ddet;
        cd b0 = (dwc - dslope * dwf) / dw;
        // E[Y[k] Y[-k]] = a(f) b(-f) + b(f) a(-f) cancels exactly for a pure timing skew: then
        // the decision free estimate carries no sign information and neither does the capture -
        // a 90 degree rotation of the constellation maps skew tau onto -tau. Resolve the sign
        // from c0 only while it is significant, otherwise report the magnitude.
        if (std::norm(c0) > 9.0 * varC0) {
          if ((std::conj(b0) * c0).real() < 0.0) { b0 = -b0; dslope = -dslope; }
        } else if (dslope.imag() < 0.0) {
          b0 = -b0; dslope = -dslope;
        }
        G = (cd(1.0, 0.0) - b0) / (cd(1.0, 0.0) + b0);
        slope = dslope;
      } else {
        // a b / (|a|^2 + |b|^2) = (1 - G^2) / (2 (1 + |G|^2)), solved with |G| ~ 1 and refined once
        cd G2 = cd(1.0, 0.0) - 4.0 * c0;
        G2 = cd(1.0, 0.0) - 2.0 * c0 * (1.0 + std::abs(G2));
        G = std::sqrt(G2);
      }
      r.iqGainImbalanceDb = 20.0 * std::log10(std::abs(G));
      r.iqQuadratureErrorDeg = std::arg(G) * 180.0 / PI;
      r.iqTimingSkewSec = slope.imag() / PI;
    }
  }

  r.fftSize = N;
  r.resampleRate = grid.gridRate;
  r.numResourceBlocks = grid.numRb;
  r.carrierResourceBlocks = grid.carrierRb;
  r.firstSubcarrierIndex = k0;
  r.numSymbols = static_cast<int>(numSym);
  r.ok = true;
  profileMark("complete");
  if (trace) {
    std::fprintf(stderr, "[demod-trace] core-exit status=ok fft=%d symbols=%d evm=%.6f peak=%.6f\n",
                 r.fftSize, r.numSymbols, r.rmsEvmPercent, r.peakEvmPercent);
  }
  return r;
}

std::vector<std::complex<float> > generateTestSignal(const Config& cfg, int numSymbols,
    int leadingSamples, unsigned seed, std::vector<cd>* referenceOut)
{
  std::vector<std::complex<float> > out;
  Grid grid;
  std::string error;
  if (!resolveGrid(cfg, grid, error)) return out;
  const int N = grid.fftSize;
  const int numSc = 12 * grid.numRb;
  const int k0 = grid.firstSubcarrier;
  const Modulation mod = cfg.modulation == MOD_AUTO ? MOD_QPSK : static_cast<Modulation>(cfg.modulation);
  const double s = qamScale(mod);
  const int maxLevel = qamMaxLevel(mod);
  const int levels = maxLevel + 1; // number of odd levels per axis
  const int cpNormal = cfg.extendedCp ? extendedCpLength(N) : normalCpLength(N);
  const int cpLong = cfg.extendedCp ? cpNormal : longCpLength(N, cfg.numerology);
  const int period = cfg.extendedCp ? 0 : (7 << cfg.numerology);
  const std::vector<int> dmrsPos = dmrsSymbolsInSlot(cfg.dmrs);
  const int perSlot = symbolsPerSlot(cfg.extendedCp);

  unsigned state = seed ? seed : 1u;
  auto nextLevel = [&]() {
    state = state * 1664525u + 1013904223u;
    return -maxLevel + 2 * static_cast<int>((state >> 16) % static_cast<unsigned>(levels));
  };

  std::vector<cd> grid_time;
  grid_time.reserve(static_cast<size_t>(numSymbols) * (N + cpLong));
  if (referenceOut) referenceOut->clear();
  const double scale = 1.0 / std::sqrt(static_cast<double>(N));
  const double dftScale = 1.0 / std::sqrt(static_cast<double>(numSc));
  for (int j = 0; j < numSymbols; ++j) {
    const bool isDmrs = cfg.dmrs.exclude &&
        std::find(dmrsPos.begin(), dmrsPos.end(), j % perSlot) != dmrsPos.end();
    std::vector<cd> d(numSc);
    for (int i = 0; i < numSc; ++i) {
      if (isDmrs) {
        // pseudo random QPSK reference sequence (noise like in time domain, as a Gold
        // sequence based DM-RS), not part of the data grid
        state = state * 1664525u + 1013904223u;
        d[i] = std::polar(1.0, PI / 4.0 + PI / 2.0 * static_cast<double>((state >> 16) % 4u));
      } else {
        d[i] = cd(nextLevel() * s, nextLevel() * s);
        if (referenceOut) referenceOut->push_back(d[i]);
      }
    }
    if (cfg.transformPrecoding && !isDmrs) {
      dft(d, false);
      for (int i = 0; i < numSc; ++i) d[i] *= dftScale;
    }
    std::vector<cd> X(N, cd(0.0, 0.0));
    for (int i = 0; i < numSc; ++i) {
      if (cfg.dcPunctured && !cfg.transformPrecoding && k0 + i == 0) continue;
      X[((k0 + i) % N + N) % N] = d[i];
    }
    fft(X, true);
    const int cp = (period > 0 && (j % period) == 0) ? cpLong : cpNormal;
    for (int k = N - cp; k < N; ++k) grid_time.push_back(X[k] * scale);
    for (int k = 0; k < N; ++k) grid_time.push_back(X[k] * scale);
  }

  std::vector<cd> sig = grid.resampled ? resample(grid_time, grid.gridRate, cfg.sampleRate) : grid_time;
  if (cfg.carrierOffsetHz != 0.0) {
    const double w = 2.0 * PI * cfg.carrierOffsetHz / cfg.sampleRate;
    for (size_t m = 0; m < sig.size(); ++m) {
      sig[m] *= std::polar(1.0, w * static_cast<double>(m + leadingSamples));
    }
  }
  out.assign(static_cast<size_t>(leadingSamples), std::complex<float>(0.0f, 0.0f));
  for (size_t m = 0; m < sig.size(); ++m) {
    out.push_back(std::complex<float>(static_cast<float>(sig[m].real()), static_cast<float>(sig[m].imag())));
  }
  return out;
}

}
