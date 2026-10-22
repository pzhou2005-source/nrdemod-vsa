#include "nbiot/NbIotCore.hpp"

#include <algorithm>
#include <cmath>

namespace nbiotdemod {
namespace {
nrdemod::Result demodulateDownlinkFrame(const float* iq, size_t sampleCount, nrdemod::Config config)
{
  using Complex = std::complex<double>;
  nrdemod::Result result;
  const double rate = 1920000.0;
  const double inputRate = config.sampleRate;
  const double pi = std::acos(-1.0);
  const int symbols = config.maxSymbols;
  if (!std::isfinite(inputRate) || inputRate <= 0.0) {
    result.error = "NB-IoT requires a finite positive sample rate.";
    return result;
  }
  if (symbols < 14 || symbols % 14 != 0 || symbols > 140) {
    result.error = "NB-IoT frame mode requires 14 to 140 symbols in complete subframes.";
    return result;
  }
  const double durationSamples = inputRate * (0.020 + symbols / 14000.0);
  const size_t wanted = durationSamples >= sampleCount ? sampleCount : static_cast<size_t>(durationSamples);
  std::vector<Complex> samples(wanted);
  for (size_t point = 0; point < wanted; ++point) {
    samples[point] = Complex(iq[2 * point], iq[2 * point + 1]);
    if (!std::isfinite(samples[point].real()) || !std::isfinite(samples[point].imag())) {
      result.error = "Nonfinite NB-IoT frame samples.";
      return result;
    }
  }
  if (inputRate != rate)
    samples = nrdemod::resample(samples, inputRate, rate);
  const int frameLength = symbols * 137 + (symbols + 6) / 7;
  const int lead = 412;
  if (samples.size() < static_cast<size_t>(frameLength)) {
    result.error = "Capture is too short for the requested NB-IoT subframes.";
    return result;
  }
  Complex dc(0.0, 0.0);
  for (size_t point = 0; point < samples.size(); ++point)
    samples[point] *= std::polar(1.0, -2.0 * pi * config.carrierOffsetHz * point / rate);
  const int signs[] = {1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1};
  std::vector<Complex> reference;
  for (int symbol = 0; symbol < 11; ++symbol) {
    std::vector<Complex> spectrum(128);
    for (int carrier = 0; carrier < 11; ++carrier)
      spectrum[(carrier - 6 + 128) % 128] = static_cast<double>(signs[symbol]) *
          std::polar(1.0, -5.0 * pi * carrier * (carrier + 1) / 11.0);
    nrdemod::fft(spectrum, true);
    const int prefix = symbol == 4 ? 10 : 9;
    for (int point = -prefix; point < 128; ++point)
      reference.push_back(spectrum[(point + 128) % 128] * std::polar(1.0, pi * point / 128.0) / 128.0);
  }
  size_t fftSize = 1;
  while (fftSize < samples.size() + reference.size())
    fftSize *= 2;
  std::vector<Complex> input(fftSize);
  std::copy(samples.begin(), samples.end(), input.begin());
  std::vector<double> power(samples.size() + 1, 0.0);
  for (size_t point = 0; point < samples.size(); ++point)
    power[point + 1] = power[point] + std::norm(samples[point]);
  std::vector<Complex> convolution(fftSize);
  double referencePower = 0.0;
  for (size_t point = 0; point < reference.size(); ++point) {
    convolution[reference.size() - 1 - point] = std::conj(reference[point]);
    referencePower += std::norm(reference[point]);
  }
  nrdemod::fft(input, false);
  nrdemod::fft(convolution, false);
  for (size_t point = 0; point < fftSize; ++point)
    convolution[point] *= input[point];
  nrdemod::fft(convolution, true);
  const auto correlationAt = [&](size_t point) {
    const double denominator = referencePower * (power[point + reference.size()] - power[point]);
    return denominator > 0.0 ? std::norm(convolution[point + reference.size() - 1]) /
        (denominator * fftSize * fftSize) : 0.0;
  };
  double maximum = 0.0;
  size_t start = 0;
  for (size_t point = lead; point + reference.size() <= samples.size() &&
      point - lead + frameLength <= samples.size(); ++point) {
    const double rho = correlationAt(point);
    if (rho > maximum) {
      maximum = rho;
      start = point - lead;
    }
  }
  if (maximum < 0.5) {
    result.error = "No valid NPSS with the requested complete NB-IoT subframes.";
    return result;
  }
  for (size_t point = lead + 1; point + reference.size() < samples.size() &&
      point - lead + frameLength <= samples.size(); ++point) {
    const double rho = correlationAt(point);
    if (rho >= maximum * 0.99 && rho >= correlationAt(point - 1) && rho > correlationAt(point + 1)) {
      start = point - lead;
      maximum = rho;
      break;
    }
  }
  std::vector<std::complex<float>> frame;
  if (config.removeIqOffset) {
    for (size_t point = start; point < start + lead; ++point)
      dc += samples[point] * std::polar(1.0, 2.0 * pi * config.carrierOffsetHz * point / rate);
    dc /= static_cast<double>(lead);
  }
  frame.reserve(frameLength);
  config.excludedSymbols.clear();
  config.decisionRotations.assign(symbols * 12, Complex(1.0, 0.0));
  config.excludedResources.assign(symbols * 12, false);
  size_t position = start;
  for (int symbol = 0; symbol < symbols; ++symbol) {
    const int prefix = symbol % 7 == 0 ? 10 : 9;
    for (int point = -prefix; point < 128; ++point) {
      const size_t index = position + prefix + point;
      const Complex offset = dc * std::polar(1.0, -2.0 * pi * config.carrierOffsetHz * index / rate);
      frame.emplace_back((samples[index] - offset) * std::polar(1.0, -pi * point / 128.0));
    }
    if (symbol % 14 < 3)
      config.excludedSymbols.push_back(symbol);
    if (symbol >= 3 && symbol < 14) {
      for (int carrier = 0; carrier < 11; ++carrier) {
        const Complex ideal = static_cast<double>(signs[symbol - 3]) *
            std::polar(1.0, -5.0 * pi * carrier * (carrier + 1) / 11.0);
        config.decisionRotations[symbol * 12 + carrier] = std::polar(1.0, pi / 4.0) * std::conj(ideal);
      }
      config.excludedResources[symbol * 12 + 11] = true;
    }
    position += prefix + 128;
  }
  config.sampleRate = rate;
  config.fftSize = 128;
  config.carrierOffsetHz = 0.0;
  config.burstSearch = false;
  config.timingSearch = false;
  config.removeIqOffset = false;
  config.modulation = nrdemod::MOD_QPSK;
  config.dmrs.enabled = false;
  result = nrdemod::demodulate(frame.data(), frame.size(), config);
  result.timingOffset = static_cast<int>(std::lround(start * inputRate / rate));
  result.syncCorrelation = std::sqrt(maximum);
  return result;
}
}

nrdemod::Result demodulate(const float* iq, size_t sampleCount, nrdemod::Config config,
    bool uplink, bool downlinkFrame)
{
  if (downlinkFrame) {
    if (!uplink)
      return demodulateDownlinkFrame(iq, sampleCount, config);
    nrdemod::Result result;
    result.error = "NPSS frame synchronization is only available for NB-IoT downlink.";
    return result;
  }
  nrdemod::Result result = nrdemod::demodulateInterleaved(iq, sampleCount, config);
  if (result.ok && uplink) {
    for (int symbol = 0; symbol < result.numSymbols; ++symbol)
      if ((symbol - result.slotStartSymbol + 14) % 7 == 3)
        config.excludedSymbols.push_back(symbol);
    result = nrdemod::demodulateInterleaved(iq, sampleCount, config);
  }
  return result;
}
}