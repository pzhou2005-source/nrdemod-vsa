/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11ad/Wifi11adOfdmCore.hpp"

#include <algorithm>
#include <cmath>

namespace wifi11addemod {

namespace {

typedef std::complex<double> cd;

// Identical windowed-power burst detector used by every other native WiFi core in this
// codebase (wifi/WifiOfdmCore.cpp, wifi11ac/Wifi11acOfdmCore.cpp, etc.) - the one piece of
// this core that needs no knowledge of the 802.11ad-specific Golay STF/CE sequence content.
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

bool finite(const cd& value)
{
  return std::isfinite(value.real()) && std::isfinite(value.imag());
}

cd derotate(const cd& value, std::size_t chip, double frequencyRadians)
{
  static const cd rotations[] = {cd(1, 0), cd(0, -1), cd(-1, 0), cd(0, 1)};
  return value * rotations[chip % 4] * std::polar(1.0, -frequencyRadians * chip);
}

void measureData(const std::vector<cd>& iq, const Config& config, Result& result)
{
  result.dataError = "SC DATA requires 1.76 GSa/s and MCS1-9; OFDM/RRC are unsupported";
  if (!std::isfinite(config.sampleRate) || std::fabs(config.sampleRate / 1.76e9 - 1.0) > 1e-6 || config.dataMcs < 1 ||
      config.dataMcs > 9)
    return;
  result.dataError = "Supply an independent 128-chip +/- STF reference and explicit training/DATA layout";
  if (config.trainingReference.size() != 128 || config.trainingOffsetSamples < 0 || config.dataOffsetSamples < 0 ||
      config.dataBlockCount <= 0)
    return;
  for (char chip : config.trainingReference) {
    if (chip != '+' && chip != '-')
      return;
  }
  const std::size_t trainingStart = static_cast<std::size_t>(config.trainingOffsetSamples);
  const std::size_t dataStart = static_cast<std::size_t>(config.dataOffsetSamples);
  result.dataError = "Truncated or overlapping SC training/DATA fields";
  if (trainingStart > iq.size() || iq.size() - trainingStart < 512 || dataStart < trainingStart + 512 ||
      dataStart > iq.size() || static_cast<std::size_t>(config.dataBlockCount) > (iq.size() - dataStart) / 512)
    return;

  result.dataError = "Invalid or nonfinite SC training";
  cd repeatCorrelation(0, 0);
  for (std::size_t chip = 0; chip < 384; ++chip) {
    if (!finite(iq[trainingStart + chip]) || !finite(iq[trainingStart + chip + 128]))
      return;
    repeatCorrelation += std::conj(iq[trainingStart + chip]) * iq[trainingStart + chip + 128];
  }
  if (!finite(repeatCorrelation) || std::abs(repeatCorrelation) < 1e-24)
    return;
  const double frequencyRadians = std::arg(repeatCorrelation) / 128.0;
  cd referenceCorrelation(0, 0);
  double trainingPower = 0.0;
  for (std::size_t chip = 0; chip < 512; ++chip) {
    const cd sample = derotate(iq[trainingStart + chip], chip, frequencyRadians);
    const double reference = config.trainingReference[chip % 128] == '+' ? 1.0 : -1.0;
    referenceCorrelation += sample * reference;
    trainingPower += std::norm(sample);
  }
  if (!finite(referenceCorrelation) || !std::isfinite(trainingPower) || trainingPower < 1e-24)
    return;
  result.trainingRho = std::abs(referenceCorrelation) / std::sqrt(512.0 * trainingPower);
  result.dataError = "SC training does not match the supplied reference";
  if (!std::isfinite(result.trainingRho) || result.trainingRho < 0.9)
    return;
  const cd channel = referenceCorrelation / 512.0;
  if (std::norm(channel) < 1e-24)
    return;
  result.frequencyErrorHz = frequencyRadians * config.sampleRate / (2.0 * std::acos(-1.0));

  result.dataError = "Nonfinite SC DATA or EVM accumulation";
  double errorPower = 0.0;
  for (int block = 0; block < config.dataBlockCount; ++block) {
    const std::size_t blockStart = dataStart + static_cast<std::size_t>(block) * 512;
    for (std::size_t chip = 64; chip < 512; ++chip) {
      const std::size_t position = blockStart + chip;
      if (!finite(iq[position]))
        return;
      const cd equalized = derotate(iq[position], position - trainingStart, frequencyRadians) / channel;
      cd reference(equalized.real() >= 0.0 ? 1.0 : -1.0, 0.0);
      if (config.dataMcs >= 6 && std::fabs(equalized.imag()) > std::fabs(equalized.real()))
        reference = cd(0.0, equalized.imag() >= 0.0 ? 1.0 : -1.0);
      errorPower += std::norm(equalized - reference);
    }
  }
  const double symbolCount = static_cast<double>(config.dataBlockCount) * 448.0;
  if (!std::isfinite(errorPower) || symbolCount > 2147483647.0)
    return;
  result.evmRmsData = std::sqrt(errorPower / symbolCount);
  if (!std::isfinite(result.evmRmsData))
    return;
  result.numDataSymbols = static_cast<int>(symbolCount);
  result.dataError.clear();
  result.dataEvmValid = true;
}

}

Config::Config()
    : burstSearchThresholdDb(15.0), measureDataEvm(false), sampleRate(1.76e9), trainingOffsetSamples(-1),
      dataOffsetSamples(-1), dataBlockCount(0), dataMcs(1)
{
}

Result::Result()
    : ok(false), burstFound(false), burstStart(0), burstLength(0), dataEvmValid(false),
      dataError("SC DATA EVM is disabled"), trainingRho(0.0), frequencyErrorHz(0.0), evmRmsData(0.0), numDataSymbols(0)
{
}

Result demodulate(const std::vector<cd>& iq, const Config& config)
{
  Result result;
  if (iq.empty()) {
    result.error = "empty capture";
    return result;
  }

  int burstStart = 0, burstLength = static_cast<int>(iq.size());
  if (!findBurst(iq, config.burstSearchThresholdDb, burstStart, burstLength)) {
    result.error = "burst not found";
    return result;
  }
  result.burstFound = true;
  result.burstStart = burstStart;
  result.burstLength = burstLength;
  result.ok = true;
  if (config.measureDataEvm)
    measureData(iq, config, result);
  return result;
}

}
