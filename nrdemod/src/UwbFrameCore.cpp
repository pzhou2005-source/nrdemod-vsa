#include "uwb/UwbHrpCore.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include "../third_party/ldpc/LdpcCode.cpp"

#ifdef _WIN32
#include <windows.h>
#include <bcrypt.h>
#endif

namespace uwbdemod {
namespace {

std::vector<int> preambleCode(int index)
{
  static const char* codes[] = {
    "-0000+0-0+++0+-000+-+++00-+0-00",
    "0+0+-0+0+000-++0-+---00+00++000",
    "-+0++000-+-++00++0+00-0000-0+0-",
    "0000+-00-00-++++0+-+000+0-0++0-",
    "-0+-00+++-+000-+0+++0-0+0000-00",
    "++00+00---+-0++-000+0+0-+0+0000",
    "+0000+-0+0+00+000+0++---0-+00-+",
    "0+00-0-0++0000--+00-+0++-++0+00",
    "+00+000-0--00--+0+0+00-+-++0+0000++-000+00-00--0-+0+0--0-+++0++000+-0+00-0++-0+++00-+00+0+0-0++-+--+000000+00000-+0000-0-000--+",
    "++00+0-+00+00+000000-000-00--000-0+-+0-0+-0-+00000+-00++0-0+00--+00++-+0+-0+0000-0-0-0-++-+0+00+0+000-+0+++000----+++0000+++0--",
    "-+-0000+00--00000-0+0+0+-0+00+00+0-00-+++00+000-+0+0-0000+++++-+0+--0+-0++--0-000+0-+00+0+----000-000000-+00+-0++000++-00++-0-0",
    "-+0++000000-0+0-+0---+-++00-+0++0+0+0+000-00-00-+00+-++000-+-0-++0-0++++0-00-0++00+0+00++-00+000+-000-0--+0000-0000--0+00000+--",
    "+000--0000--++0-++++0-0++0+0-00-+0++00++-0++0+-+0-00+00-0--000-+-00+0000-0++-00000+-0-000000-00-+-++-+000-0+0+0+++-00--00+0+000",
    "+000++0-0+0-00+-0-+0-00+0+0000+0+-0000++00+0+++++-+0-0+-0--+0++--000---0+000+0+0-+-000000+-+-0--00++000-00+00++-00--++-00-00000",
    "0+-00+0-000-++0000---++000+0+-0-+00-+000--0-00--0--+++-+0-++00+-++0+00000+0-0+++-00+00+000-0000+00--+0++0+0+0-00-0-+-0+0++00000",
    "++0000+000+00+--0+-++0-000--00+-0+00++000+++00+0+0-0-+-0-0+00+00+0++----+00++--+0+-0--+000000-0-0000-+0--00+00000+-++000-0-+0+0",
    "+--000-0-0000+-00000+000000+--+-++0-0+0+00+-00+++0-++0-00+0-+000++0+++-0--0+0+-0--00-00+000-++0000+0++-+-00+0+0+--00--0-000+00+",
    "--0+++0000+++----000+++0+-000+0+00+0+-++-0-0-0-0000+0-+0+-++00+--00+0-0++00-+00000+-0-+0-0+-+0-000--00-000-000000+00+00+-0+00++",
    "-0-++00-++000++0-+00+-000000-000----+0+00+-0+000-0--++0-+0--+0+-+++++0000-0+0+-000+00+++-00-0+00+00+0-+0+0+0-00000--00+0000-+-0",
    "--+00000+0--0000-0000+--0-000-+000+00-++00+0+00++0-00-0++++0-0++-0-+-000++-+00+-00-00-000+0+0+0++0+-00++-+---0+-0+0-000000++0+-",
    "+0+00--00-+++0+0+0-000+-++-+-00-000000-0-+00000-++0-0000+00-+-000--0-00+00-0+-+0++0-++00++0+-00-0+0++0-0++++-0++--0000--000+000",
    "0-00-++--00-++00+00-000++00--0-+-+000000-+-0+0+000+0---000--++0+--0-+0-0+-+++++0+00++0000-+0+0000+0+00-0+-0-+00-0+0-0++000+0000",
    "000++0+0-+-0-00-0+0+0++0+--00+0000-000+00+00-+++0-0+00000+0++-+00++-0+-+++--0--00-0--000+-00+-0-+0+000++---0000++-000-0+00-+000",
    "+0+-0-000++-+00000+00--0+-0000-0-000000+--0-+0+--++00+----++0+00+00+0-0-+-0-0+0+00+++000++00+0-+00--000-0++-+0--+00+000+0000++0",
    "-0++++--+--+-++++-+---++--++++++-++-+00+--+0--+0+++++--+++--0--0+-+----0-+-+-+0+--++-+-+++0",
    "++0+-+---+++++-+-++--+--++---+-0+++0-++++-+0+0--0+-++-++++++--+-++00+++--0+-----+-0+-+-+---",
    "+++--+++---+-+-0-+--0++-+-+0-++++++++++--+--++-++0++-+-+--+--+++---0-+++-0+00---++-+--0-++0",
    "+++++--+++-++---+-+--0++---+-+0+-----+0+++--+-+--+-++-++++0-0-++00+-+++-++-+0-+0--+---+++0+",
    "+-0--+-++--0++00++-++-----++++++-0+--+-++--+-+++--+++++++-+++0+-+-0-0-++---+0---++0-+-+-++-",
    "-++0--0++-00--++-++-+--+++++---+++-+-0+-+-+0+++++-+++-+++-+0------++-++-+0----+-+-0+0+-+++-",
    "-+-++0+++--++-0++00--++--+++-+------0+++-+++++--+-++---+-+---+-+0-+--0+0+-++-++0+-+--0+++++",
    "-+++++++++++---+-++--++00-+-+0-+-0-++--+++-+++0--0++-+-+-0---++-0----++++-+-0+0-+-++-++--+-"};
  if (index < 1 || index > 32) throw std::runtime_error("invalid UWB preamble code index");
  std::vector<int> decodedCode;
  for (const char* symbol = codes[index - 1]; *symbol; ++symbol)
    decodedCode.push_back(*symbol == '+' ? 1 : (*symbol == '-' ? -1 : 0));
  return decodedCode;
}

std::vector<int> spreadingBits(const std::vector<int>& code, int count)
{
  std::vector<int> bits;
  for (int value : code) if (value != 0 && bits.size() < 15) bits.push_back(value > 0);
  for (int index = 15; index < count + 15; ++index)
    bits.push_back(bits[index - 15] ^ bits[index - 14]);
  return std::vector<int>(bits.begin() + 15, bits.end());
}

int parity(int value)
{
  value ^= value >> 4;
  value ^= value >> 2;
  value ^= value >> 1;
  return value & 1;
}

std::vector<int> decodeConvolutional(const std::vector<std::array<int, 2>>& words,
    int constraintLength = 3, bool terminated = false)
{
  const int states = 1 << (constraintLength - 1);
  std::vector<int> costs(states, 1000000);
  costs[0] = 0;
  std::vector<std::vector<int>> predecessors(words.size(), std::vector<int>(states));
  for (size_t index = 0; index < words.size(); ++index) {
    std::vector<int> next(states, 1000000);
    for (int state = 0; state < states; ++state) for (int bit = 0; bit < 2; ++bit) {
      const int reg = (bit << (constraintLength - 1)) | state;
      const int target = (bit << (constraintLength - 2)) | (state >> 1);
      const int distance = (parity(reg & (constraintLength == 3 ? 2 : 0133)) != words[index][0]) +
          (parity(reg & (constraintLength == 3 ? 5 : 0171)) != words[index][1]);
      if (costs[state] + distance < next[target]) {
        next[target] = costs[state] + distance;
        predecessors[index][target] = state;
      }
    }
    costs = next;
  }
  int state = terminated ? 0 : static_cast<int>(std::min_element(costs.begin(), costs.end()) - costs.begin());
  std::vector<int> decoded(words.size());
  for (size_t index = words.size(); index > 0; --index) {
    decoded[index - 1] = state >> (constraintLength - 2);
    state = predecessors[index - 1][state];
  }
  return decoded;
}

bool checkHeader(std::vector<int>& bits)
{
  const int addresses[19] = {3,5,6,7,9,10,11,12,13,14,15,17,18,0,16,8,4,2,1};
  int syndrome = 0, total = 0;
  for (int index = 0; index < 19; ++index) if (bits[index]) {
    syndrome ^= addresses[index];
    total ^= 1;
  }
  if (syndrome && !total) return false;
  if (total) {
    const auto found = std::find(addresses, addresses + 19, syndrome);
    if (found == addresses + 19) return false;
    bits[found - addresses] ^= 1;
  }
  return true;
}

std::vector<std::array<int, 2>> encodeConvolutional(const std::vector<int>& bits, int constraintLength)
{
  std::vector<std::array<int, 2>> words;
  int state = 0;
  for (int bit : bits) {
    const int reg = (bit << (constraintLength - 1)) | state;
    words.push_back({{parity(reg & (constraintLength == 3 ? 2 : 0133)),
      parity(reg & (constraintLength == 3 ? 5 : 0171))}});
    state = (bit << (constraintLength - 2)) | (state >> 1);
  }
  return words;
}

int gfMultiply(int left, int right)
{
  int product = 0;
  for (; right; right >>= 1) {
    if (right & 1) product ^= left;
    left <<= 1;
    if (left & 64) left ^= 0x43;
  }
  return product;
}

int gfPower(int value, int exponent)
{
  int result = 1;
  for (; exponent; exponent >>= 1, value = gfMultiply(value, value))
    if (exponent & 1) result = gfMultiply(result, value);
  return result;
}

int gfDivide(int numerator, int denominator)
{
  if (!denominator) throw std::runtime_error("singular Reed-Solomon equation");
  return gfMultiply(numerator, gfPower(denominator, 62));
}

std::array<int, 8> syndromes(const std::array<int, 63>& word)
{
  std::array<int, 8> values = {};
  for (int root = 0; root < 8; ++root) {
    const int point = gfPower(2, root + 1);
    for (int symbol : word) values[root] = gfMultiply(values[root], point) ^ symbol;
  }
  return values;
}

int correctReedSolomon(std::array<int, 63>& word)
{
  const auto syndrome = syndromes(word);
  if (std::all_of(syndrome.begin(), syndrome.end(), [](int value) { return value == 0; })) return 0;
  std::array<int, 9> locator = {}, previous = {};
  locator[0] = previous[0] = 1;
  int degree = 0, shift = 1, last = 1;
  for (int step = 0; step < 8; ++step) {
    int discrepancy = syndrome[step];
    for (int index = 1; index <= degree; ++index)
      discrepancy ^= gfMultiply(locator[index], syndrome[step - index]);
    if (!discrepancy) { ++shift; continue; }
    const auto saved = locator;
    const int scale = gfDivide(discrepancy, last);
    for (int index = 0; index + shift <= 8; ++index)
      locator[index + shift] ^= gfMultiply(scale, previous[index]);
    if (2 * degree <= step) {
      degree = step + 1 - degree;
      previous = saved;
      last = discrepancy;
      shift = 1;
    } else ++shift;
  }
  if (degree < 1 || degree > 4) return -1;
  std::vector<int> positions;
  for (int position = 0; position < 63; ++position) {
    const int inverse = gfPower(2, (position + 1) % 63);
    int value = locator[degree];
    for (int index = degree - 1; index >= 0; --index) value = gfMultiply(value, inverse) ^ locator[index];
    if (!value) positions.push_back(position);
  }
  if (static_cast<int>(positions.size()) != degree) return -1;
  std::array<std::array<int, 5>, 4> equations = {};
  for (int row = 0; row < degree; ++row) {
    for (int col = 0; col < degree; ++col)
      equations[row][col] = gfPower(2, ((62 - positions[col]) * (row + 1)) % 63);
    equations[row][degree] = syndrome[row];
  }
  for (int pivot = 0; pivot < degree; ++pivot) {
    int row = pivot;
    while (row < degree && !equations[row][pivot]) ++row;
    if (row == degree) return -1;
    std::swap(equations[row], equations[pivot]);
    const int divisor = equations[pivot][pivot];
    for (int col = pivot; col <= degree; ++col) equations[pivot][col] = gfDivide(equations[pivot][col], divisor);
    for (int other = 0; other < degree; ++other) if (other != pivot) {
      const int scale = equations[other][pivot];
      for (int col = pivot; col <= degree; ++col)
        equations[other][col] ^= gfMultiply(scale, equations[pivot][col]);
    }
  }
  for (int index = 0; index < degree; ++index) word[positions[index]] ^= equations[index][degree];
  const auto checked = syndromes(word);
  return std::all_of(checked.begin(), checked.end(), [](int value) { return value == 0; }) ? degree : -1;
}

#ifdef _WIN32
std::vector<unsigned char> parseHex(const std::string& text)
{
  if (text.size() != 32 || text.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)
    throw std::runtime_error("STS key and IV must each contain 32 hexadecimal characters");
  std::vector<unsigned char> bytes(16);
  for (int index = 0; index < 16; ++index) bytes[index] = static_cast<unsigned char>(std::stoul(text.substr(2 * index, 2), nullptr, 16));
  return bytes;
}
#endif

std::vector<int> stsBits(const FrameConfig& frame, int bitCount)
{
#ifdef _WIN32
  auto keyBytes = parseHex(frame.stsKeyHex);
  auto counter = parseHex(frame.stsIvHex);
  BCRYPT_ALG_HANDLE algorithm = nullptr;
  BCRYPT_KEY_HANDLE key = nullptr;
  if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_AES_ALGORITHM, nullptr, 0) < 0)
    throw std::runtime_error("cannot open native AES provider");
  NTSTATUS status = BCryptSetProperty(algorithm, BCRYPT_CHAINING_MODE,
      reinterpret_cast<PUCHAR>(const_cast<wchar_t*>(BCRYPT_CHAIN_MODE_ECB)), sizeof(BCRYPT_CHAIN_MODE_ECB), 0);
  if (status >= 0) status = BCryptGenerateSymmetricKey(algorithm, &key, nullptr, 0, keyBytes.data(), 16, 0);
  std::vector<int> output;
  for (int block = 0; block < (bitCount + 127) / 128 && status >= 0; ++block) {
    unsigned char encrypted[16];
    ULONG count = 0;
    status = BCryptEncrypt(key, counter.data(), 16, nullptr, nullptr, 0, encrypted, 16, &count, 0);
    if (status >= 0 && count == 16)
      for (unsigned char value : encrypted) for (int bit = 7; bit >= 0; --bit) output.push_back((value >> bit) & 1);
    else status = static_cast<NTSTATUS>(-1);
    for (int byte = 15; byte >= 0; --byte) if (++counter[byte]) break;
  }
  if (key) BCryptDestroyKey(key);
  BCryptCloseAlgorithmProvider(algorithm, 0);
  if (status < 0) throw std::runtime_error("native AES STS generation failed");
  return output;
#else
  (void)frame;
  (void)bitCount;
  throw std::runtime_error("keyed STS validation currently requires Windows CNG AES");
#endif
}

}

bool validateMacFcs(const std::vector<int>& bytes, int fcsLength)
{
  if ((fcsLength != 2 && fcsLength != 4) || bytes.size() < static_cast<size_t>(fcsLength) ||
      !std::all_of(bytes.begin(), bytes.end(), [](int value) { return value >= 0 && value <= 255; })) return false;
  uint32_t crc = fcsLength == 2 ? 0 : 0xffffffffu;
  const uint32_t polynomial = fcsLength == 2 ? 0x8408u : 0xedb88320u;
  for (size_t index = 0; index < bytes.size() - fcsLength; ++index) {
    crc ^= bytes[index];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? polynomial : 0);
  }
  if (fcsLength == 4) crc ^= 0xffffffffu;
  uint32_t received = 0;
  for (int index = 0; index < fcsLength; ++index)
    received |= static_cast<uint32_t>(bytes[bytes.size() - fcsLength + index]) << (8 * index);
  return crc == received;
}

std::vector<std::complex<double>> correctCarrier(const std::vector<std::complex<double>>& iq, double radiansPerSample)
{
  std::vector<std::complex<double>> corrected(iq.size());
  const auto step = std::polar(1.0, radiansPerSample);
  for (size_t block = 0; block < iq.size(); block += 64) {
    auto rotation = std::polar(1.0, radiansPerSample * block);
    const size_t end = std::min(block + 64, iq.size());
    for (size_t point = block; point < end; ++point) {
      corrected[point] = iq[point] * rotation;
      rotation *= step;
    }
  }
  return corrected;
}

FrameResult demodulateFrame(const std::vector<std::complex<double>>& iq,
    const Config& config, const FrameConfig& frame)
{
  FrameResult result;
  try {
    const bool ehprf = config.phyMode == EHPRF;
    const bool hprf = config.phyMode == HPRF || ehprf;
    int constraintLength = ehprf ? 7 : frame.constraintLength;
    double bitRate = frame.bitRateMbps;
    bool ldpc = false;
    const bool dynamicPhr = ehprf && frame.dynamicPhr;
    if ((frame.stsSegmentLength != 16 && frame.stsSegmentLength != 32 && frame.stsSegmentLength != 64 &&
         frame.stsSegmentLength != 128 && frame.stsSegmentLength != 256) ||
        (!hprf && frame.stsSegmentLength != 64) || frame.extraStsGapLength < 0 || frame.extraStsGapLength > 127 ||
        ((!hprf || frame.stsPacketConfig != 2) && frame.extraStsGapLength != 0) ||
        (frame.macFcsLength != 0 && frame.macFcsLength != 2 && frame.macFcsLength != 4))
      throw std::runtime_error("invalid STS segment length, gap or MAC FCS length");
    if ((frame.constraintLength != 3 && frame.constraintLength != 7) ||
        (!hprf && frame.constraintLength != 3) || frame.numStsSegments < 1 || frame.numStsSegments > 4 ||
        (!hprf && frame.numStsSegments != 1) || (hprf && (!std::isfinite(frame.meanPrfMHz) ||
         (std::fabs(frame.meanPrfMHz - 124.8) > 1e-6 && std::fabs(frame.meanPrfMHz - 249.6) > 1e-6))))
      throw std::runtime_error("unsupported coding, PRF or STS segment count");
    if (iq.empty() || !std::isfinite(config.sampleRate) || config.sampleRate <= 0 ||
      !std::isfinite(config.chipRateHz) || config.chipRateHz <= 0 ||
      !std::isfinite(config.syncThreshold) || config.syncThreshold < 0 || config.syncThreshold > 1 ||
      !std::isfinite(config.burstSearchThresholdDb))
      throw std::runtime_error("empty capture or invalid sample rate");
    if ((config.phyMode != BPRF && !hprf) || frame.sfdIndex < 0 || frame.sfdIndex > 4 ||
      (!hprf && !frame.legacy && frame.sfdIndex != 0 && frame.sfdIndex != 2) ||
      (frame.legacy && frame.sfdIndex != 0) || frame.samplesPerPulse < 2 ||
        frame.samplesPerPulse > 16 || config.syncLength < 16 || config.syncLength > 4096 ||
        frame.stsPacketConfig < 0 || frame.stsPacketConfig > 3 || (frame.legacy && frame.stsPacketConfig))
      throw std::runtime_error("invalid native frame configuration");
    if (iq.size() > 40000000 || !std::all_of(iq.begin(), iq.end(), [](std::complex<double> sample) {
          return std::isfinite(sample.real()) && std::isfinite(sample.imag()); }))
      throw std::runtime_error("invalid capture size or nonfinite samples");
    const auto code = preambleCode(frame.codeIndex);
    if ((hprf && code.size() != 91) || (!frame.legacy && code.size() == 31) ||
        (frame.legacy && code.size() == 91)) throw std::runtime_error("preamble code does not match PHY mode");
    const double legacyPrf = frame.legacy ? frame.legacyMeanPrfMHz : 62.4;
    if (!hprf && legacyPrf != 3.9 && legacyPrf != 15.6 && legacyPrf != 62.4)
      throw std::runtime_error("invalid legacy mean PRF");
    if (frame.extraStsGapIndex < 0 || frame.extraStsGapIndex > 3)
      throw std::runtime_error("invalid extra STS gap index");
    const int spread = frame.preambleSpread ? frame.preambleSpread : (code.size() == 31 ? (legacyPrf < 8 ? 64 : 16) : 4);
    if ((code.size() == 31 && spread != 16 && spread != 64) || (code.size() != 31 && spread != 4))
      throw std::runtime_error("invalid preamble spreading factor");
    const int oversample = frame.samplesPerPulse;
    if (std::fabs(config.sampleRate / config.chipRateHz - oversample) > 1e-6)
      throw std::runtime_error("integer configured samples per chip required");
    Config acquisitionConfig = config;
    acquisitionConfig.preamblePeriodChips = static_cast<int>(code.size()) * spread;
    result.acquisition = demodulate(iq, acquisitionConfig);
    const auto& acquired = result.acquisition;
    if (!acquired.ok || !acquired.syncFound) throw std::runtime_error("preamble acquisition failed");
    result.frequencyErrorHz = acquired.frequencyErrorHz;
    const int period = static_cast<int>(code.size()) * spread * oversample;
    const std::vector<std::vector<int>> sfdTable = {
      {0,1,0,-1,1,0,0,-1}, {-1,-1,1,-1}, {-1,-1,-1,1,-1,-1,1,-1},
      {-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,-1,1,-1},
      {-1,-1,-1,-1,-1,-1,-1,1,-1,-1,1,-1,-1,1,-1,1,-1,1,-1,-1,-1,1,1,-1,-1,-1,1,-1,1,1,-1,-1}};
    const std::vector<int> longSfd = {0,1,0,-1,1,0,0,-1,0,1,0,-1,1,0,0,-1,-1,0,0,1,0,-1,0,1,0,1,0,
      0,0,-1,0,-1,0,-1,0,0,1,0,-1,-1,0,-1,1,0,0,0,0,1,1,0,0,-1,-1,-1,1,-1,1,1,0,0,0,0,1,1};
    const auto sfdSigns = frame.legacy && frame.legacyRateMbps == 0.11 ? longSfd : sfdTable[frame.sfdIndex];
    const int sfdSymbols = static_cast<int>(sfdSigns.size());
    const int sfdOffset = config.syncLength * period;
    const double radians = -6.2831853071795864769 * acquired.frequencyErrorHz / config.sampleRate;
    std::vector<std::complex<double>> corrected = correctCarrier(iq, radians);
    double best = 0;
    double bestEnergy = 0;
    int origin = -1;
    const auto searchSfd = [&](int burstStart) {
      for (int candidate = std::max(0, burstStart - 2 * period);
          candidate <= burstStart + period && candidate + sfdOffset + sfdSymbols * period < static_cast<int>(iq.size());
          ++candidate) {
        std::complex<double> dot(0, 0);
        double energy = 0, expectedEnergy = 0;
        for (int symbol = 0; symbol < sfdSymbols; ++symbol)
          for (size_t chip = 0; chip < code.size(); ++chip)
            if (code[chip]) {
              const auto value = corrected[candidate + sfdOffset + symbol * period + chip * spread * oversample];
              const int expected = sfdSigns[symbol] * code[chip];
              dot += value * static_cast<double>(expected);
              energy += std::norm(value);
              expectedEnergy += expected * expected;
            }
        const double score = energy > 0 ? std::abs(dot) / std::sqrt(energy * expectedEnergy) : 0;
        const double matchedEnergy = score * score * energy;
        if (score >= 0.8 && matchedEnergy > bestEnergy) {
          best = score;
          bestEnergy = matchedEnergy;
          origin = candidate;
        }
      }
    };
    searchSfd(acquired.burstStart);
    if (origin < 0 && config.burstSearch) {
      double peakPower = 0;
      for (const auto& sample : iq)
        peakPower = std::max(peakPower, std::norm(sample));
      const double threshold = peakPower * std::pow(10.0, -std::fabs(config.burstSearchThresholdDb) / 10.0);
      int quietSamples = 0;
      for (int sample = acquired.burstStart + period; sample < static_cast<int>(iq.size()); ++sample) {
        if (std::norm(iq[sample]) < threshold) {
          ++quietSamples;
        } else {
          if (quietSamples >= 2 * period) {
            searchSfd(sample);
            if (origin >= 0)
              break;
          }
          quietSamples = 0;
        }
      }
    }
    result.sfdRho = best;
    if (origin < 0 || best < 0.8) throw std::runtime_error("SFD correlation failed");
    std::complex<double> gain(0,0);
    int gainCount = 0;
    for (int symbol = 0; symbol < std::min(config.syncLength, 16); ++symbol)
      for (size_t chip = 0; chip < code.size(); ++chip) if (code[chip]) {
        gain += corrected[origin + symbol * period + chip * spread * oversample] * static_cast<double>(code[chip]);
        ++gainCount;
      }
    gain /= static_cast<double>(gainCount);
    if (std::abs(gain) < 1e-12) throw std::runtime_error("preamble gain estimate failed");
    const std::complex<double> inverseGain = 1.0 / gain;
    for (auto& sample : corrected)
      sample *= inverseGain;
    result.syncStart = origin;
    result.sfdFound = true;
    result.sfdStart = origin + sfdOffset;
    result.sfdLength = sfdSymbols * period;
    int cursor = result.sfdStart + result.sfdLength;
    std::vector<int8_t> reference((iq.size() - origin + oversample - 1) / oversample, 0);
    const int periodChips = period / oversample;
    for (int symbol = 0; symbol < config.syncLength + sfdSymbols; ++symbol)
      for (size_t chip = 0; chip < code.size(); ++chip)
        reference[symbol * periodChips + chip * spread] =
          code[chip] * (symbol < config.syncLength ? 1 : sfdSigns[symbol - config.syncLength]);
    const auto measureEvm = [&]() {
      const int taps = 33, firstDelay = -8;
      std::array<std::array<double, taps>, taps> normal = {};
      std::array<std::complex<double>, taps> channel = {};
      const int trainingEnd = std::min(config.syncLength, 16) * periodChips - taps;
      for (int chip = taps; chip < trainingEnd; ++chip) {
        for (int row = 0; row < taps; ++row) {
          const int value = reference[chip - firstDelay - row];
          if (value == 0)
            continue;
          channel[row] += corrected[origin + chip * oversample] * static_cast<double>(value);
          for (int col = 0; col < taps; ++col)
            normal[row][col] += value * reference[chip - firstDelay - col];
        }
      }
      for (int pivot = 0; pivot < taps; ++pivot) {
        normal[pivot][pivot] += 1e-10;
        int bestRow = pivot;
        for (int row = pivot + 1; row < taps; ++row)
          if (std::abs(normal[row][pivot]) > std::abs(normal[bestRow][pivot])) bestRow = row;
        if (std::abs(normal[bestRow][pivot]) < 1e-12) return;
        std::swap(normal[pivot], normal[bestRow]);
        std::swap(channel[pivot], channel[bestRow]);
        const double scale = normal[pivot][pivot];
        for (int col = pivot; col < taps; ++col) normal[pivot][col] /= scale;
        channel[pivot] /= scale;
        for (int row = 0; row < taps; ++row) if (row != pivot) {
          const double factor = normal[row][pivot];
          for (int col = pivot; col < taps; ++col) normal[row][col] -= factor * normal[pivot][col];
          channel[row] -= factor * channel[pivot];
        }
      }
      const auto fieldEvm = [&](int start, int length) {
        double error = 0, energy = 0;
        const int begin = (start - origin) / oversample;
        const int fieldChips = length / oversample;
        const int edgeGuard = std::min(taps, fieldChips / 4);
        for (int chip = begin + edgeGuard; chip < begin + fieldChips - edgeGuard; ++chip) {
          std::complex<double> expected(0, 0);
          for (int tap = 0; tap < taps; ++tap) {
            const int source = chip - firstDelay - tap;
            if (source >= 0 && source < static_cast<int>(reference.size()))
              expected += channel[tap] * static_cast<double>(reference[source]);
          }
          error += std::norm(corrected[origin + chip * oversample] - expected);
          energy += std::norm(expected);
        }
        return energy > 0 ? 100 * std::sqrt(error / energy) : 0;
      };
      result.shrEvmRmsPercent = fieldEvm(origin, result.sfdStart + result.sfdLength - origin);
      if (result.stsValid) result.stsEvmRmsPercent = fieldEvm(result.stsStart, result.stsLength);
      if (result.phrValid) result.phrEvmRmsPercent = fieldEvm(result.phrStart, result.phrLength);
      if (result.psduValid && result.psduLength) result.psduEvmRmsPercent = fieldEvm(result.psduStart, result.psduLength);
      result.evmValid = true;
    };
    const auto validateSts = [&]() {
      result.stsPresent = true;
      result.stsStart = cursor;
      const int segmentChips = 512 * frame.stsSegmentLength;
      const int extraGap = frame.stsPacketConfig == 2 ? 4 * frame.extraStsGapLength : 0;
      result.stsLength = (512 + extraGap + frame.numStsSegments * (segmentChips + 512)) * oversample;
      if (cursor + result.stsLength > static_cast<int>(iq.size())) throw std::runtime_error("truncated STS");
      const int stsSpacing = hprf ? 4 : 8;
      const int bitsPerSegment = segmentChips / stsSpacing;
      const auto expected = stsBits(frame, frame.numStsSegments * bitsPerSegment);
      result.stsRho = 1;
      for (int segment = 0; segment < frame.numStsSegments; ++segment) {
        double dot = 0, energy = 0;
        for (int index = 0; index < bitsPerSegment; ++index) {
          const int chip = 512 + extraGap + segment * (segmentChips + 512) + index * stsSpacing;
          const double value = corrected[cursor + chip * oversample].real();
          dot += value * (1 - 2 * expected[segment * bitsPerSegment + index]);
          energy += value * value;
          reference[(cursor - origin) / oversample + chip] = 1 - 2 * expected[segment * bitsPerSegment + index];
        }
        const double correlation = energy > 0 ? dot / std::sqrt(energy * bitsPerSegment) : 0;
        result.stsRho = std::min(result.stsRho, correlation);
      }
      result.stsValid = result.stsRho >= 0.8;
      cursor += result.stsLength;
      if (!result.stsValid) throw std::runtime_error("keyed STS correlation failed");
    };
    if (frame.stsPacketConfig == 1 || frame.stsPacketConfig == 3) validateSts();
    if (frame.stsPacketConfig == 3) { measureEvm(); result.ok = true; return result; }
    result.phrStart = cursor;
    int pnOffset = 0;
    if (dynamicPhr) {
      if (static_cast<size_t>(cursor) + 3072 * oversample > iq.size())
        throw std::runtime_error("truncated EHPRF PHR1");
      const char* patterns[] = {"11111111111111111111", "00000100110011001100",
        "00000001100110011001", "00000111000011110000", "00000010010110100101",
        "00000100001111000011", "00000001011010010110", "00000111111100000000",
        "00000010101001010101", "00000100110000110011"};
      const double rates[] = {1.95, 7.8, 31.2, 62.4, 124.8};
      const auto spreading = spreadingBits(code, 640);
      std::array<int, 20> decisions = {};
      for (int symbol = 0; symbol < 20; ++symbol) {
        double dot = 0;
        for (int pulse = 0; pulse < 32; ++pulse)
          dot += corrected[cursor + (symbol * 128 + pulse * 2) * oversample].real() *
            (1 - 2 * spreading[symbol * 32 + pulse]);
        decisions[symbol] = dot < 0;
      }
      int bestPattern = -1, bestDistance = 21;
      for (int pattern = 0; pattern < 10; ++pattern) {
        int distance = 0;
        for (int index = 0; index < 20; ++index) distance += decisions[index] != patterns[pattern][index] - '0';
        if (distance < bestDistance) { bestPattern = pattern; bestDistance = distance; }
        else if (distance == bestDistance) bestPattern = -1;
      }
      if (bestPattern < 0 || bestDistance > 1) throw std::runtime_error("EHPRF PHR1 rate/coding check failed");
      bitRate = rates[bestPattern / 2];
      ldpc = bestPattern % 2 == 0;
      result.phr1Valid = true;
      for (int symbol = 0; symbol < 20; ++symbol) for (int pulse = 0; pulse < 32; ++pulse)
        reference[(cursor - origin) / oversample + symbol * 128 + pulse * 2] =
          (1 - 2 * (patterns[bestPattern][symbol] - '0')) * (1 - 2 * spreading[symbol * 32 + pulse]);
      cursor += 3072 * oversample;
      pnOffset = 640;
    }
    if (ehprf && bitRate != 1.95 && bitRate != 6.8 && bitRate != 7.8 && bitRate != 27.2 &&
        bitRate != 31.2 && bitRate != 62.4 && bitRate != 124.8)
      throw std::runtime_error("invalid EHPRF bit rate");
    if (ehprf && !dynamicPhr) constraintLength = bitRate == 6.8 || bitRate == 27.2 ? 3 : 7;
    const double meanPrf = ehprf ? (bitRate <= 7.8 ? 124.8 : 249.6) : frame.meanPrfMHz;
    const int groupLength = ehprf && constraintLength == 7 ?
      static_cast<int>((124.8 / bitRate) / (249.6 / meanPrf)) : (hprf && meanPrf < 200 ? 8 : 4);
    const int chipStride = hprf && meanPrf < 200 ? 2 : 1;
    const int headerBits = dynamicPhr ? 23 : 19;
    const int headerSymbols = headerBits + constraintLength - 1;
    result.bitRateMbps = ehprf ? bitRate : 0;
    result.ldpcCoding = ldpc;
    const int bursts = legacyPrf == 3.9 ? 128 : (legacyPrf == 15.6 ? 32 : 8);
    const int lowRateBurst = legacyPrf == 3.9 ? 4 : (legacyPrf == 15.6 ? 16 : 64);
    const int headerBurst = hprf ? (constraintLength == 3 || ldpc ? 4 : 2) * groupLength :
      (frame.legacy && frame.legacyRateMbps == 0.11 ? lowRateBurst * 8 :
        (legacyPrf == 62.4 && frame.highRatePhr ? 8 : lowRateBurst));
    const int headerChips = (hprf ? 2 * chipStride : bursts) * headerBurst;
    const int headerPn = pnOffset + headerSymbols * headerBurst;
    auto pn = spreadingBits(code, headerPn);
    std::vector<std::array<int, 2>> words;
    const auto demodWords = [&](int count, int burstChips, int pnStart) {
      const int symbolSamples = (hprf ? 2 * chipStride : bursts) * burstChips * oversample;
      if (static_cast<size_t>(cursor) + static_cast<size_t>(count) * symbolSamples > iq.size())
        throw std::runtime_error("truncated PHR or PSDU");
      for (int symbol = 0; symbol < count; ++symbol) {
        if (hprf) {
          int bestWord = 0;
          double bestScore = -std::numeric_limits<double>::infinity();
          for (int word = 0; word < 4; ++word) {
            double score = 0;
            for (int chip = 0; chip < burstChips; ++chip) {
                const int group = chip / groupLength;
                const int mappedBit = constraintLength == 7 ? ((word >> (group % 2)) & 1) :
                  (word >> 1) ^ (group % 2 == 0 ? (word & 1) : 0);
                const int chipPosition = (group * 2 * groupLength + chip % groupLength) * chipStride;
              const int expected = 1 - 2 * (mappedBit ^ pn[pnStart + symbol * burstChips + chip]);
              score += corrected[cursor + chipPosition * oversample].real() * expected;
            }
            if (score > bestScore) { bestScore = score; bestWord = word; }
          }
          words.push_back({{bestWord & 1, bestWord >> 1}});
          cursor += symbolSamples;
          continue;
        }
        int bestBurst = 0;
        double bestEnergy = -1;
        for (int burst = 0; burst < bursts; ++burst) {
          double energy = 0;
          for (int chip = 0; chip < burstChips; ++chip)
            energy += std::norm(corrected[cursor + (burst * burstChips + chip) * oversample]);
          if (energy > bestEnergy) { bestEnergy = energy; bestBurst = burst; }
        }
        double decision = 0;
        for (int chip = 0; chip < burstChips; ++chip)
          decision += corrected[cursor + (bestBurst * burstChips + chip) * oversample].real() *
              (1 - 2 * pn[pnStart + symbol * burstChips + chip]);
        words.push_back({{bestBurst >= bursts / 2, decision < 0}});
        cursor += symbolSamples;
      }
    };
    result.phr2Start = cursor;
    result.phr2Length = headerSymbols * headerChips * oversample;
    result.phrLength = cursor - result.phrStart + result.phr2Length;
    demodWords(headerSymbols, headerBurst, pnOffset);
    auto header = decodeConvolutional(words, constraintLength, constraintLength == 7);
    header.resize(headerBits);
    if (dynamicPhr) {
      int crc = 0;
      for (int bit : header) {
        const bool feedback = ((crc >> 7) & 1) != bit;
        crc = ((crc << 1) & 255) ^ (feedback ? 7 : 0);
      }
      if (crc || header[0] || header[1]) throw std::runtime_error("EHPRF PHR2 CRC-8 check failed");
    } else if (!checkHeader(header)) throw std::runtime_error("PHR SECDED check failed");
    result.phrBits = header;
    result.phrValid = true;
    const int rate = 2 * header[0] + header[1];
    if (!hprf) {
      result.bitRateMbps = rate == 0 ? 0.11 : (rate == 1 ? 0.85 :
        (rate == 2 ? (legacyPrf == 3.9 ? 1.7 : 6.81) : (legacyPrf == 3.9 ? 6.81 : 27.24)));
      if ((rate == 0) != (frame.legacy && frame.legacyRateMbps == 0.11))
        throw std::runtime_error("decoded PHR rate disagrees with configured long-SFD mode");
      if (!frame.legacy && rate != 2) throw std::runtime_error("BPRF requires a 6.81 Mbps PHR rate");
    }
    int length = 0;
    const bool gapIndexInHeader = hprf && !dynamicPhr && frame.stsPacketConfig == 2 &&
      (frame.extraStsGapIndex || frame.extraStsGapLength);
    if (gapIndexInHeader && (2 * header[0] + header[1]) != frame.extraStsGapIndex)
      throw std::runtime_error("PHR extra STS gap index does not match configuration");
    for (int index = dynamicPhr ? 2 : (hprf && !gapIndexInHeader ? 0 : 2); index < (dynamicPhr ? 14 : (hprf ? 12 : 9)); ++index)
      length = (length << 1) | header[index];
    result.psduLengthBytes = length;
    const int payloadBits = 8 * length;
    const int ldpcLength = payloadBits <= 168 ? 648 : (payloadBits <= 352 || (payloadBits > 968 && payloadBits <= 1296) ? 1296 : 1944);
    const int ldpcBlocks = payloadBits <= 968 ? 1 : (payloadBits <= 1296 ? 2 : (payloadBits + 971) / 972);
    const int shortening = ldpcBlocks * ldpcLength / 2 - payloadBits;
    const bool uncoded = !hprf && rate == 3 && legacyPrf != 62.4;
    const int rsBits = payloadBits + 48 * ((payloadBits + 329) / 330);
    const int codedBits = ldpc ? (ldpcBlocks * ldpcLength - shortening) / 2 : (uncoded ? rsBits / 2 :
      payloadBits + (constraintLength == 7 ? 6 : 48 * ((payloadBits + 329) / 330)));
    const int dataBurst = hprf ? 2 * groupLength : (rate == 0 ? lowRateBurst * 8 :
      (rate == 1 ? lowRateBurst : (rate == 2 ? (legacyPrf == 62.4 ? 8 : 2) : (legacyPrf == 62.4 ? 2 : 1))));
    const int hopBits = bursts == 128 ? 5 : (bursts == 32 ? 3 : 1);
    pn = spreadingBits(code, headerPn + codedBits * dataBurst + hopBits);
    result.psduStart = cursor;
    result.psduLength = codedBits * (hprf ? 2 * chipStride : bursts) * dataBurst * oversample;
    demodWords(codedBits, dataBurst, headerPn);
    auto decoded = ldpc ? std::vector<int>() : decodeConvolutional(words, constraintLength, true);
    if (uncoded) {
      decoded = header;
      for (size_t index = headerSymbols; index < words.size(); ++index) {
        decoded.push_back(words[index][0]);
        decoded.push_back(words[index][1]);
      }
    }
    std::vector<int> payload;
    std::vector<int> correctedCodedPayload;
    int bitOffset = 19;
    if (ldpc) {
      LdpcCode decoder(ldpcLength, ldpcLength / 2);
      decoder.load_wifi_ldpc(ldpcLength, 0);
      int readBit = 2 * headerSymbols;
      for (int block = 0; block < ldpcBlocks; ++block) {
        const int shortBits = shortening / ldpcBlocks + (block < shortening % ldpcBlocks);
        const int infoBits = ldpcLength / 2 - shortBits;
        std::vector<double> llr(ldpcLength, 20.0);
        std::vector<uint8_t> hard(ldpcLength, 0);
        for (int index = 0; index < ldpcLength; ++index) {
          if (index >= infoBits && index < ldpcLength / 2) continue;
          hard[index] = words[readBit / 2][readBit % 2];
          llr[index] = hard[index] ? -4.0 : 4.0;
          ++readBit;
        }
        const auto correctedBlock = decoder.decode(llr, 50, true);
        if (!decoder.check_codeword(correctedBlock)) throw std::runtime_error("PSDU LDPC parity check failed");
        for (int index = 0; index < ldpcLength; ++index) {
          if (index >= infoBits && index < ldpcLength / 2 && correctedBlock[index])
            throw std::runtime_error("invalid shortened LDPC padding");
          result.ldpcCorrectedBits += correctedBlock[index] != hard[index];
          if (index < infoBits || index >= ldpcLength / 2) correctedCodedPayload.push_back(correctedBlock[index]);
        }
        payload.insert(payload.end(), correctedBlock.begin(), correctedBlock.begin() + infoBits);
      }
    } else if (constraintLength == 7) {
      payload.assign(decoded.begin() + headerSymbols, decoded.begin() + headerSymbols + payloadBits);
    } else for (int remaining = payloadBits; remaining > 0; remaining -= std::min(330, remaining)) {
      const int dataBits = std::min(330, remaining);
      const int missing = 330 - dataBits;
      std::array<int, 63> block = {};
      for (int index = missing; index < 378; ++index)
        block[index / 6] |= decoded[bitOffset + index - missing] << (index % 6);
      bitOffset += dataBits + 48;
      const int correctedSymbols = correctReedSolomon(block);
      if (correctedSymbols < 0) throw std::runtime_error("PSDU Reed-Solomon decoding failed");
      result.rsCorrectedSymbols += correctedSymbols;
      for (int index = 0; index < missing; ++index)
        if ((block[index / 6] >> (index % 6)) & 1) throw std::runtime_error("invalid shortened RS padding");
      for (int index = missing; index < 330; ++index) payload.push_back((block[index / 6] >> (index % 6)) & 1);
      for (int index = missing; index < 378; ++index)
        correctedCodedPayload.push_back((block[index / 6] >> (index % 6)) & 1);
    }
    for (int index = 0; index < payloadBits; index += 8) {
      int byte = 0;
      for (int bit = 0; bit < 8; ++bit) byte |= payload[index + bit] << bit;
      result.psduBytes.push_back(byte);
    }
    result.psduValid = true;
    if (frame.stsPacketConfig == 2) validateSts();
    std::vector<std::array<int, 2>> referenceWords;
    if (constraintLength == 7) {
      auto inputs = header;
      inputs.resize(headerBits + 6, 0);
      referenceWords = encodeConvolutional(inputs, 7);
      if (ldpc) {
        for (size_t index = 0; index < correctedCodedPayload.size(); index += 2)
          referenceWords.push_back({{correctedCodedPayload[index], correctedCodedPayload[index + 1]}});
      } else {
        inputs = payload;
        inputs.resize(payloadBits + 6, 0);
        const auto payloadWords = encodeConvolutional(inputs, 7);
        referenceWords.insert(referenceWords.end(), payloadWords.begin(), payloadWords.end());
      }
    } else if (uncoded) {
      auto inputs = header;
      inputs.resize(headerBits + 2, 0);
      referenceWords = encodeConvolutional(inputs, 3);
      for (size_t index = 0; index < correctedCodedPayload.size(); index += 2)
        referenceWords.push_back({{correctedCodedPayload[index], correctedCodedPayload[index + 1]}});
    } else {
      auto inputs = header;
      inputs.insert(inputs.end(), correctedCodedPayload.begin(), correctedCodedPayload.end());
      inputs.resize(inputs.size() + 2, 0);
      referenceWords = encodeConvolutional(inputs, 3);
    }
    for (size_t symbol = 0; symbol < referenceWords.size(); ++symbol) {
      const bool isHeader = symbol < static_cast<size_t>(headerSymbols);
      const int burstChips = isHeader ? headerBurst : dataBurst;
      const int symbolPn = isHeader ? pnOffset + static_cast<int>(symbol) * headerBurst :
        headerPn + (static_cast<int>(symbol) - headerSymbols) * dataBurst;
      const int symbolStart = isHeader ? (result.phr2Start - origin) / oversample + symbol * headerChips :
        (result.psduStart - origin) / oversample + (symbol - headerSymbols) *
          (hprf ? 2 * chipStride : bursts) * dataBurst;
      if (hprf) {
        const int word = referenceWords[symbol][0] | (referenceWords[symbol][1] << 1);
        for (int chip = 0; chip < burstChips; ++chip) {
          const int group = chip / groupLength;
          const int mapped = constraintLength == 7 ? (word >> (group % 2)) & 1 :
            (word >> 1) ^ (group % 2 == 0 ? (word & 1) : 0);
          const int position = (group * 2 * groupLength + chip % groupLength) * chipStride;
          reference[symbolStart + position] = 1 - 2 * (mapped ^ pn[symbolPn + chip]);
        }
      } else {
        int hop = 0;
        const int hopStart = symbolPn - (!isHeader && frame.legacyVsaHopping && burstChips == 2 && bursts == 32 ? 1 : 0);
        for (int bit = 0; bit < hopBits; ++bit) hop |= pn[hopStart + bit] << bit;
        const int burst = hop + referenceWords[symbol][0] * bursts / 2;
        for (int chip = 0; chip < burstChips; ++chip)
          reference[symbolStart + burst * burstChips + chip] =
            (1 - 2 * referenceWords[symbol][1]) * (1 - 2 * pn[symbolPn + chip]);
      }
    }
    measureEvm();
    if (frame.macFcsLength) {
      result.macFcsChecked = true;
      if (length < frame.macFcsLength) throw std::runtime_error("PSDU shorter than configured MAC FCS");
      result.macFcsValid = validateMacFcs(result.psduBytes, frame.macFcsLength);
      if (!result.macFcsValid) throw std::runtime_error("MAC FCS check failed");
    }
    result.ok = true;
  } catch (const std::exception& error) {
    result.error = error.what();
  }
  return result;
}

}