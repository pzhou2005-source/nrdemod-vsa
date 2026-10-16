#include "uwb/UwbHrpCore.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace uwbdemod {
namespace {

typedef std::complex<double> Complex;
const double PI = 3.14159265358979323846;

const char* const CODES[] = {
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
  "-+++++++++++---+-++--++00-+-+0-+-0-++--+++-+++0--0++-+-+-0---++-0----++++-+-0+0-+-++-++--+-"
};

std::vector<int> preambleCode(int index)
{
  std::vector<int> code;
  for (const char* p = CODES[index - 1]; *p; ++p)
    code.push_back(*p == '+' ? 1 : (*p == '-' ? -1 : 0));
  return code;
}

std::vector<int> sfdCode(const Config& cfg, const DataConfig& data)
{
  const int shortSfd[] = {0, 1, 0, -1, 1, 0, 0, -1};
  if (cfg.phyMode != IEEE802154A && data.sfdNumber == 2) {
    const int sfd[] = {-1, -1, -1, 1, -1, -1, 1, -1};
    return std::vector<int>(sfd, sfd + 8);
  }
  if (cfg.phyMode == HPRF) {
    const char* sfds[] = {"", "--+-", "---+--+-", "-----++--+-+--+-",
                         "-------+--+--+-+-+---++---+-++--"};
    std::vector<int> out;
    const char* seq = sfds[data.sfdNumber];
    for (; *seq; ++seq) out.push_back(*seq == '+' ? 1 : -1);
    return out;
  }
  std::vector<int> out(shortSfd, shortSfd + 8);
  if (cfg.phyMode == IEEE802154A && data.sfdLength == 64) {
    const int tail[] = {-1,0,0,1,0,-1,0,1,0,1,0,0,0,-1,0,-1,0,-1,0,0,1,0,-1,-1,
                       0,-1,1,0,0,0,0,1,1,0,0,-1,-1,-1,1,-1,1,1,0,0,0,0,1,1};
    out.insert(out.end(), shortSfd, shortSfd + 8);
    out.insert(out.end(), tail, tail + 48);
  }
  return out;
}

std::vector<double> pulse(int samplesPerChip)
{
  const int n = 16 * samplesPerChip;
  std::vector<double> p(n, 0.0);
  p[0] = 1.0;
  const double k = std::tan(PI / (2.0 * samplesPerChip));
  for (int section = 0; section < 2; ++section) {
    const double q = 1.0 / (2.0 * std::cos((section == 0 ? 1 : 3) * PI / 8.0));
    const double d = 1.0 + k / q + k * k;
    const double b = k * k / d;
    const double a1 = 2.0 * (k * k - 1.0) / d;
    const double a2 = (1.0 - k / q + k * k) / d;
    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
    for (int i = 0; i < n; ++i) {
      const double x = p[i];
      const double y = b * (x + 2.0 * x1 + x2) - a1 * y1 - a2 * y2;
      p[i] = y;
      x2 = x1; x1 = x; y2 = y1; y1 = y;
    }
  }
  return p;
}

std::vector<Complex> matchedFilter(const std::vector<Complex>& iq, int spc, double cfo, double fs)
{
  const std::vector<double> p = pulse(spc);
  double energy = 0.0;
  for (size_t j = 0; j < p.size(); ++j) energy += p[j] * p[j];
  std::vector<Complex> corrected(iq.size()), mf(iq.size());
  for (size_t i = 0; i < iq.size(); ++i)
    corrected[i] = iq[i] * std::polar(1.0, -2.0 * PI * cfo * i / fs);
  for (size_t i = 0; i < iq.size(); ++i) {
    Complex sum(0.0, 0.0);
    for (size_t j = 0; j < p.size() && i + j < iq.size(); ++j) sum += corrected[i + j] * p[j];
    mf[i] = sum / energy;
  }
  return mf;
}

Complex codeCorrelation(const std::vector<Complex>& mf, int start, const std::vector<int>& code, int stride)
{
  Complex sum(0.0, 0.0);
  int pulses = 0;
  for (size_t i = 0; i < code.size(); ++i) {
    if (!code[i]) continue;
    const size_t index = static_cast<size_t>(start) + i * stride;
    if (index < mf.size()) sum += mf[index] * static_cast<double>(code[i]);
    ++pulses;
  }
  return sum / static_cast<double>(pulses);
}

std::vector<int> pnSequence(const std::vector<int>& code, int count)
{
  std::vector<int> pn(count + 15, 0);
  int seed = 0;
  for (size_t i = 0; i < code.size() && seed < 15; ++i)
    if (code[i]) pn[seed++] = code[i] > 0 ? 1 : 0;
  for (int i = 15; i < count + 15; ++i) pn[i] = pn[i - 14] ^ pn[i - 15];
  return std::vector<int>(pn.begin() + 15, pn.end());
}

int parity(int value)
{
  int p = 0;
  for (; value; value >>= 1) p ^= value & 1;
  return p;
}

std::vector<int> viterbi(const std::vector<double>& soft, bool terminated,
                         int constraintLength = 3, bool grayMapping = false)
{
  const size_t n = soft.size() / 2;
  const double low = -std::numeric_limits<double>::infinity();
  const int states = 1 << (constraintLength-1);
  std::array<double, 64> metric;
  metric.fill(low);
  metric[0] = 0.0;
  std::vector<std::array<unsigned char, 64> > previous(n);
  int predecessor[64][2], sign0[64][2], sign1[64][2];
  for (int ns = 0; ns < states; ++ns) {
    const int input = ns >> (constraintLength-2);
    for (int b = 0; b < 2; ++b) {
      const int state = ((ns & (states/2-1)) << 1) | b;
      predecessor[ns][b] = state;
      const int reg = (input << (constraintLength-1)) | state;
      const int g0 = parity(reg & (constraintLength == 3 ? 02 : 0133));
      const int g1 = parity(reg & (constraintLength == 3 ? 05 : 0171));
      sign0[ns][b] = 1-2*(grayMapping ? g0^g1 : g0);
      sign1[ns][b] = 1-2*g1;
    }
  }
  for (size_t k = 0; k < n; ++k) {
    std::array<double, 64> next;
    next.fill(low);
    for (int ns = 0; ns < states; ++ns) {
      double best = low;
      int bestState = 0;
      for (int b = 0; b < 2; ++b) {
        const int state = predecessor[ns][b];
        const double value = metric[state]+soft[2*k]*sign0[ns][b]+soft[2*k+1]*sign1[ns][b];
        if (value > best) { best = value; bestState = state; }
      }
      next[ns] = best;
      previous[k][ns] = static_cast<unsigned char>(bestState);
    }
    metric = next;
  }
  int state = terminated ? 0 : static_cast<int>(std::max_element(metric.begin(), metric.end()) - metric.begin());
  std::vector<int> bits(n);
  for (size_t k = n; k > 0; --k) {
    bits[k-1] = state >> (constraintLength-2);
    state = previous[k-1][state];
  }
  return bits;
}

std::vector<int> secdedEncode(const std::vector<int>& input)
{
  std::vector<int> b(19, 0);
  std::copy(input.begin(), input.begin() + 13, b.begin());
  const int parity[5][8] = {
    {2,1,9,7,5,4,11,12}, {1,7,6,4,3,10,11,13}, {2,9,8,4,3,10,11,0},
    {9,8,7,6,5,10,11,0}, {13,12,0,0,0,0,0,0}
  };
  for (int row = 0; row < 5; ++row)
    for (int j = 0; j < 8; ++j) if (parity[row][j]) b[18-row] ^= b[parity[row][j]-1];
  for (int i = 0; i < 19; ++i) if (i != 13) b[13] ^= b[i];
  return b;
}

bool secdedDecode(const std::vector<int>& decoded, std::vector<int>& phr)
{
  std::vector<int> b(decoded.begin(), decoded.begin() + 19);
  for (int flip = -1; flip < 19; ++flip) {
    std::vector<int> candidate = b;
    if (flip >= 0) candidate[flip] ^= 1;
    if (secdedEncode(candidate) == candidate) {
      phr.assign(candidate.begin(), candidate.begin() + 13);
      return true;
    }
  }
  return false;
}

struct Gf64 {
  int exp[126], log[64];
  Gf64() {
    std::fill(log, log + 64, -1);
    int x = 1;
    for (int i = 0; i < 63; ++i) {
      exp[i] = x; log[x] = i;
      x <<= 1;
      if (x & 64) x ^= 0x43;
    }
    for (int i = 63; i < 126; ++i) exp[i] = exp[i-63];
  }
  int mul(int a, int b) const { return a && b ? exp[log[a]+log[b]] : 0; }
  int div(int a, int b) const { return a ? exp[(log[a]-log[b]+63)%63] : 0; }
  int power(int a, int n) const { return n == 0 ? 1 : (a ? exp[(log[a]*n)%63] : 0); }
};

std::array<int, 8> syndromes(const std::vector<int>& word, const Gf64& gf)
{
  std::array<int, 8> s = {{0,0,0,0,0,0,0,0}};
  for (int j = 0; j < 8; ++j)
    for (size_t i = 0; i < word.size(); ++i) s[j] = gf.mul(s[j], gf.exp[j+1]) ^ word[i];
  return s;
}

bool rsCorrect(std::vector<int>& word, int& corrections)
{
  const Gf64 gf;
  const std::array<int, 8> s = syndromes(word, gf);
  if (std::count(s.begin(), s.end(), 0) == 8) return true;
  int c[9] = {1,0,0,0,0,0,0,0,0}, b[9] = {1,0,0,0,0,0,0,0,0};
  int degree = 0, shift = 1, scale = 1;
  for (int n = 0; n < 8; ++n) {
    int discrepancy = s[n];
    for (int i = 1; i <= degree; ++i) discrepancy ^= gf.mul(c[i], s[n-i]);
    if (!discrepancy) { ++shift; continue; }
    int old[9];
    std::copy(c, c+9, old);
    const int ratio = gf.div(discrepancy, scale);
    for (int i = 0; i + shift < 9; ++i) c[i+shift] ^= gf.mul(ratio, b[i]);
    if (2*degree <= n) {
      degree = n+1-degree;
      std::copy(old, old+9, b);
      scale = discrepancy;
      shift = 1;
    } else ++shift;
  }
  if (degree < 1 || degree > 4) return false;
  std::vector<int> positions, roots;
  for (int pos = 0; pos < 63; ++pos) {
    const int x = gf.exp[(pos+1)%63];
    int value = c[degree];
    for (int i = degree-1; i >= 0; --i) value = gf.mul(value, x) ^ c[i];
    if (!value) { positions.push_back(pos); roots.push_back(gf.exp[62-pos]); }
  }
  if (static_cast<int>(positions.size()) != degree) return false;
  int a[4][5] = {};
  for (int row = 0; row < degree; ++row) {
    for (int col = 0; col < degree; ++col) a[row][col] = gf.power(roots[col], row+1);
    a[row][degree] = s[row];
  }
  for (int col = 0; col < degree; ++col) {
    int pivot = col;
    while (pivot < degree && !a[pivot][col]) ++pivot;
    if (pivot == degree) return false;
    for (int j = 0; j <= degree; ++j) std::swap(a[pivot][j], a[col][j]);
    const int divisor = a[col][col];
    for (int j = col; j <= degree; ++j) a[col][j] = gf.div(a[col][j], divisor);
    for (int row = 0; row < degree; ++row) if (row != col) {
      const int factor = a[row][col];
      for (int j = col; j <= degree; ++j) a[row][j] ^= gf.mul(factor, a[col][j]);
    }
  }
  for (int i = 0; i < degree; ++i) word[positions[i]] ^= a[i][degree];
  const std::array<int, 8> verified = syndromes(word, gf);
  if (std::count(verified.begin(), verified.end(), 0) != 8) return false;
  corrections += degree;
  return true;
}

bool rsDecode(const std::vector<int>& encoded, int bitCount, Result& result)
{
  size_t offset = 0;
  std::vector<int> output;
  for (int start = 0; start < bitCount; start += 330) {
    const int count = std::min(330, bitCount-start), pad = 330-count;
    if (offset + count + 48 > encoded.size()) return false;
    std::vector<int> word(63, 0);
    for (int i = 0; i < count+48; ++i) word[(pad+i)/6] |= encoded[offset+i] << ((pad+i)%6);
    if (!rsCorrect(word, result.rsCorrectedSymbols)) return false;
    for (int i = 0; i < pad; ++i) if ((word[i/6] >> (i%6)) & 1) return false;
    for (int i = 0; i < count; ++i) output.push_back((word[(pad+i)/6] >> ((pad+i)%6)) & 1);
    offset += count+48;
  }
  result.psduBits.swap(output);
  return true;
}

int chipsPerBurst(int prf, int rate)
{
  const int rates[] = {110,850,1700,6810,27240};
  const int low[] = {32,4,2,1,0}, mid[] = {128,16,0,2,1}, high[] = {512,64,0,8,2};
  for (int i = 0; i < 5; ++i)
    if (rate == rates[i]) return prf == 0 ? low[i] : (prf == 1 ? mid[i] : high[i]);
  return 0;
}

bool demodSymbols(const std::vector<Complex>& chips, const std::vector<int>& code, int symbols,
                  int bursts, int cpbPhr, int cpbData, Result& out, double& evm)
{
  const int phrSymbols = std::min(symbols, 21);
  const size_t total = static_cast<size_t>(phrSymbols)*bursts*cpbPhr
      + static_cast<size_t>(symbols-phrSymbols)*bursts*cpbData;
  if (total > chips.size()) return false;
  const std::vector<int> pn = pnSequence(code, phrSymbols*cpbPhr + (symbols-phrSymbols)*cpbData + 32);
  int hopBits = 0;
  for (int n = bursts/4; n > 1; n >>= 1) ++hopBits;
  out.softBits.clear();
  out.burstPositions.clear();
  double theta = 0.0, error = 0.0;
  size_t chipStart = 0;
  int pnStart = 0;
  for (int k = 0; k < symbols; ++k) {
    const int cpb = k < 21 ? cpbPhr : cpbData;
    int hop = 0;
    // When cpb < hopBits the next symbol's PN bits are shared, not shifted backwards.
    for (int j = 0; j < hopBits; ++j) hop |= pn[pnStart+j] << j;
    Complex z[2] = {Complex(0,0), Complex(0,0)};
    for (int half = 0; half < 2; ++half) {
      const size_t begin = chipStart + (hop + half*bursts/2)*cpb;
      for (int j = 0; j < cpb; ++j) z[half] += chips[begin+j] * static_cast<double>(1-2*pn[pnStart+j]);
      z[half] = z[half] * std::polar(1.0, -theta) / static_cast<double>(cpb);
    }
    const double systematic = std::fabs(z[0].real()) - std::fabs(z[1].real());
    const int chosen = systematic < 0.0 ? 1 : 0;
    out.softBits.push_back(systematic);
    out.softBits.push_back(z[chosen].real());
    out.burstPositions.push_back(hop);
    const Complex tracked = std::abs(z[1]) > std::abs(z[0]) ? z[1] : z[0];
    theta += 0.05 * std::arg(tracked * (tracked.real() < 0.0 ? -1.0 : 1.0));
    error += std::norm(z[chosen] - Complex(z[chosen].real() < 0 ? -1 : 1, 0));
    pnStart += cpb;
    chipStart += bursts*cpb;
  }
  evm = 10.0 * std::log10(error/symbols + 1e-30);
  return true;
}

bool demodHprfSymbols(const std::vector<Complex>& chips, const std::vector<int>& code,
                      int symbols, const DataConfig& cfg, Result& out)
{
  const bool lowPrf = std::fabs(cfg.meanPrfMHz-124.8) < 0.01;
  const int groupLength = lowPrf ? 8 : 4, spacing = lowPrf ? 2 : 1;
  const int headerSymbols = cfg.constraintLength == 3 ? 21 : 25;
  const int phrSpread = (cfg.constraintLength == 3 ? 4 : 2)*groupLength;
  const int dataSpread = 2*groupLength;
  const int nPhr = std::min(symbols,headerSymbols);
  const int spreadCount = nPhr*phrSpread+(symbols-nPhr)*dataSpread;
  if (static_cast<size_t>(spreadCount)*2*spacing > chips.size()) return false;
  const std::vector<int> pn = pnSequence(code,spreadCount);
  out.softBits.clear();
  out.burstPositions.clear();
  int chipStart = 0, pnStart = 0;
  double theta = 0.0, error = 0.0;
  for (int k = 0; k < symbols; ++k) {
    const int spread = k < headerSymbols ? phrSpread : dataSpread;
    Complex z[2] = {Complex(0,0),Complex(0,0)};
    for (int j = 0; j < spread; ++j) {
      const int group = j/groupLength, offset = j%groupLength;
      const int chip = chipStart+(2*group*groupLength+offset)*spacing;
      z[group%2] += chips[chip]*static_cast<double>(1-2*pn[pnStart+j]);
    }
    Complex tracked(0,0);
    for (int bit = 0; bit < 2; ++bit) {
      z[bit] *= std::polar(2.0/spread,-theta);
      out.softBits.push_back(z[bit].real());
      const double sign = z[bit].real() < 0.0 ? -1.0 : 1.0;
      tracked += z[bit]*sign;
      error += std::norm(z[bit]-Complex(sign,0));
    }
    theta += 0.05*std::arg(tracked);
    pnStart += spread;
    chipStart += 2*spread*spacing;
  }
  out.evmDb = 10.0*std::log10(error/(2*symbols)+1e-30);
  return true;
}

void decodeHprf(const std::vector<Complex>& chips, const std::vector<int>& code,
                const DataConfig& data, Result& out)
{
  const bool cl7 = data.constraintLength == 7;
  const int headerSymbols = cl7 ? 25 : 21;
  if (!demodHprfSymbols(chips,code,headerSymbols,data,out) ||
      !secdedDecode(viterbi(out.softBits,cl7,data.constraintLength,!cl7),out.phrBits)) {
    out.dataError = "HPRF PHR truncated or SECDED failed";
    return;
  }
  out.phrDecoded = true;
  int firstLengthBit = 0;
  if (data.stsPacketConfiguration == 2 && (data.extraStsGapIndex || data.extraStsGapLength)) {
    if (2*out.phrBits[0]+out.phrBits[1] != data.extraStsGapIndex) {
      out.dataError = "HPRF PHR extra STS gap index mismatch";
      return;
    }
    firstLengthBit = 2;
  }
  for (int i = firstLengthBit; i < 12; ++i) out.psduLengthBytes = 2*out.psduLengthBytes+out.phrBits[i];
  const int nBits = 8*out.psduLengthBytes;
  const int nEncoded = cl7 ? nBits : nBits+48*((nBits+329)/330);
  const int symbols = cl7 ? 25+nBits+6 : 19+nEncoded+2;
  if (!demodHprfSymbols(chips,code,symbols,data,out)) {
    out.dataError = "capture too short for HPRF PSDU";
    return;
  }
  // CL7 terminates the PHR and PSDU independently; the PSDU has no RS outer code.
  if (cl7) {
    const std::vector<double> psduSoft(out.softBits.begin()+50,out.softBits.end());
    const std::vector<int> decoded = viterbi(psduSoft,true,7);
    out.psduBits.assign(decoded.begin(),decoded.begin()+nBits);
  } else {
    const std::vector<int> decoded = viterbi(out.softBits,true,3,true);
    const std::vector<int> encoded(decoded.begin()+19,decoded.begin()+19+nEncoded);
    out.rsOk = rsDecode(encoded,nBits,out);
    if (!out.rsOk) { out.dataError = "HPRF Reed-Solomon decode failed"; return; }
  }
  out.psduBytes.assign(out.psduLengthBytes,0);
  for (int i = 0; i < nBits; ++i) out.psduBytes[i/8] |= out.psduBits[i] << (i%8);
  const bool lowPrf = std::fabs(data.meanPrfMHz-124.8) < 0.01;
  out.dataRateKbps = cl7 ? (lowPrf ? 7800 : 31200) : (lowPrf ? 6810 : 27240);
  out.psduDecoded = true;
}

} // namespace

DataConfig::DataConfig()
    : codeIndex(9), sfdNumber(0), sfdLength(8), preambleSpreadingFactor(0),
      meanPrfMHz(62.4), phrDataRateKbps(0), stsPacketConfiguration(0),
      constraintLength(3), numStsSegments(1), stsSegmentLength(64),
      extraStsGapIndex(0), extraStsGapLength(0)
{
}

Result demodulateData(const std::vector<Complex>& iq, const Config& cfg, const DataConfig& data)
{
  Result out = demodulate(iq, cfg);
  if (!out.ok) { out.dataError = out.error; return out; }
  const bool hprf = cfg.phyMode == HPRF;
  const double ratio = cfg.sampleRate / cfg.chipRateHz;
  const int spc = static_cast<int>(std::llround(ratio));
  const int prf = std::fabs(data.meanPrfMHz-3.9) < 0.01 ? 0
      : (std::fabs(data.meanPrfMHz-15.6) < 0.01 ? 1 : (std::fabs(data.meanPrfMHz-62.4) < 0.01 ? 2 : -1));
  const bool hprfPrf = std::fabs(data.meanPrfMHz-124.8) < 0.01 || std::fabs(data.meanPrfMHz-249.6) < 0.01;
  if (data.codeIndex < 1 || data.codeIndex > 32 || (!hprf && prf < 0) ||
      (hprf && (!hprfPrf || data.codeIndex < 25 || data.sfdNumber < 1 || data.sfdNumber > 4 ||
                (data.constraintLength != 3 && data.constraintLength != 7) ||
                data.numStsSegments < 1 || data.numStsSegments > 4 ||
                (data.stsSegmentLength != 16 && data.stsSegmentLength != 32 && data.stsSegmentLength != 64 &&
                 data.stsSegmentLength != 128 && data.stsSegmentLength != 256) ||
                data.extraStsGapIndex < 0 || data.extraStsGapIndex > 3 ||
                data.extraStsGapLength < 0 || data.extraStsGapLength > 127)) ||
      (cfg.phyMode == BPRF && (prf != 2 || data.codeIndex < 9 ||
       (data.sfdNumber != 0 && data.sfdNumber != 2))) ||
      (cfg.phyMode == IEEE802154A && (data.codeIndex > 24 || data.sfdNumber != 0 ||
       ((data.codeIndex <= 8) != (prf != 2)))) ||
      (data.sfdLength != 8 && data.sfdLength != 64) ||
      data.stsPacketConfiguration < 0 || data.stsPacketConfiguration > 3 ||
      (cfg.phyMode == IEEE802154A && data.stsPacketConfiguration != 0) ||
      (data.phrDataRateKbps != 0 && data.phrDataRateKbps != 850 && data.phrDataRateKbps != 6810) ||
      spc < 2 || spc > 32 || std::fabs(spc*cfg.chipRateHz-cfg.sampleRate) > 1e3) {
    out.dataError = "invalid or unsupported HRP data configuration";
    return out;
  }
  const std::vector<int> code = preambleCode(data.codeIndex);
  const int spreading = data.preambleSpreadingFactor ? data.preambleSpreadingFactor
      : (code.size() == 31 ? (prf == 0 ? 64 : 16) : 4);
  if (spreading != 4 && spreading != 16 && spreading != 64) {
    out.dataError = "unsupported preamble spreading factor";
    return out;
  }
  const int period = static_cast<int>(code.size()) * spreading * spc;
  const std::vector<int> sfd = sfdCode(cfg, data);
  if (iq.size() < static_cast<size_t>(period)*(cfg.syncLength+sfd.size())) {
    out.dataError = "capture too short for configured SHR";
    return out;
  }
  std::vector<Complex> mf = matchedFilter(iq, spc, 0.0, cfg.sampleRate);
  const int searchEnd = std::min(static_cast<int>(iq.size())-period,
      std::max(3*period, out.burstStart+2*period+static_cast<int>(iq.size()/200)));
  int peak = 0;
  double peakPower = 0.0;
  for (int i = 0; i <= searchEnd; ++i) {
    const double value = std::norm(codeCorrelation(mf, i, code, spreading*spc));
    if (value > peakPower) { peakPower = value; peak = i; }
  }
  if (!(peakPower > 0.0)) { out.dataError = "preamble code not found"; return out; }
  int timing = peak % period;
  while (timing + period < static_cast<int>(iq.size()) &&
         std::norm(codeCorrelation(mf, timing, code, spreading*spc)) < 0.25*peakPower) timing += period;
  if (static_cast<size_t>(timing) + static_cast<size_t>(cfg.syncLength+sfd.size())*period > iq.size()) {
    out.dataError = "capture too short after preamble acquisition";
    return out;
  }
  Complex rotation(0,0);
  Complex previous = codeCorrelation(mf, timing, code, spreading*spc);
  for (int k = 1; k < cfg.syncLength; ++k) {
    const Complex current = codeCorrelation(mf, timing+k*period, code, spreading*spc);
    rotation += current * std::conj(previous);
    previous = current;
  }
  out.frequencyErrorHz = std::arg(rotation) * cfg.sampleRate / (2*PI*period);
  mf = matchedFilter(iq, spc, out.frequencyErrorHz, cfg.sampleRate);
  int refined = timing;
  double best = 0.0;
  for (int d = -spc; d <= spc; ++d) {
    if (timing+d < 0) continue;
    double energy = 0.0;
    for (int k = 0; k < std::min(16, cfg.syncLength); ++k)
      energy += std::norm(codeCorrelation(mf, timing+d+k*period, code, spreading*spc));
    if (energy > best) { best = energy; refined = timing+d; }
  }
  timing = refined;
  Complex gain(0,0);
  for (int k = 0; k < cfg.syncLength; ++k)
    gain += codeCorrelation(mf, timing+k*period, code, spreading*spc);
  gain /= cfg.syncLength;
  if (std::norm(gain) < 1e-30) { out.dataError = "preamble phase reference is zero"; return out; }
  for (size_t k = 0; k < sfd.size(); ++k) {
    const Complex value = codeCorrelation(mf, timing+(cfg.syncLength+static_cast<int>(k))*period,
                                          code, spreading*spc) / gain;
    if (std::abs(value-Complex(sfd[k],0)) > 0.6) {
      out.dataError = "SFD does not match configured preamble length/sequence";
      return out;
    }
  }
  out.syncFound = out.sfdFound = true;
  out.syncPositionSamples = timing;
  out.preambleSymbolPeriodSamples = period;
  out.preambleSymbols = cfg.syncLength;
  out.sfdSymbol = cfg.syncLength;
  if (data.stsPacketConfiguration == 3) return out;
  int start = timing + (cfg.syncLength+static_cast<int>(sfd.size()))*period;
  if (data.stsPacketConfiguration == 1)
    start += (hprf ? 512+data.numStsSegments*(data.stsSegmentLength*512+512) : 33792)*spc;
  out.phrStartSamples = start;
  if (start >= static_cast<int>(mf.size())) { out.dataError = "capture too short for STS/PHR"; return out; }
  std::vector<Complex> chips;
  for (size_t i = start; i < mf.size(); i += spc) chips.push_back(mf[i]/gain);
  if (hprf) {
    decodeHprf(chips,code,data,out);
    return out;
  }
  const int bursts = prf == 0 ? 128 : (prf == 1 ? 32 : 8);
  std::vector<int> phrRates;
  if (cfg.phyMode == IEEE802154A) phrRates.push_back(data.sfdLength == 64 ? 110 : 850);
  else if (data.phrDataRateKbps) phrRates.push_back(data.phrDataRateKbps);
  else { phrRates.push_back(850); phrRates.push_back(6810); }
  int cpbPhr = 0;
  double evm = 0.0, bestPhrEvm = std::numeric_limits<double>::infinity();
  std::vector<int> bestPhr;
  for (size_t k = 0; k < phrRates.size(); ++k) {
    const int cpb = chipsPerBurst(prf, phrRates[k]);
    std::vector<int> candidate;
    if (demodSymbols(chips, code, 21, bursts, cpb, cpb, out, evm) &&
        secdedDecode(viterbi(out.softBits, false), candidate) && evm < bestPhrEvm) {
      // SECDED alone can accept random wrong-rate decisions; enforce PHR semantics
      // and select the rate whose burst correlations best match the modulation.
      const int preambleLengths[] = {16,64,1024,4096};
      if (candidate[10] || preambleLengths[2*candidate[11]+candidate[12]] != cfg.syncLength ||
          (cfg.phyMode == BPRF && (candidate[0] != 1 || candidate[1] != 0))) continue;
      bestPhr = candidate;
      bestPhrEvm = evm;
      cpbPhr = cpb;
    }
  }
  if (!cpbPhr) { out.dataError = "PHR truncated or SECDED failed"; return out; }
  out.phrBits.swap(bestPhr);
  out.phrDecoded = true;
  const int ratesLow[] = {110,850,1700,6810}, ratesOther[] = {110,850,6810,27240};
  const int rateIndex = 2*out.phrBits[0]+out.phrBits[1];
  out.dataRateKbps = prf == 0 ? ratesLow[rateIndex] : ratesOther[rateIndex];
  for (int i = 2; i < 9; ++i) out.psduLengthBytes = 2*out.psduLengthBytes+out.phrBits[i];
  const int cpbData = chipsPerBurst(prf, out.dataRateKbps);
  if (!cpbData || out.phrBits[10] || (cfg.phyMode == BPRF && out.dataRateKbps != 6810)) {
    out.dataError = "invalid PHR data rate or reserved bit";
    return out;
  }
  const int nBits = 8*out.psduLengthBytes;
  const int nRs = nBits + 48*((nBits+329)/330);
  const bool coded = !((prf == 0 && out.dataRateKbps == 6810) || (prf == 1 && out.dataRateKbps == 27240));
  const int symbols = coded ? 19+nRs+2 : 21+nRs/2;
  if (!demodSymbols(chips, code, symbols, bursts, cpbPhr, cpbData, out, out.evmDb)) {
    out.dataError = "capture too short for PSDU";
    return out;
  }
  std::vector<int> encoded;
  if (coded) {
    const std::vector<int> decoded = viterbi(out.softBits, true);
    encoded.assign(decoded.begin()+19, decoded.begin()+19+nRs);
  } else {
    for (int i = 0; i < nRs; ++i) encoded.push_back(out.softBits[42+i] < 0.0 ? 1 : 0);
  }
  out.rsOk = rsDecode(encoded, nBits, out);
  if (!out.rsOk) { out.dataError = "Reed-Solomon decode failed"; return out; }
  out.psduBytes.assign(out.psduLengthBytes, 0);
  for (int i = 0; i < nBits; ++i) out.psduBytes[i/8] |= out.psduBits[i] << (i%8);
  out.psduDecoded = true;
  return out;
}

} // namespace uwbdemod
