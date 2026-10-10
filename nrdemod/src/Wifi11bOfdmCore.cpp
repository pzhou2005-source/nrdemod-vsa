/**
 ****************************************************************************
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11b/Wifi11bOfdmCore.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace wifi11bdemod {

namespace {

typedef std::complex<double> cd;

const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;
const double CHIP_RATE = 11.0e6;    // 11 Mchip/s, fixed for all 802.11b data rates
const int    BARKER_LEN = 11;
const double SYMBOL_PERIOD = static_cast<double>(BARKER_LEN) / CHIP_RATE; // 1 Msym/s SYNC/header rate
const int    CCK_CHIPS_PER_SYMBOL = 8;
// Maximum number of 1 Mb/s DBPSK/Barker-11 PLCP preamble+header symbols to scan past the
// detected SYNC start while searching for the DATA field's start - the real preamble+header
// length is not a single fixed constant (observed 186 symbols against these TDC references'
// own recorded NumPreambleSymbols+NumHeaderSymbols, but the exact point found by the SYNC
// search below can land on any one of several equally-valid, repeating Barker-aligned
// positions within that preamble - see findDataFieldStart()), so the actual boundary is
// located dynamically per capture by scanning forward for where the Barker-11 correlation,
// which stays strong and essentially constant through the whole preamble+header, sharply and
// durably drops (confirmed on real captures: preamble/header symbols score ~0.96-1.0, DATA
// field symbols immediately after score <0.3 and stay low).
const int MAX_PLCP_HEADER_SYMBOLS = 400;

// IEEE 802.11-2016 15.2.3 (and the identical sequence used by every DSSS/CCK data rate's
// PLCP preamble): the 11-chip Barker sequence, +1/-1 polarity. Cross-verified against a real
// TDC 802.11b capture (11b_1Mbps_BPSK.wfm) via direct matched-filter correlation in an offline
// Python prototype before being relied upon here: achieved correlation score 0.998 at the
// correct chip-rate decimation phase, confirming both the sequence and its sign convention.
const double BARKER[BARKER_LEN] = {1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1};

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

// Matched-filter correlation of 11 consecutive chip-rate samples against the known Barker-11
// sequence, normalized to [0,1] (1 = perfect match), same technique as the OFDM cores' LTF
// matched filter but at chip (not OFDM-symbol) granularity.
cd barkerCorrelate(const std::vector<cd>& chips, int pos)
{
  cd corr(0.0, 0.0);
  for (int i = 0; i < BARKER_LEN; ++i) {
    corr += chips[pos + i] * BARKER[i]; // BARKER is real, so conj(BARKER[i]) == BARKER[i]
  }
  return corr;
}

double barkerScore(const std::vector<cd>& chips, int pos)
{
  const cd corr = barkerCorrelate(chips, pos);
  double energy = 0.0;
  for (int i = 0; i < BARKER_LEN; ++i) energy += std::norm(chips[pos + i]);
  const double denom = std::sqrt(energy * BARKER_LEN);
  return denom > 0.0 ? std::abs(corr) / denom : 0.0;
}

// A single 11-chip Barker correlation is not a strong enough statistic on its own - random
// noise alone can score as high as ~0.7 (confirmed empirically) purely by chance. The genuine
// SYNC field repeats the same Barker symbol for well over a hundred consecutive symbol
// periods, so averaging the per-symbol score over several consecutive symbol boundaries
// starting at pos gives a far more reliable confidence metric with a negligible false-positive
// rate, at the cost of needing that many genuine symbols to be present from pos onward.
double multiSymbolBarkerScore(const std::vector<cd>& chips, int pos, int numSymbols)
{
  double sum = 0.0;
  int count = 0;
  for (int s = 0; s < numSymbols; ++s) {
    const int p = pos + s * BARKER_LEN;
    if (p + BARKER_LEN > static_cast<int>(chips.size())) break;
    sum += barkerScore(chips, p);
    ++count;
  }
  return count > 0 ? sum / count : 0.0;
}

// Scans forward from the detected SYNC start, one Barker-length (11 chip) symbol at a time,
// for the first symbol whose own Barker-11 score drops below 0.5 AND whose following several
// symbols average well below 0.5 (a single genuine CCK DATA symbol can occasionally score
// above 0.5 purely by chance since it is no longer Barker-spread at all, so requiring every
// one of a short run to individually score low is too strict - an averaged confirmation
// window tolerates that per-symbol variance) - this is the start of the DATA field, whatever
// its own modulation turns out to be. Returns -1 if no such transition is found within
// MAX_PLCP_HEADER_SYMBOLS symbols (e.g. the capture ends during the preamble/header).
int findDataFieldStart(const std::vector<cd>& chips, int syncStartPos)
{
  const int confirmRun = 6;
  for (int s = 0; s < MAX_PLCP_HEADER_SYMBOLS; ++s) {
    const int pos = syncStartPos + s * BARKER_LEN;
    if (pos + BARKER_LEN > static_cast<int>(chips.size())) break;
    if (barkerScore(chips, pos) >= 0.5) continue;
    double sum = 0.0;
    int count = 0;
    for (int k = 0; k < confirmRun; ++k) {
      const int p2 = pos + k * BARKER_LEN;
      if (p2 + BARKER_LEN > static_cast<int>(chips.size())) break;
      sum += barkerScore(chips, p2);
      ++count;
    }
    if (count > 0 && sum / count < 0.5) return pos;
  }
  return -1;
}

cd hardDecisionBpsk(const cd& z)
{
  return cd(z.real() >= 0.0 ? 1.0 : -1.0, 0.0);
}

// IEEE 802.11-98/367 "Replacement Description of CCK" (Equation A, Figure D): each CCK
// codeword's 8 QPSK chips are a function of 4 phase parameters phi1..phi4, where phi1 rotates
// the entire codeword (and is itself differentially encoded symbol-to-symbol per 18.4.6.4.4 -
// NOT independently decodable per-symbol without tracking that running phase) and phi2/phi3/
// phi4 carry the actual payload bits. Verified against real TDC 802.11b captures (both 5.5 and
// 11 Mb/s) via an offline Python prototype BEFORE being relied on here: projecting each
// received 8-chip DATA symbol onto these 64 "base" codewords (phi1=0, phi2/phi3/phi4 swept
// over the 4 QPSK phases) with an unconstrained complex scalar (which absorbs the unknown
// channel gain AND the differentially-encoded phi1 together) achieves ~7% RMS residual on
// real captures, dramatically below the ~29-73% residual any mismatched formula/convention
// gives against the same real data (confirmed by testing reversed/conjugated chip order and
// several candidate codeword forms, all of which failed to drop below the random-noise
// baseline) - this is strong, independent confirmation that the formula and (phi2,phi3,phi4)
// payload convention are correct, even though the absolute phi1 value itself is only
// resolvable differentially and is therefore not decoded as a final payload bit.
cd cckBaseCodewordChip(int chipIndex, double phi2, double phi3, double phi4)
{
  switch (chipIndex) {
    case 0: return std::polar(1.0, phi2 + phi3 + phi4);
    case 1: return std::polar(1.0, phi3 + phi4);
    case 2: return std::polar(1.0, phi2 + phi4);
    case 3: return -std::polar(1.0, phi4);
    case 4: return std::polar(1.0, phi2 + phi3);
    case 5: return std::polar(1.0, phi3);
    case 6: return -std::polar(1.0, phi2);
    default: return cd(1.0, 0.0);
  }
}

// The 64 phi1=0 "base" CCK codewords (4 QPSK phases each for phi2, phi3, phi4), built once and
// reused for every DATA symbol's best-fit search.
std::vector<std::vector<cd> > cckBaseCodewords()
{
  const double qpsk[4] = {0.0, PI / 2.0, PI, 3.0 * PI / 2.0};
  std::vector<std::vector<cd> > out;
  out.reserve(64);
  for (int i2 = 0; i2 < 4; ++i2) {
    for (int i3 = 0; i3 < 4; ++i3) {
      for (int i4 = 0; i4 < 4; ++i4) {
        std::vector<cd> word(CCK_CHIPS_PER_SYMBOL);
        for (int c = 0; c < CCK_CHIPS_PER_SYMBOL; ++c) {
          word[c] = cckBaseCodewordChip(c, qpsk[i2], qpsk[i3], qpsk[i4]);
        }
        out.push_back(word);
      }
    }
  }
  return out;
}

// Rotation/amplitude-invariant best-fit match of one received 8-chip DATA symbol against the
// 64 base codewords: for each candidate, the optimal complex scalar is dot(seg,conj(word))/8
// (word energy is always 8), and the residual is ||seg||^2 - |dot|^2/8. Returns the best
// candidate's residual as a fraction of the symbol's own energy (0 = perfect match) and the
// winning index; this single search absorbs the unknown channel gain and the differentially-
// encoded phi1 rotation simultaneously, so no explicit channel/phase tracking state is needed
// between CCK symbols (unlike the SYNC field's coherent DBPSK demod above).
double cckBestFitResidualFraction(const std::vector<cd>& seg, const std::vector<std::vector<cd> >& bases,
    int& bestIndex)
{
  double segEnergy = 0.0;
  for (size_t c = 0; c < seg.size(); ++c) segEnergy += std::norm(seg[c]);
  if (segEnergy <= 0.0) {
    bestIndex = -1;
    return 1.0;
  }
  double bestResid = segEnergy; // candidate dot=0 would give residual=segEnergy
  bestIndex = 0;
  for (size_t w = 0; w < bases.size(); ++w) {
    cd dot(0.0, 0.0);
    for (int c = 0; c < CCK_CHIPS_PER_SYMBOL; ++c) dot += seg[c] * std::conj(bases[w][c]);
    const double resid = segEnergy - std::norm(dot) / static_cast<double>(CCK_CHIPS_PER_SYMBOL);
    if (resid < bestResid) {
      bestResid = resid;
      bestIndex = static_cast<int>(w);
    }
  }
  return bestResid / segEnergy;
}

}

Config::Config() : sampleRate(0.0), burstSearch(true), burstSearchThresholdDb(15.0), syncThreshold(0.5)
{
}

Result::Result()
    : ok(false), burstFound(false), burstStart(0), burstLength(0), syncFound(false), syncRho(0.0),
      timingOffsetSamples(0), frequencyErrorHz(0.0), rmsEvmPercent(0.0), peakEvmPercent(0.0),
      numSyncSymbolsMeasured(0), isCckDataRate(false), dataRmsEvmPercent(0.0), dataPeakEvmPercent(0.0),
      numDataSymbolsMeasured(0)
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

  // The receiver's own sample rate may be any integer multiple of the fixed 11 Mchip/s DSSS
  // chip rate; search every osr-phase decimation for the one that yields the strongest Barker
  // correlation (osr=1 is the common, already-at-chip-rate case).
  const double samplesPerChip = config.sampleRate / CHIP_RATE;
  if (samplesPerChip < 0.99 || std::fabs(samplesPerChip - std::llround(samplesPerChip)) > 1e-3) {
    result.error = "sample rate must be an integer multiple of 11 MHz";
    return result;
  }
  const int osr = static_cast<int>(std::llround(samplesPerChip));

  // Search a generous window at the start of the burst (the SYNC field is at least 56 chips
  // for the short PLCP, 128 for the long one, so a window of a few hundred chips is ample to
  // find the first Barker symbol regardless of which preamble variant was used) across every
  // osr decimation phase, scoring each candidate chip position by its MULTI-symbol (16
  // consecutive Barker periods) averaged correlation (see multiSymbolBarkerScore) - a single
  // 11-chip correlation alone is not a reliable enough statistic to reject noise-only false
  // positives. The SYNC field is long enough that MANY consecutive chip-aligned starting
  // positions all give an equally-high (within noise) multi-symbol average, since they all
  // remain fully inside the repeating SYNC run - so a plain global argmax over that whole
  // plateau would pick whichever position noise happens to nudge fractionally highest, not
  // necessarily the earliest. Avoid that by first finding the single highest score anywhere,
  // then taking the EARLIEST position within the same best phase whose score is within 1% of
  // that peak - the first such position in the SYNC run is the correct SYNC start.
  double bestScoreOverall = -1.0;
  int bestPhase = -1;
  const int searchChips = 400;
  const int confirmSymbols = 16;
  for (int phase = 0; phase < osr; ++phase) {
    const int maxChipIndex = static_cast<int>((static_cast<int>(iq.size()) - phase) / osr);
    const int chipsToTake = std::min(maxChipIndex, burstStart / osr + burstLength / osr + searchChips);
    if (chipsToTake <= BARKER_LEN) continue;
    std::vector<cd> chips(chipsToTake);
    for (int i = 0; i < chipsToTake; ++i) chips[i] = iq[phase + i * osr];
    const int chipBurstStart = burstStart / osr;
    const int searchEnd = std::min(static_cast<int>(chips.size()) - BARKER_LEN, chipBurstStart + searchChips);
    for (int pos = std::max(0, chipBurstStart - 2); pos < searchEnd; ++pos) {
      const double score = multiSymbolBarkerScore(chips, pos, confirmSymbols);
      if (score > bestScoreOverall) {
        bestScoreOverall = score;
        bestPhase = phase;
      }
    }
  }
  if (bestPhase < 0) {
    result.error = "Barker SYNC not found in capture";
    return result;
  }
  std::vector<cd> bestChips;
  {
    const int maxChipIndex = static_cast<int>((static_cast<int>(iq.size()) - bestPhase) / osr);
    const int chipsToTake = std::min(maxChipIndex, burstStart / osr + burstLength / osr + searchChips);
    bestChips.resize(chipsToTake);
    for (int i = 0; i < chipsToTake; ++i) bestChips[i] = iq[bestPhase + i * osr];
  }
  const int chipBurstStart = burstStart / osr;
  const int searchEnd = std::min(static_cast<int>(bestChips.size()) - BARKER_LEN, chipBurstStart + searchChips);
  int bestPos = -1;
  double bestScore = -1.0;
  for (int pos = std::max(0, chipBurstStart - 2); pos < searchEnd; ++pos) {
    const double score = multiSymbolBarkerScore(bestChips, pos, confirmSymbols);
    if (score >= 0.99 * bestScoreOverall) {
      bestPos = pos;
      bestScore = score;
      break;
    }
  }
  if (bestPos < 0) {
    result.error = "Barker SYNC not found in capture";
    return result;
  }

  result.syncRho = bestScore;
  result.syncFound = result.syncRho >= config.syncThreshold;
  result.timingOffsetSamples = bestPhase + bestPos * osr - burstStart;

  if (!result.syncFound) {
    result.ok = true;
    return result;
  }

  // Carrier frequency offset via the standard BPSK-squaring technique: each symbol-spaced (11
  // chip stride) Barker correlator output lies on one of two phase states 180 degrees apart
  // (DBPSK encodes data as a phase flip, not an absolute phase), so squaring the complex
  // correlator output removes that +-180 degree ambiguity, leaving (twice) the residual
  // carrier phase accumulated over one symbol period - no knowledge of the actual
  // (differentially-encoded, scrambled) bit values is required for this.
  //
  // The long PLCP SYNC+SFD+PLCP-header fields (192 1-bit DBPSK/Barker symbols: 128 SYNC + 16
  // SFD + 8 SIGNAL + 8 SERVICE + 16 LENGTH + 16 CRC, IEEE 802.11-2016 15.2.3) are always sent
  // at 1 Mb/s DBPSK/Barker regardless of the frame's actual DATA rate; measuring no further
  // than that keeps every symbol used here on the one, fully-defined modulation this core
  // decodes, instead of bleeding into the CCK- or DQPSK-coded DATA field beyond it.
  const int maxSymbols = (static_cast<int>(bestChips.size()) - bestPos) / BARKER_LEN;
  const int numSymbols = std::min(maxSymbols, 192);
  std::vector<cd> symbolCorr(numSymbols);
  for (int s = 0; s < numSymbols; ++s) {
    symbolCorr[s] = barkerCorrelate(bestChips, bestPos + s * BARKER_LEN);
  }
  cd diffSum(0.0, 0.0);
  for (int s = 1; s < numSymbols; ++s) {
    const cd sq = symbolCorr[s] * symbolCorr[s];
    const cd sqPrev = symbolCorr[s - 1] * symbolCorr[s - 1];
    diffSum += sq * std::conj(sqPrev);
  }
  const double diffPhase = std::abs(diffSum) > 0.0 ? std::arg(diffSum) : 0.0;
  result.frequencyErrorHz = diffPhase / (4.0 * PI * SYMBOL_PERIOD);
  const double freqRadPerSymbol = TWO_PI * result.frequencyErrorHz * SYMBOL_PERIOD;

  // Per-symbol EVM against the nearest ideal DBPSK constellation point (the Barker matched
  // filter already references the known, fully-defined spreading sequence, so after
  // correcting for the just-measured residual carrier frequency and the (unknown, but
  // constant) overall carrier phase offset, a genuine SYNC/header symbol must lie close to one
  // of the two antipodal points +-1 regardless of its actual (unknown-to-this-core)
  // differentially-encoded bit value - the same "blind EVM against nearest constellation
  // point" technique a vector signal analyzer uses). The static rotation is itself unknown (no
  // absolute phase reference - only the known Barker chip pattern, which is insensitive to a
  // constant carrier phase), so it is estimated the same way the symbol-to-symbol frequency
  // drift above was: squaring removes the +-180 degree DBPSK data ambiguity, and averaging the
  // frequency-corrected squared symbols across the whole SYNC field gives (twice) that
  // constant offset directly from the signal itself.
  cd rotationSum(0.0, 0.0);
  for (int s = 0; s < numSymbols; ++s) {
    const cd corrected = symbolCorr[s] * std::polar(1.0, -freqRadPerSymbol * s);
    rotationSum += corrected * corrected;
  }
  const double staticRotation = std::abs(rotationSum) > 0.0 ? 0.5 * std::arg(rotationSum) : 0.0;

  double evmSumSquares = 0.0;
  double evmPeak = -1.0;
  int evmCount = 0;
  for (int s = 0; s < numSymbols; ++s) {
    double energy = 0.0;
    for (int i = 0; i < BARKER_LEN; ++i) energy += std::norm(bestChips[bestPos + s * BARKER_LEN + i]);
    const double norm = std::sqrt(energy * BARKER_LEN);
    if (norm <= 0.0) continue;
    const cd corrected =
        symbolCorr[s] * std::polar(1.0, -freqRadPerSymbol * s - staticRotation) / norm;
    const cd decision = hardDecisionBpsk(corrected);
    const double err = std::norm(corrected - decision);
    const double refPower = std::norm(decision) > 0.0 ? std::norm(decision) : 1.0;
    const double evmFraction = err / refPower;
    evmSumSquares += evmFraction;
    ++evmCount;
    if (evmFraction > evmPeak) evmPeak = evmFraction;
  }
  result.numSyncSymbolsMeasured = evmCount;
  result.rmsEvmPercent = evmCount > 0 ? 100.0 * std::sqrt(evmSumSquares / static_cast<double>(evmCount)) : 0.0;
  result.peakEvmPercent = evmPeak >= 0.0 ? 100.0 * std::sqrt(evmPeak) : 0.0;

  // DATA field: starts after the preamble+header, found dynamically (see findDataFieldStart)
  // rather than at a fixed symbol count. 1/2 Mb/s DATA stays Barker-11 spread (so no
  // score-drop transition exists at all - findDataFieldStart simply returns -1, which is
  // expected and fine since those rates need no further decode here), while 5.5/11 Mb/s DATA
  // switches to the 8-chip-per-symbol CCK codeword set (confirmed empirically on real
  // captures: preamble/header symbols score ~0.96-1.0, CCK DATA symbols immediately drop to
  // ~0.02-0.3 and never recover for the remainder of the capture).
  const int dataChipStart = findDataFieldStart(bestChips, bestPos);
  result.isCckDataRate = dataChipStart >= 0;

  if (result.isCckDataRate) {
    const std::vector<std::vector<cd> > bases = cckBaseCodewords();
    // Bound by the detected burst end (NOT the full search buffer, which extends searchChips
    // past the burst and would otherwise let the DATA field measurement run into trailing
    // noise/silence past the real end of the transmission, corrupting the EVM average with
    // noise-only "symbols" - the same class of bug fixed for the VHT DATA field earlier).
    const int chipBurstEnd = (burstStart + burstLength) / osr;
    const int maxCckSymbols =
        (std::min(chipBurstEnd, static_cast<int>(bestChips.size())) - dataChipStart) / CCK_CHIPS_PER_SYMBOL;
    const int numCckSymbols = std::max(0, std::min(maxCckSymbols, 2000));
    double dataEvmSumSquares = 0.0;
    double dataEvmPeak = -1.0;
    int dataEvmCount = 0;
    for (int s = 0; s < numCckSymbols; ++s) {
      const int chipStart = dataChipStart + s * CCK_CHIPS_PER_SYMBOL;
      std::vector<cd> seg(bestChips.begin() + chipStart, bestChips.begin() + chipStart + CCK_CHIPS_PER_SYMBOL);
      int bestIndex = -1;
      const double evmFraction = cckBestFitResidualFraction(seg, bases, bestIndex);
      if (bestIndex < 0) continue;
      dataEvmSumSquares += evmFraction;
      ++dataEvmCount;
      if (evmFraction > dataEvmPeak) dataEvmPeak = evmFraction;
    }
    result.numDataSymbolsMeasured = dataEvmCount;
    result.dataRmsEvmPercent =
        dataEvmCount > 0 ? 100.0 * std::sqrt(dataEvmSumSquares / static_cast<double>(dataEvmCount)) : 0.0;
    result.dataPeakEvmPercent = dataEvmPeak >= 0.0 ? 100.0 * std::sqrt(dataEvmPeak) : 0.0;
  }

  result.ok = true;
  return result;
}

}
