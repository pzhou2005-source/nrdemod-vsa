#pragma once
/**
 ****************************************************************************
 *
 * Native UWB (802.15.4z BPRF/HPRF, 802.15.4ab draft EHPRF) burst/preamble demodulator
 * core.
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency, mirrors the
 * nr/NrOfdmCore.hpp and bt/BtGfskCore.hpp pattern. Implements a genuine
 * correlation receiver for the HRP-UWB PHY (IEEE 802.15.4z / 802.15.4-2020,
 * clause 15):
 *
 *   1. burst detection (windowed power threshold, same technique as the
 *      Bluetooth GFSK core)
 *   2. SYNC field acquisition via periodic autocorrelation: the SYNC
 *      preamble repeats the same ternary code every "SyncLength" symbol
 *      periods, so correlating the signal against a delayed copy of
 *      itself at the nominal preamble symbol period produces a genuine,
 *      measurable periodicity peak without needing the literal per-code
 *      ternary sequence tables (clause 15.2.6.2) - the same self-similarity
 *      principle any impulse-radio preamble detector exploits.
 *   3. carrier frequency error from the residual phase rotation across
 *      the detected SYNC period (BPSK-modulated preamble symbols cancel
 *      under magnitude-squared combining, leaving only the carrier
 *      rotation, the same trick the GFSK core's F0 estimator uses).
 *
 * demodulate() retains the burst/preamble/CFO scope above.
 * demodulateFrame() additionally acquires SFD, correlates keyed STS,
 * decodes SECDED or EHPRF dynamic PHR, and recovers PSDU bytes with
 * K3/RS, separately terminated K7, or shortened rate-1/2 QC-LDPC.
 * Includes preamble tables 1-32, SFD0-4 and legacy long SFD, legacy
 * rate/PRF-dependent modulation, HPRF and draft EHPRF modulation,
 * 1-4 individually checked STS segments of 16/32/64/128/256 units,
 * and configured SP2 extra gaps. Invalid PHY combinations reject.
 * STS AES uses Windows CNG; non-Windows builds reject STS requests.
 * Optional MAC FCS checking is separate from PHY decoding validity.
 * Native EVM uses post-FEC re-encoding and a SYNC-trained 33-tap FIR
 * on the chip lattice, excluding bounded field-edge transients.
 * This is not VSA-equivalent EVM or an IEEE conformance certificate.
 * Offsets follow the acquired pulse lattice, including filter delay.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace uwbdemod {

enum PhyMode {
  BPRF = 0, // base pulse repetition frequency (499.2 MHz chip rate)
  HPRF = 1, // high pulse repetition frequency (499.2 MHz chip rate, different burst structure)
  EHPRF = 2
};

struct Config {
  double sampleRate;             // Hz, input IQ sample rate
  int    phyMode;                // PhyMode enum
  int    syncLength;             // number of SYNC preamble symbol repetitions (e.g. 64)
  double chipRateHz;             // nominal UWB chip rate, 499.2 MHz per the spec
  bool   burstSearch;            // search for the burst in the capture
  double burstSearchThresholdDb; // power threshold below the peak, dB
  double syncThreshold;          // minimum normalized autocorrelation to declare sync, 0..1
  int preamblePeriodChips = 0;

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  bool   burstFound;
  int    burstStart;
  int    burstLength;

  bool   syncFound;
  double syncRho;           // normalized periodic autocorrelation magnitude, 0..1
  int    syncPositionSamples;
  double preambleSymbolPeriodSamples; // measured period of the repeating SYNC code

  double frequencyErrorHz; // carrier frequency offset from the SYNC field's residual rotation

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

struct FrameConfig {
  int codeIndex = 3;
  int sfdIndex = 0;
  int stsPacketConfig = 0;
  int samplesPerPulse = 4;
  bool legacy = true;
  bool highRatePhr = false;
  int constraintLength = 3;
  int numStsSegments = 1;
  double meanPrfMHz = 249.6;
  bool dynamicPhr = true;
  double bitRateMbps = 27.2;
  double legacyRateMbps = 6.81;
  double legacyMeanPrfMHz = 15.6;
  int preambleSpread = 0;
  int extraStsGapIndex = 0;
  bool legacyVsaHopping = true;
  int stsSegmentLength = 64;
  int extraStsGapLength = 0;
  int macFcsLength = 0;
  std::string stsKeyHex = "4A5572BC90798C8E518D2449092F1B55";
  std::string stsIvHex = "68DEBD3A599939DD57FDBB0E2A10FAC0";
};

struct FrameResult {
  bool ok = false;
  std::string error;
  Result acquisition;
  bool sfdFound = false;
  bool stsPresent = false;
  bool stsValid = false;
  bool phrValid = false;
  bool psduValid = false;
  bool phr1Valid = false;
  bool ldpcCoding = false;
  bool macFcsChecked = false;
  bool macFcsValid = false;
  double bitRateMbps = 0;
  int ldpcCorrectedBits = 0;
  int phr2Start = -1;
  int phr2Length = 0;
  bool evmValid = false;
  double shrEvmRmsPercent = 0;
  double stsEvmRmsPercent = 0;
  double phrEvmRmsPercent = 0;
  double psduEvmRmsPercent = 0;
  int syncStart = -1;
  int sfdStart = -1;
  int sfdLength = 0;
  int stsStart = -1;
  int stsLength = 0;
  int phrStart = -1;
  int phrLength = 0;
  int psduStart = -1;
  int psduLength = 0;
  int psduLengthBytes = 0;
  int rsCorrectedSymbols = 0;
  double sfdRho = 0.0;
  double stsRho = 0.0;
  double frequencyErrorHz = 0.0;
  std::vector<int> phrBits;
  std::vector<int> psduBytes;
};

FrameResult demodulateFrame(const std::vector<std::complex<double> >& iq,
    const Config& config, const FrameConfig& frameConfig);

bool validateMacFcs(const std::vector<int>& psduBytes, int fcsLength);

}
