#pragma once
/**
 ****************************************************************************
 *
 * Native UWB (IEEE 802.15.4z HRP) impulse-radio burst/preamble demodulator
 * core.
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency, mirrors the
 * nr/NrOfdmCore.hpp and bt/BtGfskCore.hpp pattern. Implements a genuine
 * correlation receiver for the HRP-UWB PHY (IEEE 802.15.4z / 802.15.4-2020,
 * clause 15):
 *
 *   1. burst detection (windowed power threshold, same technique as the
 *      Bluetooth GFSK core)
 *   2. blind SYNC period acquisition using normalized energy correlation
 *   3. carrier frequency error from complex correlation of adjacent
 *      repetitions of the same signed preamble code
 *
 * demodulate() preserves the acquisition-only interface. demodulateData()
 * additionally decodes BPRF/802.15.4a/HPRF SFD, PHR and PSDU with SECDED,
 * convolutional and Reed-Solomon error correction.
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
  IEEE802154A = 2
};

struct Config {
  double sampleRate;             // Hz, input IQ sample rate
  int    phyMode;                // PhyMode enum
  int    syncLength;             // number of SYNC preamble symbol repetitions (e.g. 64)
  double chipRateHz;             // nominal UWB chip rate, 499.2 MHz per the spec
  bool   burstSearch;            // search for the burst in the capture
  double burstSearchThresholdDb; // power threshold below the peak, dB
  double syncThreshold;          // minimum normalized autocorrelation to declare sync, 0..1

  Config();
};

struct DataConfig {
  int codeIndex;
  int sfdNumber;
  int sfdLength;                 // 802.15.4a: 8 or 64
  int preambleSpreadingFactor;   // 0 selects the standard code-dependent value
  double meanPrfMHz;             // 4a: 3.9/15.6/62.4; BPRF: 62.4; HPRF: 124.8/249.6
  int phrDataRateKbps;           // BPRF: 0 (auto), 850 or 6810
  int stsPacketConfiguration;    // 0..3; STS is skipped, not authenticated
  int constraintLength;         // HPRF: 3 (with RS) or 7 (no RS)
  int numStsSegments;            // HPRF: 1..4
  int stsSegmentLength;          // HPRF: 16,32,64,128,256 units of 512 chips
  int extraStsGapIndex;          // HPRF SP2: 0..3
  int extraStsGapLength;         // HPRF SP2: 0..127 units of 4 chips

  DataConfig();
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

  std::string dataError;
  bool sfdFound;
  bool phrDecoded;
  bool psduDecoded;
  bool rsOk;                    // not applicable (false) for HPRF CL7
  int preambleSymbols;
  int sfdSymbol;
  int phrStartSamples;
  int psduLengthBytes;
  int dataRateKbps;
  int rsCorrectedSymbols;
  double evmDb;                 // matched burst/group correlation EVM, not VSA chip NRMSE
  std::vector<int> phrBits;      // 13 SECDED-corrected information bits
  std::vector<int> psduBits;
  std::vector<int> psduBytes;
  std::vector<int> burstPositions;
  std::vector<double> softBits;  // two metrics/symbol; HPRF CL3 first metric is g0 XOR g1

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);
Result demodulateData(const std::vector<std::complex<double> >& iq, const Config& config,
                     const DataConfig& dataConfig);

}
