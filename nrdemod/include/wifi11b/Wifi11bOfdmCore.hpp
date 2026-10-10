#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11b DSSS/CCK demodulator core.
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency, mirrors the
 * nr/NrOfdmCore.hpp, bt/BtGfskCore.hpp, uwb/UwbHrpCore.hpp pattern.
 * Implements a genuine receiver chain for the 802.11b DSSS/CCK PHY (IEEE
 * 802.11-2016 clause 15/16):
 *
 *   1. burst detection (windowed power threshold)
 *   2. chip-rate (11 Mchip/s) timing recovery and SYNC/SFD acquisition via
 *      matched-filter correlation against the 11-chip Barker sequence
 *      (every 802.11b PPDU, regardless of final data rate, begins with a
 *      long or short DBPSK/DQPSK PREAMBLE spread with the same Barker-11
 *      code - a direct, fully-defined correlation, not blind
 *      self-similarity)
 *   3. per-symbol differential (DBPSK) phase decode and EVM accumulation
 *      against the Barker-spread PLCP header, always sent at 1 Mb/s DBPSK
 *      Barker regardless of the frame's actual DATA rate.
 *   4. for 5.5/11 Mb/s captures (whose DATA field is CCK, not Barker-
 *      spread), per-symbol EVM against the IEEE 802.11-98/367 CCK
 *      codeword set via a rotation/amplitude-invariant best-fit search
 *      (absorbs the unknown channel gain and the differentially-encoded
 *      phi1 rotation together - see Wifi11bOfdmCore.cpp for the
 *      cross-verification against real TDC captures).
 *
 * Scope limit: the actual payload bits (phi2/phi3/phi4 -> dibits -> octets)
 * are not decoded/descrambled - only EVM/sync accuracy is measured, the
 * same split the other native cores use (PHY modulation accuracy, not MAC-
 * layer payload decode).
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace wifi11bdemod {

struct Config {
  double sampleRate;             // Hz, input IQ sample rate (802.11b: 11 MHz chip rate nominal)
  bool   burstSearch;            // search for the burst in the capture
  double burstSearchThresholdDb; // power threshold below the peak, dB
  double syncThreshold;          // minimum Barker correlation confidence (0..1) to declare sync

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  bool burstFound;
  int  burstStart;
  int  burstLength;

  bool   syncFound;
  double syncRho;             // Barker-11 matched-filter correlation confidence, 0..1
  int    timingOffsetSamples; // sample index (from burstStart) of the first SYNC chip

  double frequencyErrorHz; // carrier frequency offset, from consecutive Barker symbol phase drift

  double rmsEvmPercent;  // RMS EVM over the SYNC field DBPSK symbols, as a percentage
  double peakEvmPercent;
  int    numSyncSymbolsMeasured;

  bool   isCckDataRate;        // true if the DATA field's own Barker correlation is weak (5.5/11 Mb/s CCK)
  double dataRmsEvmPercent;    // RMS EVM over the CCK DATA field (best-fit residual), as a percentage
  double dataPeakEvmPercent;
  int    numDataSymbolsMeasured;

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
