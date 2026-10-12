#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11ad (DMG, 60 GHz / WiGig) burst detector and optional
 * explicitly configured single-carrier DATA EVM measurement.
 *
 * Pure C++ / STL. 802.11ad uses a fundamentally different PHY from every
 * other WiFi variant in this codebase: a single-carrier (SC) modulation
 * with a Golay-complementary-sequence (Ga128/Gb128) STF/CE preamble (IEEE
 * 802.11-2016 clause 20.5.3), not an OFDM L-STF/L-LTF structure at all - so
 * none of the existing wifi/wifi11n/wifi11ac/wifi11ax sync code applies.
 *
 * DATA scope: one sample per chip at 1.76 GSa/s, flat channel, SC MCS1-9.
 * Callers supply an independent known 128-chip +/- STF reference repeated
 * four times, and absolute training/DATA offsets. Training estimates CFO,
 * phase and scalar channel gain; DATA excludes the first 64 chips of each
 * 512-chip block. EVM is a ratio against nearest constellation decisions.
 * No built-in Golay table, automatic field/header decode, CE multipath
 * equalizer, RRC matched filter, payload bit decode or OFDM support.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace wifi11addemod {

struct Config {
  double burstSearchThresholdDb; // power threshold below the peak, dB
  bool measureDataEvm;
  double sampleRate;
  std::string trainingReference;
  int trainingOffsetSamples;
  int dataOffsetSamples;
  int dataBlockCount;
  int dataMcs;

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  bool burstFound;
  int  burstStart;
  int  burstLength;
  bool dataEvmValid;
  std::string dataError;
  double trainingRho;
  double frequencyErrorHz;
  double evmRmsData;
  int numDataSymbols;

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
