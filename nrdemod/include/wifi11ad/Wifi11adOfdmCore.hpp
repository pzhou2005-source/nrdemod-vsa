#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11ad (DMG, 60 GHz / WiGig) demodulator core - burst
 * detection only.
 *
 * Pure C++ / STL. 802.11ad uses a fundamentally different PHY from every
 * other WiFi variant in this codebase: a single-carrier (SC) modulation
 * with a Golay-complementary-sequence (Ga128/Gb128) STF/CE preamble (IEEE
 * 802.11-2016 clause 20.5.3), not an OFDM L-STF/L-LTF structure at all - so
 * none of the existing wifi/wifi11n/wifi11ac/wifi11ax sync code applies.
 *
 * Scope: ONLY power-threshold burst detection (reusing the same windowed-
 * power algorithm already validated by every other native core in this
 * codebase) is implemented. The Golay STF/CE correlation sync, CE-field
 * channel estimate, and SC/OFDM DATA field decode are NOT implemented:
 * the exact Ga128/Gb128 sequence content could not be obtained from a
 * source this project could independently verify against real captures
 * (unlike the 802.11a LTF table or the CCK codeword formula, both of which
 * were cross-checked against multiple independent sources AND real TDC
 * captures before being hardcoded) - rather than guess at an unverified
 * 128-chip sequence and silently risk producing a confidently-wrong sync
 * result, this core deliberately stops at the one thing it CAN verify:
 * burst presence via power envelope, which needs no sequence content at
 * all. mcsIndex/syncRho/frequencyErrorHz/EVM are NOT produced.
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

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  bool burstFound;
  int  burstStart;
  int  burstLength;

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
