#pragma once
/**
 ****************************************************************************
 *
 * Native Bluetooth LE GFSK (LE1M/LE2M) demodulator core.
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency so it can be unit
 * tested in isolation, mirroring nr/NrOfdmCore.hpp. Implements the genuine
 * PHY-layer receiver chain for the Bluetooth LE uncoded GFSK PHYs
 * (Bluetooth Core Spec v5.x, Vol 6, Part B, 2.3 "BR/EDR/LE modulation" and
 * Part A, 1 "LE physical channel"):
 *
 *   1. burst detection (windowed power threshold)
 *   2. FM discriminator GFSK demodulation (instantaneous frequency)
 *   3. symbol timing recovery from the standard alternating preamble
 *   4. Access Address correlation (bit-sync / "sync word" detection)
 *   5. PHY accuracy metrics: carrier frequency error (F0/Icft) and
 *      frequency deviation (DeltaFAvg/Max/Min), matching the quantities a
 *      vector signal analyzer reports for GFSK modulation accuracy.
 *
 * Does not decode payload bits beyond the Access Address: whitening, CRC
 * and L2CAP framing are protocol-layer concerns, not PHY modulation
 * accuracy, and are out of scope here (same split as the legacy Bebop
 * library's RESULT set, which also stops at the Access Address / header).
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <cstdint>
#include <string>
#include <vector>

namespace btdemod {

enum Phy {
  LE_1M = 1, // 1 Msym/s GFSK
  LE_2M = 2  // 2 Msym/s GFSK
};

struct Config {
  double   sampleRate;             // Hz, input IQ sample rate
  int      phy;                    // Phy enum, selects the 1/2 Msym/s symbol rate
  double   modulationIndex;        // nominal h, 0.5 for Bluetooth LE
  bool     burstSearch;            // search for the burst in the capture
  double   burstSearchThresholdDb; // power threshold below the peak, dB
  // Access Address correlated against to declare "sync word found". Bluetooth Core Spec
  // Vol 6 Part F (Direct Test Mode) fixes AA = 0x71764129 for test packets; override via
  // Config when the capture used a different (e.g. randomized) connection Access Address.
  uint32_t accessAddress;
  double   syncThreshold; // minimum Access Address bit match rate (0..1) to declare sync

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  bool   burstFound;
  int    burstStart;  // first sample of the detected burst
  int    burstLength; // burst length in samples

  bool   syncWordFound;
  double syncWordRho;        // Access Address bit match rate, 0..1
  int    syncPositionSamples; // sample index where the Access Address starts

  double frequencyErrorHz; // carrier frequency offset (F0 / Icft)
  double deltaFAvgHz;      // average |instantaneous frequency deviation|
  double deltaFMaxHz;
  double deltaFMinHz;

  double samplesPerSymbol; // oversampling factor actually used
  int    symbolTimingPhase; // locked sample phase within a symbol period, 0..round(samplesPerSymbol)-1

  std::vector<double> frequencyDeviationHz; // instantaneous frequency, one value per input sample within the burst
  std::vector<int>    accessAddressBits;    // demodulated bits (LSB first) of the Access Address field

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
