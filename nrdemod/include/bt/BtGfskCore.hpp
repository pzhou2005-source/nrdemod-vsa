#pragma once
/**
 ****************************************************************************
 *
 * Native Bluetooth GFSK demodulator core.
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency so it can be unit
 * tested in isolation, mirroring nr/NrOfdmCore.hpp. Implements the genuine
 * PHY-layer receiver chain for every Bluetooth variant whose physical
 * channel modulation is GFSK (Bluetooth Core Spec v5.x, Vol 6, Part B, 2.3
 * "BR/EDR/LE modulation" and Part A, 1 "LE physical channel"):
 *
 *   1. burst detection (windowed power threshold)
 *   2. FM discriminator GFSK demodulation (instantaneous frequency)
 *   3. symbol timing recovery from the standard alternating preamble
 *   4. Access Address correlation (bit-sync / "sync word" detection) -
 *      LE1M/LE2M only, see Phy enum for the scope limits on BDR/EDR/Coded
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
    LE_1M = 1,           // 1 Msym/s GFSK
    LE_2M = 2,           // 2 Msym/s GFSK
    BDR_GFSK = 3,        // classic Basic Rate, 1 Msym/s GFSK (Core Spec Vol 2 Part B 2.1), h ~ 0.3
    EDR_GFSK_HEADER = 4, // classic EDR packet's leading GFSK access-code+header, 1 Msym/s, h ~ 0.3
    LE_CODED_S8 = 5      // LE Coded (S=8), 1 Msym/s GFSK on air - same PHY as LE_1M, FEC-coded bits
  };

  struct Config {
    double sampleRate;             // Hz, input IQ sample rate
    int phy;                       // Phy enum, selects the symbol rate (and sync scope for BDR_GFSK)
    double modulationIndex;        // nominal h: 0.5 for Bluetooth LE, ~0.3 for classic BDR
    bool steadyFrequencyDeviation;
    bool burstSearch;              // search for the burst in the capture
    double burstSearchThresholdDb; // power threshold below the peak, dB
    // Access Address correlated against to declare "sync word found". Bluetooth Core Spec
    // Vol 6 Part F (Direct Test Mode) fixes AA = 0x71764129 for test packets; override via
    // Config when the capture used a different (e.g. randomized) connection Access Address.
    // Not used for BDR_GFSK/EDR_GFSK_HEADER: classic Bluetooth's 64-bit sync word is derived
    // from the 24-bit LAP via a dedicated generator procedure, not a literal bit pattern -
    // deliberately left undecoded here rather than guessed at (same policy as the 802.11ad
    // Golay sequence gap). Not used for LE_CODED_S8 either: the on-air Access Address bits are
    // FEC pattern-mapped (coded), not the literal 32-bit value correlated for LE1M/LE2M.
    uint32_t accessAddress;
    double syncThreshold; // minimum Access Address bit match rate (0..1) to declare sync

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
