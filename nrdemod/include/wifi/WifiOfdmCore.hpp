#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11a OFDM PHY demodulator core.
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency, mirrors the
 * nr/NrOfdmCore.hpp, bt/BtGfskCore.hpp and uwb/UwbHrpCore.hpp pattern.
 * Implements a genuine, from-first-principles 802.11a receiver (IEEE
 * 802.11-2016 clause 17):
 *
 *   1. burst detection (windowed power threshold, same technique as the
 *      other native cores)
 *   2. coarse preamble timing via periodic autocorrelation at the 16
 *      sample Short Training Field (STF) repetition period - this needs
 *      no knowledge of the literal STF waveform, only that it repeats
 *      (same self-similarity principle as the UWB core's SYNC search)
 *   3. fine timing + channel estimate + carrier frequency offset from the
 *      two Long Training Field (LTF) repetitions: the correct alignment
 *      is the one where both LTF copies, divided by the known LTF
 *      frequency-domain sequence, agree with each other (a genuine,
 *      self-verifying consistency metric - "SyncRho" below)
 *   4. SIGNAL field decode (pilot-corrected BPSK, rate-1/2 Viterbi) to
 *      recover the modulation / coding rate / PSDU length, exactly as a
 *      real receiver must before it can process the DATA field
 *   5. per DATA-symbol pilot-based common phase error correction and
 *      per-subcarrier equalization, with EVM accumulated against hard
 *      decisions - the same modulation-accuracy quantity a vector signal
 *      analyzer (and the legacy Bebop library's EvmRms/EvmPeak) reports.
 *
 * Does not decode the DATA field's convolutionally-coded payload bits
 * (depuncturing / Viterbi / descrambling / CRC for the actual PSDU
 * payload is a MAC-layer concern, not PHY modulation accuracy) - only the
 * tiny 48 bit SIGNAL field is Viterbi-decoded, since that is required to
 * even know how many DATA symbols exist and with what modulation.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace wifidemod {

enum Modulation {
  MOD_BPSK  = 0,
  MOD_QPSK  = 1,
  MOD_QAM16 = 2,
  MOD_QAM64 = 3
};

struct Config {
  double sampleRate;             // Hz, input IQ sample rate (802.11a: 20 MHz nominal)
  bool   burstSearch;            // search for the burst in the capture
  double burstSearchThresholdDb; // power threshold below the peak, dB
  double syncThreshold;          // minimum LTF channel-consistency confidence (0..1) to declare sync

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  bool burstFound;
  int  burstStart;
  int  burstLength;

  bool   syncFound;
  double syncRho;             // LTF channel-consistency confidence, 0..1 (1 = perfect)
  int    timingOffsetSamples; // sample index (from burstStart) of the first STF sample

  double frequencyErrorHz; // carrier frequency offset from the two LTF copies' residual phase

  int modulation;         // Modulation enum, decoded from the SIGNAL field
  int codingRateNumerator;
  int codingRateDenominator;
  int dataRateMbps;
  int psduLengthBytes;
  int numDataSymbols;

  double rmsEvmPercent;
  double peakEvmPercent;
  int    peakEvmSymbolIndex;     // 0 based, counts only DATA symbols (SIGNAL excluded)
  int    peakEvmSubcarrierIndex; // 0..47, index within the 48 data subcarriers

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
