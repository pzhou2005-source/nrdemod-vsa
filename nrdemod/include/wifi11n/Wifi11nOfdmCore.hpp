#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11n (HT, 20 MHz, single spatial stream) OFDM demodulator
 * core.
 *
 * Pure C++ / STL, mirrors wifi/WifiOfdmCore.hpp (802.11a). HT mixed-format
 * preambles start with the exact same Legacy-STF/Legacy-LTF/L-SIG fields as
 * 802.11a (IEEE 802.11-2016 clause 19.3.2), so burst detection and the LTF
 * matched-filter timing/channel-estimate/CFO recovery reuse the identical
 * approach already validated against real 802.11a captures. HT-SIG (two
 * BPSK/QBPSK OFDM symbols on the legacy 48 data subcarriers, same rate-1/2
 * K=7 convolutional code and interleaver as the SIGNAL field) is decoded to
 * recover MCS/bandwidth/PSDU length, then the HT-STF/HT-LTF realign the
 * channel estimate for the wider 56-subcarrier (52 data + 4 pilot) HT DATA
 * symbol grid before per-symbol pilot-tracked EVM is measured.
 *
 * Scope: 20 MHz, single spatial stream (Nss=1), no space-time block coding,
 * no 40 MHz / short-GI distinction beyond what HT-SIG reports (bandwidth
 * and GI are read back from the decoded header, not independently
 * verified). MIMO (multiple spatial streams) is out of scope.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace wifi11ndemod {

enum Modulation {
  MOD_BPSK  = 0,
  MOD_QPSK  = 1,
  MOD_QAM16 = 2,
  MOD_QAM64 = 3
};

struct Config {
  double sampleRate;             // Hz, input IQ sample rate (20 MHz nominal)
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
  double syncRho;             // L-LTF channel-consistency confidence, 0..1
  int    timingOffsetSamples; // sample index (from burstStart) of the first L-STF sample

  double frequencyErrorHz; // carrier frequency offset from the two L-LTF copies' residual phase

  int mcsIndex;            // decoded HT-SIG MCS index (0..7 for Nss=1, BPSK..64QAM)
  int modulation;           // Modulation enum, derived from mcsIndex
  int codingRateNumerator;
  int codingRateDenominator;
  int bandwidthMHz;         // 20 or 40, from HT-SIG CBW bit (only 20 MHz is demodulated)
  int psduLengthBytes;      // HT_LENGTH field
  int numDataSymbols;

  double rmsEvmPercent;
  double peakEvmPercent;
  int    peakEvmSymbolIndex;
  int    peakEvmSubcarrierIndex;

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
