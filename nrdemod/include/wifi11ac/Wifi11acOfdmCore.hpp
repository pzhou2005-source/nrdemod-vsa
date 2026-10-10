#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11ac (VHT) OFDM demodulator core.
 *
 * Pure C++ / STL, mirrors wifi11n/Wifi11nOfdmCore.hpp (802.11n HT). A VHT
 * PPDU starts with the exact same Legacy-STF/Legacy-LTF/L-SIG fields as
 * 802.11a/n (IEEE 802.11-2016 clause 22.3.2), and this legacy preamble
 * (together with VHT-SIG-A) is replicated identically on every 20 MHz
 * subchannel of a wider VHT PPDU (22.3.8.3.2/22.3.8.3.3). For 40/80/160 MHz
 * captures, a lowpass-FIR channelizer (frequency-shift + windowed-sinc
 * filter + decimate) extracts each candidate 20 MHz subchannel in turn and
 * keeps whichever gives the strongest L-LTF matched-filter score as the
 * primary channel; burst detection and the L-LTF matched-filter timing/
 * channel-estimate/CFO recovery then reuse the identical, already-validated
 * 20 MHz approach on that extracted subchannel. VHT-SIG-A (two BPSK OFDM
 * symbols on the legacy 48 data subcarrier grid, same rate-1/2 K=7
 * convolutional code and interleaver as HT-SIG/SIGNAL) is decoded to
 * recover bandwidth/MCS/NSTS, then VHT-STF/VHT-LTF/VHT-SIG-B are skipped
 * over (not independently decoded - the channel estimate from L-LTF is
 * reused) before per-symbol pilot-tracked EVM is measured on the wider
 * 56-subcarrier (52 data + 4 pilot) VHT DATA symbol grid, identical to HT
 * 20 MHz.
 *
 * Scope: burst/sync/VHT-SIG-A (bandwidth/MCS/NSTS) decode works for 20/40/
 * 80/160 MHz captures. DATA-field EVM/symbol-count measurement remains
 * scoped to 20 MHz, single spatial stream (Nsts=1), no STBC: VHT DATA
 * symbols at 40/80/160 MHz span wider FFT grids with bandwidth-specific
 * subcarrier/pilot layouts not yet implemented here, so for those
 * bandwidths (or Nsts>1) this core still reports ok=true with the burst/
 * sync/VHT-SIG-A results (independently verified) but numDataSymbols=0 and
 * rmsEvmPercent=0. MU-MIMO is out of scope entirely.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace wifi11acdemod {

enum Modulation {
  MOD_BPSK   = 0,
  MOD_QPSK   = 1,
  MOD_QAM16  = 2,
  MOD_QAM64  = 3,
  MOD_QAM256 = 4
};

struct Config {
  double sampleRate;             // Hz, input IQ sample rate (20 MHz nominal)
  bool   burstSearch;            // search for the burst in the capture
  double burstSearchThresholdDb; // power threshold below the peak, dB
  double syncThreshold;          // minimum L-LTF channel-consistency confidence (0..1) to declare sync

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

  int mcsIndex;              // decoded VHT-SIG-A MCS index (0..9, BPSK..256QAM) - see caveat below
  int modulation;            // Modulation enum, derived from mcsIndex
  int codingRateNumerator;
  int codingRateDenominator;
  int bandwidthMHz;          // 20/40/80/160, from VHT-SIG-A1 BW field (only 20 MHz is demodulated)
  int nsts;                  // decoded NSTS field (only Nsts=1 is demodulated)
  int numDataSymbols;

  double rmsEvmPercent;
  double peakEvmPercent;
  int    peakEvmSymbolIndex;
  int    peakEvmSubcarrierIndex;

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
