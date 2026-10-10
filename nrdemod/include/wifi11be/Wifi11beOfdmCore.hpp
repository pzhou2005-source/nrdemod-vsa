#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11be (EHT, Extremely High Throughput, Wi-Fi 7) OFDM
 * demodulator core - burst/sync/CFO scope only.
 *
 * Pure C++ / STL, mirrors wifi11ax/Wifi11axOfdmCore.hpp. An EHT PPDU starts
 * with the exact same Legacy-STF/Legacy-LTF/L-SIG fields as 802.11a/n/ac/ax
 * (IEEE 802.11-2024 36.3.4), so burst detection and the L-LTF matched-filter
 * timing/channel-estimate/CFO recovery reuse the identical, already-
 * validated approach - including the lowpass-FIR 20 MHz subchannel-search
 * channelizer used for 802.11ac/ax 40/80/160(/320) MHz (the legacy preamble
 * is likewise replicated on every 20 MHz subchannel of a wider EHT PPDU,
 * verified against real 320 MHz TDC captures at 400 Msps).
 *
 * Scope: ONLY the independently-verified legacy burst/L-LTF-sync/CFO result
 * is produced. EHT-SIG/U-SIG (MRU allocation, MCS, 320 MHz/16x16 MU-MIMO
 * signaling, puncturing pattern) and the EHT-LTF/DATA field (1x/2x/4x LTF,
 * 0.8/1.6/3.2us GI, up to 4096-point FFT at 320 MHz) are NOT decoded by
 * this core at all - EHT's MRU/puncturing structure is an even larger
 * decode problem than HE's OFDMA resource-unit structure, and is
 * intentionally left out of scope rather than guessed at. mcsIndex/
 * bandwidthMHz/numDataSymbols/EVM are therefore not produced; only burst/
 * sync/frequency-error (independently verified via L-LTF channel self-
 * consistency, same as every other native WiFi core here) are reported.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace wifi11bedemod {

struct Config {
  double sampleRate;             // Hz, input IQ sample rate
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

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
