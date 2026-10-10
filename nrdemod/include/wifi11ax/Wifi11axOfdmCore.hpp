#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11ax (HE, High Efficiency) OFDM demodulator core -
 * burst/sync/CFO scope only.
 *
 * Pure C++ / STL, mirrors wifi11ac/Wifi11acOfdmCore.hpp. An HE PPDU starts
 * with the exact same Legacy-STF/Legacy-LTF/L-SIG fields as 802.11a/n/ac
 * (IEEE 802.11-2021 27.3.4), so burst detection and the L-LTF matched-filter
 * timing/channel-estimate/CFO recovery reuse the identical, already-
 * validated approach - including the lowpass-FIR 20 MHz subchannel
 * extraction used for 802.11ac 40/80/160 MHz (the legacy preamble is
 * likewise replicated on every 20 MHz subchannel of a wider HE PPDU).
 *
 * Scope: ONLY the independently-verified legacy burst/L-LTF-sync/CFO result
 * is produced. HE-SIG-A/B (RU allocation, MCS, GI+LTF-size signaling,
 * OFDMA resource-unit structure) and the HE-LTF/DATA field (1x/2x/4x LTF,
 * 0.8/1.6/3.2us GI, 256-point FFT at 20 MHz scaling up to 2048-point at
 * 160 MHz, different active/pilot subcarrier layout per RU) are NOT decoded
 * by this core at all - HE's resource-unit/OFDMA structure is a materially
 * different, much larger decode problem than VHT's single-user SIG-A, and
 * is intentionally left out of scope rather than guessed at. mcsIndex/
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

namespace wifi11axdemod {

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
