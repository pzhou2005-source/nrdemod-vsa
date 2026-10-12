#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11ax (HE, High Efficiency) OFDM demodulator core -
 * burst/sync/CFO with optional explicitly configured DATA EVM.
 *
 * Pure C++ / STL, mirrors wifi11ac/Wifi11acOfdmCore.hpp. An HE PPDU starts
 * with the exact same Legacy-STF/Legacy-LTF/L-SIG fields as 802.11a/n/ac
 * (IEEE 802.11-2021 27.3.4), so burst detection and the L-LTF matched-filter
 * timing/channel-estimate/CFO recovery reuse the identical, already-
 * validated approach - including the lowpass-FIR 20 MHz subchannel
 * extraction used for 802.11ac 40/80/160 MHz (the legacy preamble is
 * likewise replicated on every 20 MHz subchannel of a wider HE PPDU).
 *
 * Optional DATA EVM supports 20 MHz, one spatial stream, full RU242,
 * 4x HE-LTF and MCS0-11 using the shared HE/EHT training-and-pilot path.
 * dataConfig contains explicit FFT offsets, GI and symbol count; offsets
 * are relative to PPDU start in 20 MSa/s samples. dataResult reports a
 * separately gated EVM ratio. HE-SIG-A/B decoding, OFDMA/MRU, wider DATA,
 * multiple streams and 1x/2x LTF remain unsupported. Sync-only use is unchanged.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "wifi11be/Wifi11beOfdmCore.hpp"

#include <complex>
#include <string>
#include <vector>

namespace wifi11axdemod {

struct Config {
  double sampleRate;             // Hz, input IQ sample rate
  bool   burstSearch;            // search for the burst in the capture
  double burstSearchThresholdDb; // power threshold below the peak, dB
  double syncThreshold;          // minimum L-LTF channel-consistency confidence (0..1) to declare sync

  wifi11bedemod::Config dataConfig;

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

  wifi11bedemod::Result dataResult;

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
