#pragma once
/**
 ****************************************************************************
 *
 * Native IEEE 802.11be (EHT, Extremely High Throughput, Wi-Fi 7) OFDM
 * demodulator core with burst/sync/CFO and explicitly configured DATA EVM.
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
 * Optional DATA EVM supports 20 MHz, one spatial stream, a full RU242,
 * 4x EHT-LTF and MCS0-13. Training and DATA FFT offsets are supplied in
 * 20 MSa/s samples relative to PPDU start; GI is 16, 32 or 64 samples.
 * EHT-LTF estimates each tone independently of DATA. Pilot phase tracking
 * and nearest-constellation DATA decisions produce RMS EVM as a ratio.
 * EHT-SIG/U-SIG decoding, wider DATA bandwidths, MRU/puncturing, multiple
 * spatial streams and 1x/2x LTF remain unsupported. Sync-only use is unchanged.
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

  bool measureDataEvm;
  int dataBandwidthMHz;
  int numSpatialStreams;
  int ehtLtfType;
  int dataMcs;
  int ehtLtfFftOffsetSamples;
  int dataFftOffsetSamples;
  int dataGuardIntervalSamples;
  int numDataSymbols;

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

  bool dataEvmValid;
  std::string dataError;
  double evmRmsData;
  int numDataSymbols;

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

void measureDataEvm(const std::vector<std::complex<double>>& iq, double ppduStartSamples, const Config& config,
    Result& result);
}
