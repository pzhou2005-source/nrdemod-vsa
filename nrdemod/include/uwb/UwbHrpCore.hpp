#pragma once
/**
 ****************************************************************************
 *
 * Native UWB (IEEE 802.15.4z HRP) impulse-radio burst/preamble demodulator
 * core.
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency, mirrors the
 * nr/NrOfdmCore.hpp and bt/BtGfskCore.hpp pattern. Implements a genuine
 * correlation receiver for the HRP-UWB PHY (IEEE 802.15.4z / 802.15.4-2020,
 * clause 15):
 *
 *   1. burst detection (windowed power threshold, same technique as the
 *      Bluetooth GFSK core)
 *   2. SYNC field acquisition via periodic autocorrelation: the SYNC
 *      preamble repeats the same ternary code every "SyncLength" symbol
 *      periods, so correlating the signal against a delayed copy of
 *      itself at the nominal preamble symbol period produces a genuine,
 *      measurable periodicity peak without needing the literal per-code
 *      ternary sequence tables (clause 15.2.6.2) - the same self-similarity
 *      principle any impulse-radio preamble detector exploits.
 *   3. carrier frequency error from the residual phase rotation across
 *      the detected SYNC period (BPSK-modulated preamble symbols cancel
 *      under magnitude-squared combining, leaving only the carrier
 *      rotation, the same trick the GFSK core's F0 estimator uses).
 *
 * Does not decode PHR/PSDU payload bits (ternary-code correlation,
 * Reed-Solomon/convolutional FEC and scrambling are protocol-layer
 * concerns well beyond PHY burst/sync accuracy) - deliberately scoped the
 * same way as the native Bluetooth LE core (BurstFound/SyncRho/
 * FrequencyError, matching the subset of the legacy Bebop library's own
 * RESULT set that are pure modulation-accuracy quantities).
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <string>
#include <vector>

namespace uwbdemod {

enum PhyMode {
  BPRF = 0, // base pulse repetition frequency (499.2 MHz chip rate)
  HPRF = 1  // high pulse repetition frequency (499.2 MHz chip rate, different burst structure)
};

struct Config {
  double sampleRate;             // Hz, input IQ sample rate
  int    phyMode;                // PhyMode enum
  int    syncLength;             // number of SYNC preamble symbol repetitions (e.g. 64)
  double chipRateHz;             // nominal UWB chip rate, 499.2 MHz per the spec
  bool   burstSearch;            // search for the burst in the capture
  double burstSearchThresholdDb; // power threshold below the peak, dB
  double syncThreshold;          // minimum normalized autocorrelation to declare sync, 0..1

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  bool   burstFound;
  int    burstStart;
  int    burstLength;

  bool   syncFound;
  double syncRho;           // normalized periodic autocorrelation magnitude, 0..1
  int    syncPositionSamples;
  double preambleSymbolPeriodSamples; // measured period of the repeating SYNC code

  double frequencyErrorHz; // carrier frequency offset from the SYNC field's residual rotation

  Result();
};

Result demodulate(const std::vector<std::complex<double> >& iq, const Config& config);

}
