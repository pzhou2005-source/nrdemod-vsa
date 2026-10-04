#pragma once
/**
 ****************************************************************************
 *
 * Native 5G NR CP-OFDM / DFT-s-OFDM demodulator core (3GPP TS 38.211).
 *
 * Pure C++ / STL: no libdemod, UNO or MKL dependency so it can be unit
 * tested in isolation. Arbitrary input sample rates are resampled to the
 * FFT grid Fs = FftSize * SCS.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include <complex>
#include <cstddef>
#include <string>
#include <vector>

namespace nrdemod {

enum Modulation {
  MOD_AUTO   = 0,
  MOD_QPSK   = 1,
  MOD_QAM16  = 2,
  MOD_QAM64  = 3,
  MOD_QAM256 = 4
};

enum FrequencyRange {
  FR1 = 1,
  FR2 = 2
};

struct DmrsConfig {
  bool exclude;             // drop DM-RS symbols from EVM / equalizer statistics
  bool mappingTypeB;        // PUSCH mapping type B (type A otherwise)
  int  typeAPosition;       // l0 for mapping type A: 2 or 3
  int  additionalPositions; // 0..3
  bool doubleSymbol;        // two consecutive DM-RS symbols

  DmrsConfig();
};

struct Config {
  int    numerology;        // mu = 0..3 (SCS = 15 kHz * 2^mu)
  int    fftSize;           // 0 = derive from sample rate / bandwidth
  int    frequencyRange;    // FrequencyRange, selects the carrier RB table
  int    channelBandwidthMHz; // 0 = unknown (RB grid centred, 90 % fill)
  int    numResourceBlocks; // 0 = whole carrier (or 90 % of fftSize when bandwidth unknown)
  int    resourceBlockOffset; // first allocated RB relative to the carrier's lowest RB
  bool   extendedCp;        // only valid for mu = 2
  int    modulation;        // Modulation enum
  int    maxSymbols;        // 0 = all symbols in the capture
  bool   burstSearch;       // search for burst boundaries in the capture
  double burstSearchThresholdDb; // power threshold for burst search (dB below peak)
  bool   timingSearch;      // coarse symbol timing via CP correlation
  int    syncSearchSymbols; // symbols accumulated in the coarse timing search
  bool   cfoCorrection;     // carrier frequency offset via CP correlation
  double carrierOffsetHz;   // known offset of the carrier centre from baseband DC; an NR
                            // carrier may sit half a subcarrier off the tuned centre
  bool   equalize;          // decision directed per subcarrier equalizer
  int    equalizerIterations;
  bool   phaseTracking;     // per symbol common phase error correction
  bool   timingTracking;    // per symbol linear phase (timing drift) correction, CP-OFDM only
  bool   amplitudeTracking; // per symbol gain correction, CP-OFDM only
  bool   transformPrecoding;// DFT-s-OFDM (PUSCH with transform precoding)
  bool   dcPunctured;       // DC subcarrier carries no data
  bool   removeIqOffset;    // remove IQ DC offset before demodulation
  double symbolTimingAdjustmentPercent; // FFT window shift in % of the CP (negative = earlier)
  bool   resample;          // allow resampling when Fs != fftSize * SCS
  double sampleRate;        // Hz
  DmrsConfig dmrs;
  std::vector<int> excludedSymbols; // additional symbol indices (0 based) excluded from EVM

  Config();
};

struct Result {
  bool        ok;
  std::string error;

  int    fftSize;
  double resampleRate;       // Fs actually used for the FFT grid
  int    numResourceBlocks;
  int    carrierResourceBlocks; // 0 when the bandwidth is unknown
  int    firstSubcarrierIndex;  // k of the first allocated subcarrier (DC = 0)
  int    numSymbols;
  int    timingOffset;       // samples (input rate) from capture start to first CP start
  int    slotStartSymbol;    // index of the first symbol assumed to start a slot
  double frequencyErrorHz;
  int    detectedModulation; // Modulation enum (never MOD_AUTO)
  double rmsEvmPercent;
  double peakEvmPercent;
  double rmsEvmDb;
  int    burstStart;          // first sample of the detected burst
  int    burstLength;         // burst length in samples
  double syncCorrelation;     // normalised CP correlation magnitude
  double normalizationFactor; // 1/gain applied to the constellation
  double dataPowerDb;         // average data subcarrier power (dB)
  double burstPowerDb;        // average burst power (dB)
  double iqOffsetDb;          // IQ DC offset power (dB relative to signal)
  double iqGainImbalanceDb;   // IQ gain imbalance (dB)
  double iqQuadratureErrorDeg;// IQ quadrature error (degrees)
  // IQ timing skew (seconds); without a gain/quadrature imbalance to anchor the 90 degree
  // constellation ambiguity the sign is not observable and the magnitude is reported
  double iqTimingSkewSec;
  double commonPhaseErrorDeg; // RMS common phase error (degrees)
  double dmrsEvmPercent;      // EVM on DM-RS symbols (percent)
  double dmrsPowerDb;         // DM-RS average power (dB)
  double flatnessRippleRange1Db;   // spectral flatness ripple range 1 (dB)
  double flatnessRippleRange2Db;   // spectral flatness ripple range 2 (dB)
  double flatnessMaxRange1MinRange2Db; // flatness max(range1) - min(range2) (dB)
  double flatnessMaxRange2MinRange1Db; // flatness max(range2) - min(range1) (dB)
  int    peakEvmSymbol;       // symbol index of the peak EVM
  int    peakEvmSubcarrier;   // subcarrier index of the peak EVM
  int    numSlots;            // number of complete slots demodulated
  int    firstSlotIndex;      // estimated slot index of the first demodulated slot

  std::vector<int>    dmrsSymbols;       // symbol indices treated as DM-RS
  std::vector<double> evmPerSymbol;      // percent
  std::vector<double> evmPerSubcarrier;  // percent
  std::vector<double> spectrumDb;        // fftSize bins, -Fs/2 .. +Fs/2
  std::vector<std::complex<double> > constellation;   // equalized, symbol major
  std::vector<std::complex<double> > reference;       // hard decisions
  std::vector<std::complex<double> > channelEstimate; // per active subcarrier
  std::vector<std::complex<double> > errorVector;       // raw error vector per data RE
  std::vector<std::complex<double> > commonPhaseErrorPerSymbol; // unit phasor per symbol
  std::vector<double> evmPerSlot;            // RMS EVM per slot (percent)
  std::vector<double> peakEvmPerSlot;        // peak EVM per slot (percent)
  std::vector<double> peakEvmPerSymbol;      // peak EVM per symbol (percent)
  std::vector<double> peakEvmPerSubcarrier;  // peak EVM per subcarrier (percent)
  std::vector<double> evmPerResourceBlock;   // EVM per resource block (percent)
  std::vector<double> powerPerResourceBlockDb; // power per RB (dB)
  std::vector<double> channelImpulseResponseDb; // CIR magnitude (dB)
  std::vector<double> channelPhaseDifference;   // channel phase difference per subcarrier
  std::vector<double> channelTypePerSymbol;     // channel type classification per symbol

  Result();
};

int normalCpLength(int fftSize);
int longCpLength(int fftSize, int numerology);
int extendedCpLength(int fftSize);
int symbolsPerSlot(bool extendedCp);
int maxResourceBlocks(int fftSize);

/** 3GPP TS 38.101-1/2 table 5.3.2-1, 0 when the combination is not defined. */
int carrierResourceBlocks(int frequencyRange, int channelBandwidthMHz, int numerology);

/** DM-RS symbol indices within a 14 symbol slot (TS 38.211 tables 6.4.1.1.3-3/4). */
std::vector<int> dmrsSymbolsInSlot(const DmrsConfig& dmrs);

/** In-place radix-2 FFT, n must be a power of two. */
void fft(std::vector<std::complex<double> >& data, bool inverse);

/** In-place DFT of arbitrary length (Bluestein), unnormalised like fft(). */
void dft(std::vector<std::complex<double> >& data, bool inverse);

/**
 * External FFT provider callback. When installed, dft() delegates to this
 * instead of the built-in Cooley-Tukey / Bluestein implementation.
 * Signature: in-place complex DFT of \p n points, forward when \p inverse
 * is false. The output is NOT normalised (same convention as the built-in).
 */
typedef void (*FftProviderFn)(std::complex<double>* data, size_t n, bool inverse);

/** Install an external FFT (e.g. MKL). Pass NULL to revert to built-in. */
void setFftProvider(FftProviderFn fn);

/** Currently active provider, or NULL when using the built-in. */
FftProviderFn getFftProvider();

/** Windowed sinc resampler, arbitrary ratio. */
std::vector<std::complex<double> > resample(const std::vector<std::complex<double> >& in,
    double sampleRateIn, double sampleRateOut);

/** Hard decision on a unit-average-power square QAM grid. */
std::complex<double> hardDecision(const std::complex<double>& z, Modulation mod);

Result demodulate(const std::complex<float>* iq, size_t numSamples, const Config& cfg);

/**
 * Test helper: generate an NR burst with random symbols on the active
 * subcarriers (DFT-s-OFDM when cfg.transformPrecoding). Returns time domain
 * samples at cfg.sampleRate (resampled when it is not the FFT grid rate).
 */
std::vector<std::complex<float> > generateTestSignal(const Config& cfg, int numSymbols,
    int leadingSamples, unsigned seed, std::vector<std::complex<double> >* referenceOut);

}
