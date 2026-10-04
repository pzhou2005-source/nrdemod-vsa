/**
 ****************************************************************************
 *
 * Flat C ABI around the native 5G NR CP-OFDM / DFT-s-OFDM demodulator core.
 *
 * Designed to be loaded with ctypes from Python on Linux and Windows, so it
 * deliberately avoids C++ types, exceptions and STL containers at the
 * boundary. Results are kept behind an opaque handle and read back by name.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#ifndef NRDEMOD_C_H
#define NRDEMOD_C_H

#include <stddef.h>

#if defined(_WIN32)
#  if defined(NRDEMOD_BUILD_SHARED)
#    define NRDEMOD_API __declspec(dllexport)
#  else
#    define NRDEMOD_API __declspec(dllimport)
#  endif
#  define NRDEMOD_CALL __cdecl
#else
#  define NRDEMOD_API __attribute__((visibility("default")))
#  define NRDEMOD_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** ABI revision, bumped whenever NrdConfig changes layout. */
#define NRDEMOD_ABI_VERSION 1

enum NrdModulation {
  NRD_MOD_AUTO   = 0,
  NRD_MOD_QPSK   = 1,
  NRD_MOD_QAM16  = 2,
  NRD_MOD_QAM64  = 3,
  NRD_MOD_QAM256 = 4
};

enum NrdFrequencyRange {
  NRD_FR1 = 1,
  NRD_FR2 = 2
};

enum NrdStatus {
  NRD_OK              =  0,
  NRD_ERR_NULL_ARG    = -1,
  NRD_ERR_UNKNOWN_KEY = -2,
  NRD_ERR_SIZE        = -3,
  NRD_ERR_FAILED      = -4,
  NRD_ERR_ABI         = -5
};

/**
 * Mirror of nrdemod::Config. Booleans are int (0/1) to keep the layout
 * predictable across compilers. Fill with nrd_config_default() first.
 */
typedef struct NrdConfig {
  int    abiVersion;                    /* must be NRDEMOD_ABI_VERSION */
  int    numerology;                    /* mu = 0..3, SCS = 15 kHz * 2^mu */
  int    fftSize;                       /* 0 = derive from sample rate / bandwidth */
  int    frequencyRange;                /* NrdFrequencyRange */
  int    channelBandwidthMHz;           /* 0 = unknown */
  int    numResourceBlocks;             /* 0 = whole carrier */
  int    resourceBlockOffset;
  int    extendedCp;
  int    modulation;                    /* NrdModulation */
  int    maxSymbols;                    /* 0 = all symbols in the capture */
  int    burstSearch;
  double burstSearchThresholdDb;
  int    timingSearch;
  int    syncSearchSymbols;
  int    cfoCorrection;
  double carrierOffsetHz;
  int    equalize;
  int    equalizerIterations;
  int    phaseTracking;
  int    timingTracking;
  int    amplitudeTracking;
  int    transformPrecoding;
  int    dcPunctured;
  int    removeIqOffset;
  double symbolTimingAdjustmentPercent;
  int    resample;
  double sampleRate;                    /* Hz */

  int    dmrsExclude;
  int    dmrsMappingTypeB;
  int    dmrsTypeAPosition;
  int    dmrsAdditionalPositions;
  int    dmrsDoubleSymbol;

  const int* excludedSymbols;           /* may be NULL */
  int        numExcludedSymbols;
} NrdConfig;

/** Opaque demodulation result. */
typedef struct NrdResult NrdResult;

NRDEMOD_API const char* NRDEMOD_CALL nrd_version(void);
NRDEMOD_API int         NRDEMOD_CALL nrd_abi_version(void);

/** Populates \p cfg with the core defaults. */
NRDEMOD_API int NRDEMOD_CALL nrd_config_default(NrdConfig* cfg);

/**
 * Demodulates \p numSamples interleaved float32 I/Q pairs.
 * \p iq points to 2 * numSamples floats. Always returns a handle (even on
 * failure) unless allocation fails; inspect "ok" / nrd_result_error().
 */
NRDEMOD_API NrdResult* NRDEMOD_CALL nrd_demodulate(const float* iq, size_t numSamples,
                                                   const NrdConfig* cfg);

/** Same, but from interleaved float64 pairs (converted internally). */
NRDEMOD_API NrdResult* NRDEMOD_CALL nrd_demodulate64(const double* iq, size_t numSamples,
                                                     const NrdConfig* cfg);

NRDEMOD_API void NRDEMOD_CALL nrd_result_free(NrdResult* r);

/** 1 when the demodulation succeeded. */
NRDEMOD_API int         NRDEMOD_CALL nrd_result_ok(const NrdResult* r);
NRDEMOD_API const char* NRDEMOD_CALL nrd_result_error(const NrdResult* r);

/** Scalar access by field name, e.g. "rmsEvmPercent", "fftSize". */
NRDEMOD_API int NRDEMOD_CALL nrd_result_get_double(const NrdResult* r, const char* name, double* out);
NRDEMOD_API int NRDEMOD_CALL nrd_result_get_int(const NrdResult* r, const char* name, int* out);

/**
 * Vector access by field name. Real vectors use nrd_result_vector_*, complex
 * ones nrd_result_cvector_* (copied as interleaved re/im).
 * Returns the element count, or a negative NrdStatus.
 */
NRDEMOD_API long NRDEMOD_CALL nrd_result_vector_size(const NrdResult* r, const char* name);
NRDEMOD_API int  NRDEMOD_CALL nrd_result_vector_copy(const NrdResult* r, const char* name,
                                                     double* dst, size_t count);
NRDEMOD_API long NRDEMOD_CALL nrd_result_ivector_size(const NrdResult* r, const char* name);
NRDEMOD_API int  NRDEMOD_CALL nrd_result_ivector_copy(const NrdResult* r, const char* name,
                                                      int* dst, size_t count);
NRDEMOD_API long NRDEMOD_CALL nrd_result_cvector_size(const NrdResult* r, const char* name);
NRDEMOD_API int  NRDEMOD_CALL nrd_result_cvector_copy(const NrdResult* r, const char* name,
                                                      double* dst, size_t count);

/** Introspection: NUL separated, double NUL terminated list of field names. */
NRDEMOD_API const char* NRDEMOD_CALL nrd_field_names(int kind); /* 0=int 1=double 2=vector 3=cvector 4=ivector */

/** 3GPP TS 38.101 carrier RB count, 0 when the combination is undefined. */
NRDEMOD_API int NRDEMOD_CALL nrd_carrier_resource_blocks(int frequencyRange, int bandwidthMHz, int numerology);

/**
 * Generates a synthetic NR burst for self test / correlation sanity checks.
 * Writes 2 * (*numSamples) interleaved floats into a buffer owned by the
 * library; call nrd_free_signal() when done.
 */
NRDEMOD_API float* NRDEMOD_CALL nrd_generate_test_signal(const NrdConfig* cfg, int numSymbols,
                                                         int leadingSamples, unsigned seed,
                                                         size_t* numSamples);
NRDEMOD_API void NRDEMOD_CALL nrd_free_signal(float* samples);

#ifdef __cplusplus
}
#endif

#endif /* NRDEMOD_C_H */
