/**
 ****************************************************************************
 *
 * Flat C ABI around the native WiFi (802.11a/b/n/ac/ax/ad/be), Bluetooth LE
 * (GFSK) and UWB (802.15.4z HRP) demodulator cores.
 *
 * Same conventions as nrdemod_c.h (see that file): no C++ types/exceptions
 * at the boundary, results read back by name through an opaque handle, so
 * this can be loaded with ctypes from Python on Linux and Windows.
 *
 * One shared DemodConfig covers every family; each family only reads the
 * subset of fields relevant to it (see per-field comments below). Call
 * demod_config_default() for a given family first to get sane defaults.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#ifndef NRDEMOD_DEMOD_C_H
#define NRDEMOD_DEMOD_C_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(NRDEMOD_BUILD_SHARED)
#    define DEMOD_API __declspec(dllexport)
#  else
#    define DEMOD_API __declspec(dllimport)
#  endif
#  define DEMOD_CALL __cdecl
#else
#  define DEMOD_API __attribute__((visibility("default")))
#  define DEMOD_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DEMOD_ABI_VERSION 1

/** Selects which native core demod_demodulate() dispatches to. */
enum DemodFamily {
  DEMOD_WIFI_A  = 0, /* 802.11a, legacy OFDM, full SIGNAL decode + EVM */
  DEMOD_WIFI_N  = 1, /* 802.11n (HT), 20 MHz Nss=1, HT-SIG decode + EVM */
  DEMOD_WIFI_AC = 2, /* 802.11ac (VHT), 20/40/80/160 MHz sync, 20 MHz EVM */
  DEMOD_WIFI_AX = 3, /* 802.11ax (HE), burst/sync/CFO only */
  DEMOD_WIFI_AD = 4, /* 802.11ad (DMG/WiGig), burst detection only */
  DEMOD_WIFI_B  = 5, /* 802.11b DSSS/CCK, Barker sync + CCK DATA EVM */
  DEMOD_WIFI_BE = 6, /* 802.11be (EHT), burst/sync/CFO only */
  DEMOD_BT_LE   = 7, /* Bluetooth LE1M/LE2M GFSK */
  DEMOD_UWB_HRP = 8  /* 802.15.4z HRP-UWB */
};

enum DemodStatus {
  DEMOD_OK              =  0,
  DEMOD_ERR_NULL_ARG    = -1,
  DEMOD_ERR_UNKNOWN_KEY = -2,
  DEMOD_ERR_SIZE        = -3,
  DEMOD_ERR_FAILED      = -4,
  DEMOD_ERR_ABI         = -5,
  DEMOD_ERR_FAMILY      = -6
};

/**
 * Union of every native core's Config fields. Which fields are read is
 * determined entirely by the DemodFamily passed to demod_demodulate():
 *
 *   sampleRate, burstSearch, burstSearchThresholdDb, syncThreshold
 *       - read by every WiFi family and DEMOD_UWB_HRP (DEMOD_WIFI_AD only
 *         reads burstSearchThresholdDb; its sampleRate/burstSearch/
 *         syncThreshold fields do not exist in that core and are ignored).
 *   phy, modulationIndex, accessAddress
 *       - DEMOD_BT_LE only. phy: 1 = LE1M, 2 = LE2M.
 *   phyMode, syncLength, chipRateHz
 *       - DEMOD_UWB_HRP only. phyMode: 0 = BPRF, 1 = HPRF.
 */
typedef struct DemodConfig {
  int      abiVersion; /* must be DEMOD_ABI_VERSION */

  double   sampleRate;
  int      burstSearch;
  double   burstSearchThresholdDb;
  double   syncThreshold;

  int      phy;
  double   modulationIndex;
  uint32_t accessAddress;

  int      phyMode;
  int      syncLength;
  double   chipRateHz;
} DemodConfig;

/** Opaque demodulation result, tagged internally with the DemodFamily it came from. */
typedef struct DemodResult DemodResult;

DEMOD_API const char* DEMOD_CALL demod_version(void);
DEMOD_API int         DEMOD_CALL demod_abi_version(void);

/** Populates \p cfg with that family's core defaults. */
DEMOD_API int DEMOD_CALL demod_config_default(int family, DemodConfig* cfg);

/**
 * Demodulates \p numSamples interleaved float32 I/Q pairs with the given
 * family's native core. Always returns a handle (even on failure) unless
 * allocation fails or \p family is invalid; inspect demod_result_ok() /
 * demod_result_error().
 */
DEMOD_API DemodResult* DEMOD_CALL demod_demodulate(int family, const float* iq, size_t numSamples,
                                                   const DemodConfig* cfg);

/** Same, but from interleaved float64 pairs (converted internally). */
DEMOD_API DemodResult* DEMOD_CALL demod_demodulate64(int family, const double* iq, size_t numSamples,
                                                     const DemodConfig* cfg);

DEMOD_API void DEMOD_CALL demod_result_free(DemodResult* r);

DEMOD_API int         DEMOD_CALL demod_result_ok(const DemodResult* r);
DEMOD_API const char* DEMOD_CALL demod_result_error(const DemodResult* r);

/** Scalar access by field name, e.g. "syncRho", "burstFound", "rmsEvmPercent". */
DEMOD_API int DEMOD_CALL demod_result_get_double(const DemodResult* r, const char* name, double* out);
DEMOD_API int DEMOD_CALL demod_result_get_int(const DemodResult* r, const char* name, int* out);
DEMOD_API int DEMOD_CALL demod_result_get_bool(const DemodResult* r, const char* name, int* out);

/**
 * Vector access by field name (currently only populated by DEMOD_BT_LE's
 * frequencyDeviationHz [real] and accessAddressBits [int]).
 */
DEMOD_API long DEMOD_CALL demod_result_vector_size(const DemodResult* r, const char* name);
DEMOD_API int  DEMOD_CALL demod_result_vector_copy(const DemodResult* r, const char* name,
                                                   double* dst, size_t count);
DEMOD_API long DEMOD_CALL demod_result_ivector_size(const DemodResult* r, const char* name);
DEMOD_API int  DEMOD_CALL demod_result_ivector_copy(const DemodResult* r, const char* name,
                                                    int* dst, size_t count);

/**
 * Introspection: NUL separated, double NUL terminated list of field names
 * for the given family. kind: 0 = bool, 1 = int, 2 = double, 3 = vector<double>,
 * 4 = vector<int>.
 */
DEMOD_API const char* DEMOD_CALL demod_field_names(int family, int kind);

#ifdef __cplusplus
}
#endif

#endif /* NRDEMOD_DEMOD_C_H */
