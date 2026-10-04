/**
 ****************************************************************************
 *
 * Implementation of the flat C ABI declared in nrdemod/nrdemod_c.h.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "nrdemod/nrdemod_c.h"
#include "nr/NrOfdmCore.hpp"

#include <cstring>
#include <exception>
#include <new>
#include <string>
#include <vector>

#define NRDEMOD_VERSION_STRING "1.0.0"

/* --- field tables ------------------------------------------------------- */

#define NRD_INT_FIELDS(X)        \
  X(fftSize)                     \
  X(numResourceBlocks)           \
  X(carrierResourceBlocks)       \
  X(firstSubcarrierIndex)        \
  X(numSymbols)                  \
  X(timingOffset)                \
  X(slotStartSymbol)             \
  X(detectedModulation)          \
  X(burstStart)                  \
  X(burstLength)                 \
  X(peakEvmSymbol)               \
  X(peakEvmSubcarrier)           \
  X(numSlots)                    \
  X(firstSlotIndex)

#define NRD_DOUBLE_FIELDS(X)     \
  X(resampleRate)                \
  X(frequencyErrorHz)            \
  X(rmsEvmPercent)               \
  X(peakEvmPercent)              \
  X(rmsEvmDb)                    \
  X(syncCorrelation)             \
  X(normalizationFactor)         \
  X(dataPowerDb)                 \
  X(burstPowerDb)                \
  X(iqOffsetDb)                  \
  X(iqGainImbalanceDb)           \
  X(iqQuadratureErrorDeg)        \
  X(iqTimingSkewSec)             \
  X(commonPhaseErrorDeg)         \
  X(dmrsEvmPercent)              \
  X(dmrsPowerDb)                 \
  X(flatnessRippleRange1Db)      \
  X(flatnessRippleRange2Db)      \
  X(flatnessMaxRange1MinRange2Db)\
  X(flatnessMaxRange2MinRange1Db)

#define NRD_VECTOR_FIELDS(X)     \
  X(evmPerSymbol)                \
  X(evmPerSubcarrier)            \
  X(spectrumDb)                  \
  X(evmPerSlot)                  \
  X(peakEvmPerSlot)              \
  X(peakEvmPerSymbol)            \
  X(peakEvmPerSubcarrier)        \
  X(evmPerResourceBlock)         \
  X(powerPerResourceBlockDb)     \
  X(channelImpulseResponseDb)    \
  X(channelPhaseDifference)      \
  X(channelTypePerSymbol)

#define NRD_CVECTOR_FIELDS(X)    \
  X(constellation)               \
  X(reference)                   \
  X(channelEstimate)             \
  X(errorVector)                 \
  X(commonPhaseErrorPerSymbol)

#define NRD_IVECTOR_FIELDS(X)    \
  X(dmrsSymbols)

struct NrdResult {
  nrdemod::Result core;
};

namespace {

const std::vector<double>* realVector(const nrdemod::Result& r, const char* name)
{
#define NRD_PICK(f) if (std::strcmp(name, #f) == 0) return &r.f;
  NRD_VECTOR_FIELDS(NRD_PICK)
#undef NRD_PICK
  return 0;
}

const std::vector<std::complex<double> >* complexVector(const nrdemod::Result& r, const char* name)
{
#define NRD_PICK(f) if (std::strcmp(name, #f) == 0) return &r.f;
  NRD_CVECTOR_FIELDS(NRD_PICK)
#undef NRD_PICK
  return 0;
}

const std::vector<int>* intVector(const nrdemod::Result& r, const char* name)
{
#define NRD_PICK(f) if (std::strcmp(name, #f) == 0) return &r.f;
  NRD_IVECTOR_FIELDS(NRD_PICK)
#undef NRD_PICK
  return 0;
}

int toCore(const NrdConfig* in, nrdemod::Config& cfg)
{
  if (in == 0) return NRD_ERR_NULL_ARG;
  if (in->abiVersion != NRDEMOD_ABI_VERSION) return NRD_ERR_ABI;

  cfg.numerology                   = in->numerology;
  cfg.fftSize                      = in->fftSize;
  cfg.frequencyRange               = in->frequencyRange;
  cfg.channelBandwidthMHz          = in->channelBandwidthMHz;
  cfg.numResourceBlocks            = in->numResourceBlocks;
  cfg.resourceBlockOffset          = in->resourceBlockOffset;
  cfg.extendedCp                   = in->extendedCp != 0;
  cfg.modulation                   = in->modulation;
  cfg.maxSymbols                   = in->maxSymbols;
  cfg.burstSearch                  = in->burstSearch != 0;
  cfg.burstSearchThresholdDb       = in->burstSearchThresholdDb;
  cfg.timingSearch                 = in->timingSearch != 0;
  cfg.syncSearchSymbols            = in->syncSearchSymbols;
  cfg.cfoCorrection                = in->cfoCorrection != 0;
  cfg.carrierOffsetHz              = in->carrierOffsetHz;
  cfg.equalize                     = in->equalize != 0;
  cfg.equalizerIterations          = in->equalizerIterations;
  cfg.phaseTracking                = in->phaseTracking != 0;
  cfg.timingTracking               = in->timingTracking != 0;
  cfg.amplitudeTracking            = in->amplitudeTracking != 0;
  cfg.transformPrecoding           = in->transformPrecoding != 0;
  cfg.dcPunctured                  = in->dcPunctured != 0;
  cfg.removeIqOffset               = in->removeIqOffset != 0;
  cfg.symbolTimingAdjustmentPercent = in->symbolTimingAdjustmentPercent;
  cfg.resample                     = in->resample != 0;
  cfg.sampleRate                   = in->sampleRate;

  cfg.dmrs.exclude             = in->dmrsExclude != 0;
  cfg.dmrs.mappingTypeB        = in->dmrsMappingTypeB != 0;
  cfg.dmrs.typeAPosition       = in->dmrsTypeAPosition;
  cfg.dmrs.additionalPositions = in->dmrsAdditionalPositions;
  cfg.dmrs.doubleSymbol        = in->dmrsDoubleSymbol != 0;

  cfg.excludedSymbols.clear();
  if (in->excludedSymbols != 0 && in->numExcludedSymbols > 0) {
    cfg.excludedSymbols.assign(in->excludedSymbols,
                               in->excludedSymbols + in->numExcludedSymbols);
  }
  return NRD_OK;
}

NrdResult* run(const std::complex<float>* iq, size_t numSamples, const NrdConfig* in)
{
  NrdResult* out = new (std::nothrow) NrdResult();
  if (out == 0) return 0;

  nrdemod::Config cfg;
  const int rc = toCore(in, cfg);
  if (rc != NRD_OK) {
    out->core.ok = false;
    out->core.error = (rc == NRD_ERR_ABI) ? "NrdConfig ABI version mismatch"
                                          : "null configuration";
    return out;
  }

  try {
    out->core = nrdemod::demodulate(iq, numSamples, cfg);
  } catch (const std::exception& e) {
    out->core = nrdemod::Result();
    out->core.ok = false;
    out->core.error = std::string("exception: ") + e.what();
  } catch (...) {
    out->core = nrdemod::Result();
    out->core.ok = false;
    out->core.error = "unknown exception";
  }
  return out;
}

/* NUL separated, double NUL terminated name lists built once. */
const char* nameList(int kind)
{
  static std::string lists[5];
  static bool built = false;
  if (!built) {
#define NRD_APPEND(slot, f) lists[slot] += #f; lists[slot] += '\0';
#define NRD_I(f) NRD_APPEND(0, f)
#define NRD_D(f) NRD_APPEND(1, f)
#define NRD_V(f) NRD_APPEND(2, f)
#define NRD_C(f) NRD_APPEND(3, f)
#define NRD_IV(f) NRD_APPEND(4, f)
    NRD_INT_FIELDS(NRD_I)
    NRD_DOUBLE_FIELDS(NRD_D)
    NRD_VECTOR_FIELDS(NRD_V)
    NRD_CVECTOR_FIELDS(NRD_C)
    NRD_IVECTOR_FIELDS(NRD_IV)
#undef NRD_IV
#undef NRD_C
#undef NRD_V
#undef NRD_D
#undef NRD_I
#undef NRD_APPEND
    for (int i = 0; i < 5; ++i) lists[i] += '\0';
    built = true;
  }
  if (kind < 0 || kind > 4) return "\0";
  return lists[kind].c_str();
}

}

/* --- exported API ------------------------------------------------------- */

extern "C" {

const char* NRDEMOD_CALL nrd_version(void)
{
  return NRDEMOD_VERSION_STRING;
}

int NRDEMOD_CALL nrd_abi_version(void)
{
  return NRDEMOD_ABI_VERSION;
}

int NRDEMOD_CALL nrd_config_default(NrdConfig* out)
{
  if (out == 0) return NRD_ERR_NULL_ARG;

  const nrdemod::Config cfg;
  std::memset(out, 0, sizeof(*out));
  out->abiVersion                    = NRDEMOD_ABI_VERSION;
  out->numerology                    = cfg.numerology;
  out->fftSize                       = cfg.fftSize;
  out->frequencyRange                = cfg.frequencyRange;
  out->channelBandwidthMHz           = cfg.channelBandwidthMHz;
  out->numResourceBlocks             = cfg.numResourceBlocks;
  out->resourceBlockOffset           = cfg.resourceBlockOffset;
  out->extendedCp                    = cfg.extendedCp ? 1 : 0;
  out->modulation                    = cfg.modulation;
  out->maxSymbols                    = cfg.maxSymbols;
  out->burstSearch                   = cfg.burstSearch ? 1 : 0;
  out->burstSearchThresholdDb        = cfg.burstSearchThresholdDb;
  out->timingSearch                  = cfg.timingSearch ? 1 : 0;
  out->syncSearchSymbols             = cfg.syncSearchSymbols;
  out->cfoCorrection                 = cfg.cfoCorrection ? 1 : 0;
  out->carrierOffsetHz               = cfg.carrierOffsetHz;
  out->equalize                      = cfg.equalize ? 1 : 0;
  out->equalizerIterations           = cfg.equalizerIterations;
  out->phaseTracking                 = cfg.phaseTracking ? 1 : 0;
  out->timingTracking                = cfg.timingTracking ? 1 : 0;
  out->amplitudeTracking             = cfg.amplitudeTracking ? 1 : 0;
  out->transformPrecoding            = cfg.transformPrecoding ? 1 : 0;
  out->dcPunctured                   = cfg.dcPunctured ? 1 : 0;
  out->removeIqOffset                = cfg.removeIqOffset ? 1 : 0;
  out->symbolTimingAdjustmentPercent = cfg.symbolTimingAdjustmentPercent;
  out->resample                      = cfg.resample ? 1 : 0;
  out->sampleRate                    = cfg.sampleRate;
  out->dmrsExclude                   = cfg.dmrs.exclude ? 1 : 0;
  out->dmrsMappingTypeB              = cfg.dmrs.mappingTypeB ? 1 : 0;
  out->dmrsTypeAPosition             = cfg.dmrs.typeAPosition;
  out->dmrsAdditionalPositions       = cfg.dmrs.additionalPositions;
  out->dmrsDoubleSymbol              = cfg.dmrs.doubleSymbol ? 1 : 0;
  return NRD_OK;
}

NrdResult* NRDEMOD_CALL nrd_demodulate(const float* iq, size_t numSamples, const NrdConfig* cfg)
{
  if (iq == 0 && numSamples != 0) return 0;
  return run(reinterpret_cast<const std::complex<float>*>(iq), numSamples, cfg);
}

NrdResult* NRDEMOD_CALL nrd_demodulate64(const double* iq, size_t numSamples, const NrdConfig* cfg)
{
  if (iq == 0 && numSamples != 0) return 0;
  std::vector<std::complex<float> > buf(numSamples);
  for (size_t i = 0; i < numSamples; ++i) {
    buf[i] = std::complex<float>(static_cast<float>(iq[2 * i]),
                                 static_cast<float>(iq[2 * i + 1]));
  }
  return run(buf.empty() ? 0 : &buf[0], numSamples, cfg);
}

void NRDEMOD_CALL nrd_result_free(NrdResult* r)
{
  delete r;
}

int NRDEMOD_CALL nrd_result_ok(const NrdResult* r)
{
  return (r != 0 && r->core.ok) ? 1 : 0;
}

const char* NRDEMOD_CALL nrd_result_error(const NrdResult* r)
{
  return (r == 0) ? "null result" : r->core.error.c_str();
}

int NRDEMOD_CALL nrd_result_get_double(const NrdResult* r, const char* name, double* out)
{
  if (r == 0 || name == 0 || out == 0) return NRD_ERR_NULL_ARG;
#define NRD_PICK(f) if (std::strcmp(name, #f) == 0) { *out = r->core.f; return NRD_OK; }
  NRD_DOUBLE_FIELDS(NRD_PICK)
#undef NRD_PICK
  /* allow reading integer fields as double for convenience */
#define NRD_PICK(f) if (std::strcmp(name, #f) == 0) { *out = r->core.f; return NRD_OK; }
  NRD_INT_FIELDS(NRD_PICK)
#undef NRD_PICK
  return NRD_ERR_UNKNOWN_KEY;
}

int NRDEMOD_CALL nrd_result_get_int(const NrdResult* r, const char* name, int* out)
{
  if (r == 0 || name == 0 || out == 0) return NRD_ERR_NULL_ARG;
#define NRD_PICK(f) if (std::strcmp(name, #f) == 0) { *out = r->core.f; return NRD_OK; }
  NRD_INT_FIELDS(NRD_PICK)
#undef NRD_PICK
  return NRD_ERR_UNKNOWN_KEY;
}

long NRDEMOD_CALL nrd_result_vector_size(const NrdResult* r, const char* name)
{
  if (r == 0 || name == 0) return NRD_ERR_NULL_ARG;
  const std::vector<double>* v = realVector(r->core, name);
  if (v == 0) return NRD_ERR_UNKNOWN_KEY;
  return static_cast<long>(v->size());
}

int NRDEMOD_CALL nrd_result_vector_copy(const NrdResult* r, const char* name,
                                        double* dst, size_t count)
{
  if (r == 0 || name == 0 || dst == 0) return NRD_ERR_NULL_ARG;
  const std::vector<double>* v = realVector(r->core, name);
  if (v == 0) return NRD_ERR_UNKNOWN_KEY;
  if (count < v->size()) return NRD_ERR_SIZE;
  if (!v->empty()) std::memcpy(dst, &(*v)[0], v->size() * sizeof(double));
  return NRD_OK;
}

long NRDEMOD_CALL nrd_result_ivector_size(const NrdResult* r, const char* name)
{
  if (r == 0 || name == 0) return NRD_ERR_NULL_ARG;
  const std::vector<int>* v = intVector(r->core, name);
  if (v == 0) return NRD_ERR_UNKNOWN_KEY;
  return static_cast<long>(v->size());
}

int NRDEMOD_CALL nrd_result_ivector_copy(const NrdResult* r, const char* name,
                                         int* dst, size_t count)
{
  if (r == 0 || name == 0 || dst == 0) return NRD_ERR_NULL_ARG;
  const std::vector<int>* v = intVector(r->core, name);
  if (v == 0) return NRD_ERR_UNKNOWN_KEY;
  if (count < v->size()) return NRD_ERR_SIZE;
  if (!v->empty()) std::memcpy(dst, &(*v)[0], v->size() * sizeof(int));
  return NRD_OK;
}

long NRDEMOD_CALL nrd_result_cvector_size(const NrdResult* r, const char* name)
{
  if (r == 0 || name == 0) return NRD_ERR_NULL_ARG;
  const std::vector<std::complex<double> >* v = complexVector(r->core, name);
  if (v == 0) return NRD_ERR_UNKNOWN_KEY;
  return static_cast<long>(v->size());
}

int NRDEMOD_CALL nrd_result_cvector_copy(const NrdResult* r, const char* name,
                                         double* dst, size_t count)
{
  if (r == 0 || name == 0 || dst == 0) return NRD_ERR_NULL_ARG;
  const std::vector<std::complex<double> >* v = complexVector(r->core, name);
  if (v == 0) return NRD_ERR_UNKNOWN_KEY;
  if (count < v->size()) return NRD_ERR_SIZE;
  for (size_t i = 0; i < v->size(); ++i) {
    dst[2 * i]     = (*v)[i].real();
    dst[2 * i + 1] = (*v)[i].imag();
  }
  return NRD_OK;
}

const char* NRDEMOD_CALL nrd_field_names(int kind)
{
  return nameList(kind);
}

int NRDEMOD_CALL nrd_carrier_resource_blocks(int frequencyRange, int bandwidthMHz, int numerology)
{
  return nrdemod::carrierResourceBlocks(frequencyRange, bandwidthMHz, numerology);
}

float* NRDEMOD_CALL nrd_generate_test_signal(const NrdConfig* in, int numSymbols,
                                             int leadingSamples, unsigned seed,
                                             size_t* numSamples)
{
  if (numSamples == 0) return 0;
  *numSamples = 0;

  nrdemod::Config cfg;
  if (toCore(in, cfg) != NRD_OK) return 0;

  std::vector<std::complex<float> > sig;
  try {
    sig = nrdemod::generateTestSignal(cfg, numSymbols, leadingSamples, seed, 0);
  } catch (...) {
    return 0;
  }
  if (sig.empty()) return 0;

  float* raw = new (std::nothrow) float[2 * sig.size()];
  if (raw == 0) return 0;
  for (size_t i = 0; i < sig.size(); ++i) {
    raw[2 * i]     = sig[i].real();
    raw[2 * i + 1] = sig[i].imag();
  }
  *numSamples = sig.size();
  return raw;
}

void NRDEMOD_CALL nrd_free_signal(float* samples)
{
  delete[] samples;
}

}
