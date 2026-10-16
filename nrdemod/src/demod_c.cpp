/**
 ****************************************************************************
 *
 * Implementation of the flat C ABI declared in nrdemod/demod_c.h, dispatching
 * to the native WiFi/Bluetooth LE/UWB cores by DemodFamily.
 *
 * Copyright by Advantest Europe GmbH
 *
 ****************************************************************************
 */

#include "nrdemod/demod_c.h"

#include "wifi/WifiOfdmCore.hpp"
#include "wifi11n/Wifi11nOfdmCore.hpp"
#include "wifi11ac/Wifi11acOfdmCore.hpp"
#include "wifi11ax/Wifi11axOfdmCore.hpp"
#include "wifi11ad/Wifi11adOfdmCore.hpp"
#include "wifi11b/Wifi11bOfdmCore.hpp"
#include "wifi11be/Wifi11beOfdmCore.hpp"
#include "bt/BtGfskCore.hpp"
#include "uwb/UwbHrpCore.hpp"

#include <cstring>
#include <exception>
#include <new>
#include <string>
#include <vector>

#define DEMOD_VERSION_STRING "1.0.0"

struct DemodResult {
  int family;
  bool ok;
  std::string error;
  std::string dataError;

  /* Scalars, tagged by name across every family (unused ones stay at their
     default). Simpler than one struct per family since callers only read
     by name anyway. */
  struct {
    bool burstFound;
    bool syncFound;
    bool syncWordFound;
    bool isCckDataRate;
    bool dataEvmValid;
  } b;
  struct {
    int burstStart;
    int burstLength;
    int timingOffsetSamples;
    int syncPositionSamples;
    int modulation;
    int mcsIndex;
    int codingRateNumerator;
    int codingRateDenominator;
    int dataRateMbps;
    int bandwidthMHz;
    int nsts;
    int psduLengthBytes;
    int numDataSymbols;
    int peakEvmSymbolIndex;
    int peakEvmSubcarrierIndex;
    int numSyncSymbolsMeasured;
    int numDataSymbolsMeasured;
    int symbolTimingPhase;
  } i;
  struct {
    double syncRho;
    double frequencyErrorHz;
    double rmsEvmPercent;
    double peakEvmPercent;
    double dataRmsEvmPercent;
    double dataPeakEvmPercent;
    double syncWordRho;
    double deltaFAvgHz;
    double deltaFMaxHz;
    double deltaFMinHz;
    double samplesPerSymbol;
    double preambleSymbolPeriodSamples;
    double evmRmsData;
  } d;

  std::vector<double> frequencyDeviationHz;
  std::vector<int> accessAddressBits;
  bool frameAttempted = false;
  uwbdemod::FrameResult uwbFrame;
};

namespace {

template <typename CoreConfig>
void fillCommon(CoreConfig& cfg, const DemodConfig* in)
{
  cfg.sampleRate = in->sampleRate;
  cfg.burstSearch = in->burstSearch != 0;
  cfg.burstSearchThresholdDb = in->burstSearchThresholdDb;
  cfg.syncThreshold = in->syncThreshold;
}

DemodResult* fail(int family, const char* msg)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = family;
  out->ok = false;
  out->error = msg;
  return out;
}

DemodResult* fromWifiA(const wifidemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_WIFI_A;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncFound = r.syncFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.timingOffsetSamples = r.timingOffsetSamples;
  out->i.modulation = r.modulation;
  out->i.codingRateNumerator = r.codingRateNumerator;
  out->i.codingRateDenominator = r.codingRateDenominator;
  out->i.dataRateMbps = r.dataRateMbps;
  out->i.psduLengthBytes = r.psduLengthBytes;
  out->i.numDataSymbols = r.numDataSymbols;
  out->i.peakEvmSymbolIndex = r.peakEvmSymbolIndex;
  out->i.peakEvmSubcarrierIndex = r.peakEvmSubcarrierIndex;
  out->d.syncRho = r.syncRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  out->d.rmsEvmPercent = r.rmsEvmPercent;
  out->d.peakEvmPercent = r.peakEvmPercent;
  return out;
}

DemodResult* fromWifiN(const wifi11ndemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_WIFI_N;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncFound = r.syncFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.timingOffsetSamples = r.timingOffsetSamples;
  out->i.mcsIndex = r.mcsIndex;
  out->i.modulation = r.modulation;
  out->i.codingRateNumerator = r.codingRateNumerator;
  out->i.codingRateDenominator = r.codingRateDenominator;
  out->i.bandwidthMHz = r.bandwidthMHz;
  out->i.psduLengthBytes = r.psduLengthBytes;
  out->i.numDataSymbols = r.numDataSymbols;
  out->i.peakEvmSymbolIndex = r.peakEvmSymbolIndex;
  out->i.peakEvmSubcarrierIndex = r.peakEvmSubcarrierIndex;
  out->d.syncRho = r.syncRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  out->d.rmsEvmPercent = r.rmsEvmPercent;
  out->d.peakEvmPercent = r.peakEvmPercent;
  return out;
}

DemodResult* fromWifiAc(const wifi11acdemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_WIFI_AC;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncFound = r.syncFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.timingOffsetSamples = r.timingOffsetSamples;
  out->i.mcsIndex = r.mcsIndex;
  out->i.modulation = r.modulation;
  out->i.codingRateNumerator = r.codingRateNumerator;
  out->i.codingRateDenominator = r.codingRateDenominator;
  out->i.bandwidthMHz = r.bandwidthMHz;
  out->i.nsts = r.nsts;
  out->i.numDataSymbols = r.numDataSymbols;
  out->i.peakEvmSymbolIndex = r.peakEvmSymbolIndex;
  out->i.peakEvmSubcarrierIndex = r.peakEvmSubcarrierIndex;
  out->d.syncRho = r.syncRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  out->d.rmsEvmPercent = r.rmsEvmPercent;
  out->d.peakEvmPercent = r.peakEvmPercent;
  return out;
}

DemodResult* fromWifiAx(const wifi11axdemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_WIFI_AX;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncFound = r.syncFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.timingOffsetSamples = r.timingOffsetSamples;
  out->d.syncRho = r.syncRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  return out;
}

DemodResult* fromWifiAd(const wifi11addemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_WIFI_AD;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  return out;
}

DemodResult* fromWifiB(const wifi11bdemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_WIFI_B;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncFound = r.syncFound;
  out->b.isCckDataRate = r.isCckDataRate;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.timingOffsetSamples = r.timingOffsetSamples;
  out->i.numSyncSymbolsMeasured = r.numSyncSymbolsMeasured;
  out->i.numDataSymbolsMeasured = r.numDataSymbolsMeasured;
  out->d.syncRho = r.syncRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  out->d.rmsEvmPercent = r.rmsEvmPercent;
  out->d.peakEvmPercent = r.peakEvmPercent;
  out->d.dataRmsEvmPercent = r.dataRmsEvmPercent;
  out->d.dataPeakEvmPercent = r.dataPeakEvmPercent;
  return out;
}

DemodResult* fromWifiBe(const wifi11bedemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_WIFI_BE;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncFound = r.syncFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.timingOffsetSamples = r.timingOffsetSamples;
  out->d.syncRho = r.syncRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  out->b.dataEvmValid = r.dataEvmValid;
  out->dataError = r.dataError;
  out->d.evmRmsData = r.evmRmsData;
  out->i.numDataSymbols = r.numDataSymbols;
  return out;
}

DemodResult* fromBt(const btdemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_BT_LE;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncWordFound = r.syncWordFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.syncPositionSamples = r.syncPositionSamples;
  out->i.symbolTimingPhase = r.symbolTimingPhase;
  out->d.syncWordRho = r.syncWordRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  out->d.deltaFAvgHz = r.deltaFAvgHz;
  out->d.deltaFMaxHz = r.deltaFMaxHz;
  out->d.deltaFMinHz = r.deltaFMinHz;
  out->d.samplesPerSymbol = r.samplesPerSymbol;
  out->frequencyDeviationHz = r.frequencyDeviationHz;
  out->accessAddressBits = r.accessAddressBits;
  return out;
}

DemodResult* fromUwb(const uwbdemod::Result& r)
{
  DemodResult* out = new (std::nothrow) DemodResult();
  if (out == 0) return 0;
  out->family = DEMOD_UWB_HRP;
  out->ok = r.ok;
  out->error = r.error;
  out->b.burstFound = r.burstFound;
  out->b.syncFound = r.syncFound;
  out->i.burstStart = r.burstStart;
  out->i.burstLength = r.burstLength;
  out->i.syncPositionSamples = r.syncPositionSamples;
  out->d.syncRho = r.syncRho;
  out->d.frequencyErrorHz = r.frequencyErrorHz;
  out->d.preambleSymbolPeriodSamples = r.preambleSymbolPeriodSamples;
  return out;
}

DemodResult* run(int family, const std::complex<float>* iqf, size_t numSamples, const DemodConfig* in)
{
  if (in == 0) return fail(family, "null configuration");
  if (in->abiVersion != DEMOD_ABI_VERSION) return fail(family, "DemodConfig ABI version mismatch");

  std::vector<std::complex<double> > iq(numSamples);
  for (size_t i = 0; i < numSamples; ++i) iq[i] = std::complex<double>(iqf[i].real(), iqf[i].imag());

  try {
    switch (family) {
      case DEMOD_WIFI_A: {
        wifidemod::Config cfg;
        fillCommon(cfg, in);
        return fromWifiA(wifidemod::demodulate(iq, cfg));
      }
      case DEMOD_WIFI_N: {
        wifi11ndemod::Config cfg;
        fillCommon(cfg, in);
        return fromWifiN(wifi11ndemod::demodulate(iq, cfg));
      }
      case DEMOD_WIFI_AC: {
        wifi11acdemod::Config cfg;
        fillCommon(cfg, in);
        return fromWifiAc(wifi11acdemod::demodulate(iq, cfg));
      }
      case DEMOD_WIFI_AX: {
        wifi11axdemod::Config cfg;
        fillCommon(cfg, in);
        return fromWifiAx(wifi11axdemod::demodulate(iq, cfg));
      }
      case DEMOD_WIFI_AD: {
        wifi11addemod::Config cfg;
        cfg.burstSearchThresholdDb = in->burstSearchThresholdDb;
        return fromWifiAd(wifi11addemod::demodulate(iq, cfg));
      }
      case DEMOD_WIFI_B: {
        wifi11bdemod::Config cfg;
        fillCommon(cfg, in);
        return fromWifiB(wifi11bdemod::demodulate(iq, cfg));
      }
      case DEMOD_WIFI_BE: {
        wifi11bedemod::Config cfg;
        fillCommon(cfg, in);
        return fromWifiBe(wifi11bedemod::demodulate(iq, cfg));
      }
      case DEMOD_BT_LE: {
        btdemod::Config cfg;
        fillCommon(cfg, in);
        cfg.phy = in->phy;
        cfg.modulationIndex = in->modulationIndex;
        cfg.accessAddress = in->accessAddress;
        return fromBt(btdemod::demodulate(iq, cfg));
      }
      case DEMOD_UWB_HRP: {
        uwbdemod::Config cfg;
        fillCommon(cfg, in);
        cfg.phyMode = in->phyMode;
        cfg.syncLength = in->syncLength;
        cfg.chipRateHz = in->chipRateHz;
        return fromUwb(uwbdemod::demodulate(iq, cfg));
      }
      default:
        return fail(family, "unknown DemodFamily");
    }
  } catch (const std::exception& e) {
    return fail(family, (std::string("exception: ") + e.what()).c_str());
  } catch (...) {
    return fail(family, "unknown exception");
  }
}

}

extern "C" {

DemodResult* DEMOD_CALL demod_uwb_frame(const float* samples, size_t numSamples,
    const DemodConfig* input, const DemodUwbFrameConfig* frame)
{
  if (!input || !frame || (!samples && numSamples)) return fail(DEMOD_UWB_HRP, "null argument");
  if (input->abiVersion != DEMOD_ABI_VERSION ||
      (frame->structSize != sizeof(DemodUwbFrameConfig) &&
        frame->structSize != offsetof(DemodUwbFrameConfig, dynamicPhr) &&
       frame->structSize != offsetof(DemodUwbFrameConfig, constraintLength)))
    return fail(DEMOD_UWB_HRP, "UWB frame configuration ABI mismatch");
  if (numSamples > 40000000) return fail(DEMOD_UWB_HRP, "capture exceeds native frame size limit");
  try {
    uwbdemod::Config cfg;
    fillCommon(cfg, input);
    cfg.phyMode = input->phyMode;
    cfg.syncLength = input->syncLength;
    cfg.chipRateHz = input->chipRateHz;
    uwbdemod::FrameConfig settings;
    settings.codeIndex = frame->codeIndex;
    settings.sfdIndex = frame->sfdIndex;
    settings.stsPacketConfig = frame->stsPacketConfig;
    settings.samplesPerPulse = frame->samplesPerPulse;
    settings.legacy = frame->legacy != 0;
    settings.highRatePhr = frame->highRatePhr != 0;
    if (frame->stsKeyHex) settings.stsKeyHex = frame->stsKeyHex;
    if (frame->stsIvHex) settings.stsIvHex = frame->stsIvHex;
    if (frame->structSize >= static_cast<int>(offsetof(DemodUwbFrameConfig, dynamicPhr))) {
      settings.constraintLength = frame->constraintLength;
      settings.numStsSegments = frame->numStsSegments;
      settings.meanPrfMHz = frame->meanPrfMHz;
    }
    if (frame->structSize == sizeof(DemodUwbFrameConfig)) {
      settings.dynamicPhr = frame->dynamicPhr != 0;
      settings.bitRateMbps = frame->bitRateMbps;
      settings.stsSegmentLength = frame->stsSegmentLength;
      settings.extraStsGapLength = frame->extraStsGapLength;
      settings.macFcsLength = frame->macFcsLength;
      settings.extraStsGapIndex = frame->extraStsGapIndex;
      settings.legacyRateMbps = frame->legacyRateMbps;
      settings.legacyMeanPrfMHz = frame->legacyMeanPrfMHz;
      settings.preambleSpread = frame->preambleSpread;
      settings.legacyVsaHopping = frame->legacyVsaHopping != 0;
    }
    std::vector<std::complex<double>> iq(numSamples);
    for (size_t index = 0; index < numSamples; ++index)
      iq[index] = {samples[2 * index], samples[2 * index + 1]};
    const auto decoded = uwbdemod::demodulateFrame(iq, cfg, settings);
    DemodResult* out = fromUwb(decoded.acquisition);
    if (!out) return nullptr;
    out->frameAttempted = true;
    out->uwbFrame = decoded;
    out->ok = decoded.ok;
    out->error = decoded.error;
    out->i.psduLengthBytes = decoded.psduLengthBytes;
    return out;
  } catch (const std::exception& error) {
    return fail(DEMOD_UWB_HRP, error.what());
  } catch (...) {
    return fail(DEMOD_UWB_HRP, "unknown UWB frame exception");
  }
}

DemodResult* DEMOD_CALL demod_wifi_be_data(const float* samples, size_t numSamples,
    const DemodConfig* input, const DemodWifiBeDataConfig* data)
{
  if (!input || !data || (!samples && numSamples)) return fail(DEMOD_WIFI_BE, "null argument");
  if (input->abiVersion != DEMOD_ABI_VERSION) return fail(DEMOD_WIFI_BE, "DemodConfig ABI version mismatch");
  try {
    wifi11bedemod::Config cfg;
    fillCommon(cfg, input);
    cfg.measureDataEvm = true;
    cfg.dataBandwidthMHz = data->dataBandwidthMHz;
    cfg.numSpatialStreams = data->numSpatialStreams;
    cfg.ehtLtfType = data->ehtLtfType;
    cfg.dataMcs = data->dataMcs;
    cfg.ehtLtfFftOffsetSamples = data->ehtLtfFftOffsetSamples;
    cfg.dataFftOffsetSamples = data->dataFftOffsetSamples;
    cfg.dataGuardIntervalSamples = data->dataGuardIntervalSamples;
    cfg.numDataSymbols = data->numDataSymbols;
    std::vector<std::complex<double>> iq(numSamples);
    for (size_t index = 0; index < numSamples; ++index)
      iq[index] = std::complex<double>(samples[2 * index], samples[2 * index + 1]);
    return fromWifiBe(wifi11bedemod::demodulate(iq, cfg));
  } catch (const std::exception& error) {
    return fail(DEMOD_WIFI_BE, error.what());
  } catch (...) {
    return fail(DEMOD_WIFI_BE, "unknown exception");
  }
}

const char* DEMOD_CALL demod_result_data_error(const DemodResult* result)
{
  return result ? result->dataError.c_str() : "null result";
}

const char* DEMOD_CALL demod_version(void) { return DEMOD_VERSION_STRING; }
int DEMOD_CALL demod_abi_version(void) { return DEMOD_ABI_VERSION; }

int DEMOD_CALL demod_config_default(int family, DemodConfig* out)
{
  if (out == 0) return DEMOD_ERR_NULL_ARG;
  std::memset(out, 0, sizeof(*out));
  out->abiVersion = DEMOD_ABI_VERSION;

  switch (family) {
    case DEMOD_WIFI_A:
    case DEMOD_WIFI_N:
    case DEMOD_WIFI_AC:
    case DEMOD_WIFI_AX:
    case DEMOD_WIFI_BE:
      out->sampleRate = 0.0;
      out->burstSearch = 1;
      out->burstSearchThresholdDb = 15.0;
      out->syncThreshold = 0.5;
      return DEMOD_OK;
    case DEMOD_WIFI_AD:
      out->burstSearchThresholdDb = 15.0;
      return DEMOD_OK;
    case DEMOD_WIFI_B:
      out->sampleRate = 0.0;
      out->burstSearch = 1;
      out->burstSearchThresholdDb = 15.0;
      out->syncThreshold = 0.5;
      return DEMOD_OK;
    case DEMOD_BT_LE:
      out->sampleRate = 0.0;
      out->phy = 1; /* LE_1M */
      out->modulationIndex = 0.5;
      out->burstSearch = 1;
      out->burstSearchThresholdDb = 15.0;
      out->accessAddress = 0x71764129u;
      out->syncThreshold = 0.85;
      return DEMOD_OK;
    case DEMOD_UWB_HRP:
      out->sampleRate = 0.0;
      out->phyMode = 0; /* BPRF */
      out->syncLength = 64;
      out->chipRateHz = 499.2e6;
      out->burstSearch = 1;
      out->burstSearchThresholdDb = 15.0;
      out->syncThreshold = 0.5;
      return DEMOD_OK;
    default:
      return DEMOD_ERR_FAMILY;
  }
}

DemodResult* DEMOD_CALL demod_demodulate(int family, const float* iq, size_t numSamples, const DemodConfig* cfg)
{
  if (iq == 0 && numSamples != 0) return 0;
  return run(family, reinterpret_cast<const std::complex<float>*>(iq), numSamples, cfg);
}

DemodResult* DEMOD_CALL demod_demodulate64(int family, const double* iq, size_t numSamples, const DemodConfig* cfg)
{
  if (iq == 0 && numSamples != 0) return 0;
  std::vector<std::complex<float> > buf(numSamples);
  for (size_t i = 0; i < numSamples; ++i) {
    buf[i] = std::complex<float>(static_cast<float>(iq[2 * i]), static_cast<float>(iq[2 * i + 1]));
  }
  return run(family, buf.empty() ? 0 : &buf[0], numSamples, cfg);
}

void DEMOD_CALL demod_result_free(DemodResult* r) { delete r; }

int DEMOD_CALL demod_result_ok(const DemodResult* r) { return (r != 0 && r->ok) ? 1 : 0; }

const char* DEMOD_CALL demod_result_error(const DemodResult* r)
{
  return (r == 0) ? "null result" : r->error.c_str();
}

int DEMOD_CALL demod_result_get_bool(const DemodResult* r, const char* name, int* out)
{
  if (r == 0 || name == 0 || out == 0) return DEMOD_ERR_NULL_ARG;
  if (r->family == DEMOD_UWB_HRP && std::strcmp(name, "frameAttempted") == 0) {
    *out = r->frameAttempted;
    return DEMOD_OK;
  }
  if (r->frameAttempted) {
#define FRAME_BOOL(field) if (std::strcmp(name, #field) == 0) { *out = r->uwbFrame.field; return DEMOD_OK; }
    FRAME_BOOL(sfdFound) FRAME_BOOL(stsPresent) FRAME_BOOL(stsValid) FRAME_BOOL(phrValid) FRAME_BOOL(psduValid)
    FRAME_BOOL(phr1Valid) FRAME_BOOL(ldpcCoding) FRAME_BOOL(macFcsChecked) FRAME_BOOL(macFcsValid) FRAME_BOOL(evmValid)
#undef FRAME_BOOL
  }
#define PICK(f) if (std::strcmp(name, #f) == 0) { *out = r->b.f ? 1 : 0; return DEMOD_OK; }
  PICK(burstFound) PICK(syncFound) PICK(syncWordFound) PICK(isCckDataRate)
  PICK(dataEvmValid)
#undef PICK
  return DEMOD_ERR_UNKNOWN_KEY;
}

int DEMOD_CALL demod_result_get_int(const DemodResult* r, const char* name, int* out)
{
  if (r == 0 || name == 0 || out == 0) return DEMOD_ERR_NULL_ARG;
  if (r->frameAttempted) {
#define FRAME_INT(field) if (std::strcmp(name, #field) == 0) { *out = r->uwbFrame.field; return DEMOD_OK; }
    FRAME_INT(syncStart) FRAME_INT(sfdStart) FRAME_INT(sfdLength) FRAME_INT(stsStart) FRAME_INT(stsLength)
    FRAME_INT(phrStart) FRAME_INT(phrLength) FRAME_INT(psduStart) FRAME_INT(psduLength) FRAME_INT(rsCorrectedSymbols)
    FRAME_INT(phr2Start) FRAME_INT(phr2Length) FRAME_INT(ldpcCorrectedBits)
#undef FRAME_INT
  }
#define PICK(f) if (std::strcmp(name, #f) == 0) { *out = r->i.f; return DEMOD_OK; }
  PICK(burstStart) PICK(burstLength) PICK(timingOffsetSamples) PICK(syncPositionSamples)
  PICK(modulation) PICK(mcsIndex) PICK(codingRateNumerator) PICK(codingRateDenominator)
  PICK(dataRateMbps) PICK(bandwidthMHz) PICK(nsts) PICK(psduLengthBytes) PICK(numDataSymbols)
  PICK(peakEvmSymbolIndex) PICK(peakEvmSubcarrierIndex) PICK(numSyncSymbolsMeasured)
  PICK(numDataSymbolsMeasured) PICK(symbolTimingPhase)
#undef PICK
  return DEMOD_ERR_UNKNOWN_KEY;
}

int DEMOD_CALL demod_result_get_double(const DemodResult* r, const char* name, double* out)
{
  if (r == 0 || name == 0 || out == 0) return DEMOD_ERR_NULL_ARG;
  if (r->frameAttempted) {
#define FRAME_DOUBLE(field) if (std::strcmp(name, #field) == 0) { *out = r->uwbFrame.field; return DEMOD_OK; }
    FRAME_DOUBLE(sfdRho) FRAME_DOUBLE(stsRho) FRAME_DOUBLE(bitRateMbps)
    FRAME_DOUBLE(shrEvmRmsPercent) FRAME_DOUBLE(stsEvmRmsPercent)
    FRAME_DOUBLE(phrEvmRmsPercent) FRAME_DOUBLE(psduEvmRmsPercent)
#undef FRAME_DOUBLE
  }
  if (std::strcmp(name, "evmRmsData") == 0) {
    if (!r->b.dataEvmValid) return DEMOD_ERR_FAILED;
    *out = r->d.evmRmsData;
    return DEMOD_OK;
  }
#define PICK(f) if (std::strcmp(name, #f) == 0) { *out = r->d.f; return DEMOD_OK; }
  PICK(syncRho) PICK(frequencyErrorHz) PICK(rmsEvmPercent) PICK(peakEvmPercent)
  PICK(dataRmsEvmPercent) PICK(dataPeakEvmPercent) PICK(syncWordRho) PICK(deltaFAvgHz)
  PICK(deltaFMaxHz) PICK(deltaFMinHz) PICK(samplesPerSymbol) PICK(preambleSymbolPeriodSamples)
#undef PICK
  /* allow reading int fields as double for convenience */
  int iv = 0;
  if (demod_result_get_int(r, name, &iv) == DEMOD_OK) { *out = iv; return DEMOD_OK; }
  return DEMOD_ERR_UNKNOWN_KEY;
}

long DEMOD_CALL demod_result_vector_size(const DemodResult* r, const char* name)
{
  if (r == 0 || name == 0) return DEMOD_ERR_NULL_ARG;
  if (std::strcmp(name, "frequencyDeviationHz") == 0) return static_cast<long>(r->frequencyDeviationHz.size());
  return DEMOD_ERR_UNKNOWN_KEY;
}

int DEMOD_CALL demod_result_vector_copy(const DemodResult* r, const char* name, double* dst, size_t count)
{
  if (r == 0 || name == 0 || dst == 0) return DEMOD_ERR_NULL_ARG;
  if (std::strcmp(name, "frequencyDeviationHz") != 0) return DEMOD_ERR_UNKNOWN_KEY;
  if (count < r->frequencyDeviationHz.size()) return DEMOD_ERR_SIZE;
  if (!r->frequencyDeviationHz.empty())
    std::memcpy(dst, &r->frequencyDeviationHz[0], r->frequencyDeviationHz.size() * sizeof(double));
  return DEMOD_OK;
}

long DEMOD_CALL demod_result_ivector_size(const DemodResult* r, const char* name)
{
  if (r == 0 || name == 0) return DEMOD_ERR_NULL_ARG;
  if (r->frameAttempted && std::strcmp(name, "psduBytes") == 0) return static_cast<long>(r->uwbFrame.psduBytes.size());
  if (r->frameAttempted && std::strcmp(name, "phrBits") == 0) return static_cast<long>(r->uwbFrame.phrBits.size());
  if (std::strcmp(name, "accessAddressBits") == 0) return static_cast<long>(r->accessAddressBits.size());
  return DEMOD_ERR_UNKNOWN_KEY;
}

int DEMOD_CALL demod_result_ivector_copy(const DemodResult* r, const char* name, int* dst, size_t count)
{
  if (r == 0 || name == 0 || dst == 0) return DEMOD_ERR_NULL_ARG;
  if (r->frameAttempted && (std::strcmp(name, "psduBytes") == 0 || std::strcmp(name, "phrBits") == 0)) {
    const auto& values = std::strcmp(name, "psduBytes") == 0 ? r->uwbFrame.psduBytes : r->uwbFrame.phrBits;
    if (count < values.size()) return DEMOD_ERR_SIZE;
    if (!values.empty()) std::memcpy(dst, values.data(), values.size() * sizeof(int));
    return DEMOD_OK;
  }
  if (std::strcmp(name, "accessAddressBits") != 0) return DEMOD_ERR_UNKNOWN_KEY;
  if (count < r->accessAddressBits.size()) return DEMOD_ERR_SIZE;
  if (!r->accessAddressBits.empty())
    std::memcpy(dst, &r->accessAddressBits[0], r->accessAddressBits.size() * sizeof(int));
  return DEMOD_OK;
}

const char* DEMOD_CALL demod_field_names(int family, int kind)
{
  static const char* boolFieldsCommon = "burstFound\0syncFound\0";
  static const char* boolFieldsB = "burstFound\0syncFound\0isCckDataRate\0";
  static const char* boolFieldsAd = "burstFound\0";
  static const char* boolFieldsBt = "burstFound\0syncWordFound\0";

  switch (kind) {
    case 0: /* bool */
      switch (family) {
        case DEMOD_WIFI_AD: return boolFieldsAd;
        case DEMOD_WIFI_B: return boolFieldsB;
        case DEMOD_BT_LE: return boolFieldsBt;
        case DEMOD_WIFI_BE: return "burstFound\0syncFound\0dataEvmValid\0";
        case DEMOD_UWB_HRP: return "burstFound\0syncFound\0frameAttempted\0sfdFound\0stsPresent\0stsValid\0phrValid\0psduValid\0"
          "phr1Valid\0ldpcCoding\0macFcsChecked\0macFcsValid\0evmValid\0";
        default: return boolFieldsCommon;
      }
    case 1: /* int */
      switch (family) {
        case DEMOD_WIFI_A: return "burstStart\0burstLength\0timingOffsetSamples\0modulation\0"
                                   "codingRateNumerator\0codingRateDenominator\0dataRateMbps\0"
                                   "psduLengthBytes\0numDataSymbols\0peakEvmSymbolIndex\0peakEvmSubcarrierIndex\0";
        case DEMOD_WIFI_N: return "burstStart\0burstLength\0timingOffsetSamples\0mcsIndex\0modulation\0"
                                   "codingRateNumerator\0codingRateDenominator\0bandwidthMHz\0"
                                   "psduLengthBytes\0numDataSymbols\0peakEvmSymbolIndex\0peakEvmSubcarrierIndex\0";
        case DEMOD_WIFI_AC: return "burstStart\0burstLength\0timingOffsetSamples\0mcsIndex\0modulation\0"
                                    "codingRateNumerator\0codingRateDenominator\0bandwidthMHz\0nsts\0"
                                    "numDataSymbols\0peakEvmSymbolIndex\0peakEvmSubcarrierIndex\0";
        case DEMOD_WIFI_AX: return "burstStart\0burstLength\0timingOffsetSamples\0";
        case DEMOD_WIFI_BE: return "burstStart\0burstLength\0timingOffsetSamples\0numDataSymbols\0";
        case DEMOD_WIFI_AD: return "burstStart\0burstLength\0";
        case DEMOD_WIFI_B: return "burstStart\0burstLength\0timingOffsetSamples\0"
                                   "numSyncSymbolsMeasured\0numDataSymbolsMeasured\0";
        case DEMOD_BT_LE: return "burstStart\0burstLength\0syncPositionSamples\0symbolTimingPhase\0";
        case DEMOD_UWB_HRP: return "burstStart\0burstLength\0syncPositionSamples\0syncStart\0sfdStart\0sfdLength\0"
          "stsStart\0stsLength\0phrStart\0phrLength\0psduStart\0psduLength\0psduLengthBytes\0rsCorrectedSymbols\0"
          "phr2Start\0phr2Length\0ldpcCorrectedBits\0";
        default: return "\0";
      }
    case 2: /* double */
      switch (family) {
        case DEMOD_WIFI_A:
        case DEMOD_WIFI_N:
        case DEMOD_WIFI_AC: return "syncRho\0frequencyErrorHz\0rmsEvmPercent\0peakEvmPercent\0";
        case DEMOD_WIFI_AX: return "syncRho\0frequencyErrorHz\0";
        case DEMOD_WIFI_BE: return "syncRho\0frequencyErrorHz\0evmRmsData\0";
        case DEMOD_WIFI_AD: return "\0";
        case DEMOD_WIFI_B: return "syncRho\0frequencyErrorHz\0rmsEvmPercent\0peakEvmPercent\0"
                                   "dataRmsEvmPercent\0dataPeakEvmPercent\0";
        case DEMOD_BT_LE: return "syncWordRho\0frequencyErrorHz\0deltaFAvgHz\0deltaFMaxHz\0"
                                  "deltaFMinHz\0samplesPerSymbol\0";
        case DEMOD_UWB_HRP: return "syncRho\0frequencyErrorHz\0preambleSymbolPeriodSamples\0sfdRho\0stsRho\0bitRateMbps\0"
          "shrEvmRmsPercent\0stsEvmRmsPercent\0phrEvmRmsPercent\0psduEvmRmsPercent\0";
        default: return "\0";
      }
    case 3: /* vector<double> */
      return (family == DEMOD_BT_LE) ? "frequencyDeviationHz\0" : "\0";
    case 4: /* vector<int> */
      if (family == DEMOD_UWB_HRP) return "phrBits\0psduBytes\0";
      return (family == DEMOD_BT_LE) ? "accessAddressBits\0" : "\0";
    default:
      return "\0";
  }
}

}
