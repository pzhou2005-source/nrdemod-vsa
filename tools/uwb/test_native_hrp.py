"""Native C++ payload verification against independent MATLAB HRP stage vectors.

Run with: python -m unittest discover -s tools\\uwb -p test_native_hrp.py -v
Generate optional MATLAB fixtures with tools\\uwb\\gen_hrp_reference.m.
"""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "nrdemod" / "python"), str(ROOT / "nrdemod" / "python" / "examples")]

from nrdemod import demod
import uwb_hrp as hrp
from verify_hrp_demod import cfg_from_ref, impair


def native(iq, cfg):
    return demod.demodulate(
        demod.UWB_HRP, iq,
        demod.Config(sampleRate=cfg.sample_rate, phyMode=2 if cfg.mode == "802.15.4a" else 0,
                     syncLength=cfg.preamble_duration),
        data_config=demod.UwbHrpDataConfig(
            codeIndex=cfg.code_index, sfdNumber=cfg.sfd_number,
            sfdLength=64 if cfg.mode == "802.15.4a" and cfg.data_rate_kbps == 110 else 8,
            preambleSpreadingFactor=cfg.L, meanPrfMHz=cfg.mean_prf,
            phrDataRateKbps=0, stsPacketConfiguration=cfg.sts_packet_configuration))


def hprf_packet(bits, data, sync=32):
    """Test transmitter using standard code 25 and an independent bit-domain encoder."""
    code = np.array([1 if x == "+" else -1 if x == "-" else 0 for x in
                     "-0++++--+--+-++++-+---++--++++++-++-+00+--+0--+0+++++--+++--0--0+-+----0-+-+-+0+--++-+-+++0"])
    phr = np.zeros(13, np.uint8)
    length = len(bits) // 8
    phr[:12] = [(length >> i) & 1 for i in range(11, -1, -1)]
    if data.stsPacketConfiguration == 2 and (data.extraStsGapIndex or data.extraStsGapLength):
        phr[:2] = [data.extraStsGapIndex >> 1, data.extraStsGapIndex & 1]
    header = hrp.secded_encode(phr)
    if data.constraintLength == 3:
        encoded = hrp.conv_encode(np.r_[header, hrp.rs_encode(bits), 0, 0]).reshape(-1, 2)
        n_header = 21
    else:
        encoded = []
        state = 0
        for bit in np.r_[header, np.zeros(6, np.uint8), bits, np.zeros(6, np.uint8)]:
            register = (int(bit) << 6) | state
            encoded.append([(register & 0o133).bit_count() & 1, (register & 0o171).bit_count() & 1])
            state = (int(bit) << 5) | (state >> 1)
        encoded = np.array(encoded)
        n_header = 25
    group_length = 8 if data.meanPrfMHz == 124.8 else 4
    spacing = 2 if data.meanPrfMHz == 124.8 else 1
    n_groups = np.where(np.arange(len(encoded)) < n_header,
                        4 if data.constraintLength == 3 else 2, 2)
    pn = np.zeros(int(n_groups.sum()) * group_length + 15, int)
    pn[:15] = (code[code != 0][:15] > 0).astype(int)
    for i in range(15, len(pn)):
        pn[i] = pn[i-14] ^ pn[i-15]
    symbols = []
    offset = 15
    for (g0, g1), groups in zip(encoded, n_groups):
        first = int(g0) ^ int(g1) if data.constraintLength == 3 else int(g0)
        mapped = np.repeat([first, int(g1)] * (int(groups) // 2), group_length)
        active = (1 - 2 * mapped) * (1 - 2 * pn[offset:offset + len(mapped)])
        guarded = np.zeros((int(groups), 2 * group_length))
        guarded[:, :group_length] = active.reshape(-1, group_length)
        symbol = np.zeros(guarded.size * spacing)
        symbol[::spacing] = guarded.ravel()
        symbols.append(symbol)
        offset += len(mapped)
    spread = np.zeros(91 * 4)
    spread[::4] = code
    shr = np.r_[np.tile(spread, sync), np.concatenate([s * spread for s in hrp._SFD_4Z[data.sfdNumber]])]
    sts = np.zeros(512 + data.numStsSegments * (data.stsSegmentLength * 512 + 512))
    body = np.concatenate(symbols)
    if data.stsPacketConfiguration == 1:
        chips = np.r_[shr, sts, body]
    elif data.stsPacketConfiguration == 2:
        chips = np.r_[shr, body, np.zeros(4 * data.extraStsGapLength), sts]
    elif data.stsPacketConfiguration == 3:
        chips = np.r_[shr, sts]
    else:
        chips = np.r_[shr, body]
    return hrp.pulse_shape(chips, 4)


class NativeHrpTests(unittest.TestCase):
    def check_packet(self, result, bits, cl7=False):
        self.assertTrue(result.ok, result.error)
        self.assertTrue(result.sfdFound, result.dataError)
        self.assertTrue(result.phrDecoded, result.dataError)
        self.assertTrue(result.psduDecoded, result.dataError)
        self.assertEqual(result.rsOk, not cl7)
        self.assertEqual(result.dataError, "")
        np.testing.assert_array_equal(result.psduBits, bits)
        self.assertEqual(result.psduBytes.astype(np.uint8).tobytes(),
                         np.packbits(bits, bitorder="little").tobytes())
        self.assertEqual(result.softBits.size % 2, 0)
        if result.burstPositions.size:
            self.assertEqual(result.burstPositions.size, result.softBits.size // 2)

    def test_independent_matlab_vectors(self):
        paths = sorted((ROOT / "captures" / "generated" / "uwb_hrp").glob("*.mat"))
        if not paths:
            self.skipTest("generate MATLAB reference fixtures with gen_hrp_reference.m")
        rng = np.random.default_rng(123)
        for path in paths:
            m = loadmat(path, squeeze_me=True)
            cfg = cfg_from_ref(m)
            bits = np.asarray(m["psdu_bits"], np.uint8).ravel()
            wave = np.asarray(m["waveform"], complex).ravel()
            for snr, cfo in ((None, 0.0), (18.0, 90e3 if cfg.mean_prf == 3.9 else 200e3)):
                with self.subTest(vector=path.stem, snr=snr, cfo=cfo):
                    x = impair(wave, cfg.sample_rate, rng, snr_db=snr, cfo_hz=cfo, lead=911, tail=1024)
                    r = native(x, cfg)
                    self.check_packet(r, bits)
                    self.assertAlmostEqual(r.frequencyErrorHz, cfo, delta=200)
                    self.assertEqual(r.phrStartSamples, 911 + int(m["idx_PHR"][0]) - 1)
                    np.testing.assert_array_equal(r.phrBits, np.asarray(m["phr_bits"], np.uint8)[:13])

    def test_sample_rates_codes_and_phr_rates(self):
        bits = np.unpackbits(np.arange(127, dtype=np.uint8), bitorder="little")
        for spc in (2, 4, 8):
            for rate in (850, 6810):
                for code in (9, 19, 24):
                    with self.subTest(spc=spc, phr_rate=rate, code=code):
                        cfg = hrp.HrpConfig(samples_per_pulse=spc, phr_data_rate_kbps=rate, code_index=code)
                        r = native(hrp.generate(bits, cfg)["waveform"], cfg)
                        self.check_packet(r, bits)

    def test_independent_hprf_matlab_vectors(self):
        paths = sorted((ROOT / "captures" / "generated" / "uwb_hprf").glob("*.mat"))
        if not paths:
            self.skipTest("generate HPRF fixtures with gen_hprf_reference.m")
        rng = np.random.default_rng(456)
        for path in paths:
            m = loadmat(path, squeeze_me=True)
            fs = float(m["sample_rate"])
            config = demod.Config(sampleRate=fs, phyMode=1, syncLength=int(m["preamble_duration"]))
            data = demod.UwbHrpDataConfig(
                codeIndex=int(m["code_index"]), sfdNumber=int(m["sfd_number"]),
                preambleSpreadingFactor=int(m["preamble_spreading_factor"]), meanPrfMHz=float(m["mean_prf"]),
                constraintLength=int(m["constraint_length"]),
                stsPacketConfiguration=int(m["sts_packet_configuration"]),
                numStsSegments=int(m["num_sts_segments"]), stsSegmentLength=int(m["sts_segment_length"]),
                extraStsGapIndex=int(m["extra_sts_gap_index"]), extraStsGapLength=int(m["extra_sts_gap_length"]))
            bits = np.asarray(m["psdu_bits"], np.uint8).ravel()
            for snr, cfo in ((None, 0.0), (18.0, -200e3)):
                with self.subTest(vector=path.stem, snr=snr, cfo=cfo):
                    x = impair(np.asarray(m["waveform"], complex).ravel(), fs, rng,
                               snr_db=snr, cfo_hz=cfo, lead=733, tail=1024)
                    r = demod.demodulate(demod.UWB_HRP, x, config, data_config=data)
                    self.check_packet(r, bits, cl7=data.constraintLength == 7)
                    self.assertAlmostEqual(r.frequencyErrorHz, cfo, delta=200)
                    self.assertEqual(r.phrStartSamples, 733 + int(m["idx_PHR"][0]) - 1)

    def test_reed_solomon_and_secded_correction(self):
        bits = np.unpackbits(np.array([1, 35, 69, 103], np.uint8), bitorder="little")
        cfg = hrp.HrpConfig()
        original = hrp.generate(bits, cfg)
        for count in range(6):
            for phr_errors in (0, 1, 2):
                with self.subTest(rs_errors=count, phr_errors=phr_errors):
                    rs = original["rs_psdu_bits"].copy()
                    rs[2:2 + 6 * count:6] ^= 1
                    phr = original["phr_bits"].copy()
                    phr[[5, 11][:phr_errors]] ^= 1
                    encoded = hrp.conv_encode(np.r_[phr, rs, 0, 0])
                    chips = np.r_[original["shr_chips"], hrp.bpm_bpsk_modulate(
                        encoded, cfg, cfg.chips_per_burst(850), cfg.chips_per_burst(6810))]
                    r = native(hrp.pulse_shape(chips, 4), cfg)
                    if count <= 4 and phr_errors <= 1:
                        self.check_packet(r, bits)
                        self.assertEqual(r.rsCorrectedSymbols, count)
                    else:
                        self.assertFalse(r.psduDecoded)
                        self.assertNotEqual(r.dataError, "")

    def test_hprf_mapping_matrix(self):
        rng = np.random.default_rng(789)
        for prf in (124.8, 249.6):
            for cl in (3, 7):
                for sp in (0, 1, 2, 3):
                    with self.subTest(prf=prf, cl=cl, sp=sp):
                        data = demod.UwbHrpDataConfig(
                            codeIndex=25, sfdNumber=sp + 1, meanPrfMHz=prf, constraintLength=cl,
                            stsPacketConfiguration=sp, numStsSegments=2, stsSegmentLength=16,
                            extraStsGapIndex=1 if sp == 2 else 0, extraStsGapLength=127 if sp == 2 else 0)
                        bits = rng.integers(0, 2, 80, dtype=np.uint8)
                        wave = hprf_packet(bits, data)
                        x = impair(wave, 1996.8e6, rng, snr_db=18, cfo_hz=-200e3, lead=713, tail=512)
                        r = demod.demodulate(demod.UWB_HRP, x, sampleRate=1996.8e6, phyMode=1,
                                             syncLength=32, data_config=data)
                        if sp == 3:
                            self.assertTrue(r.sfdFound, r.dataError)
                            self.assertFalse(r.phrDecoded or r.psduDecoded)
                            self.assertEqual(r.psduBytes.size, 0)
                        else:
                            self.check_packet(r, bits, cl7=cl == 7)
                        self.assertEqual(r.dataError, "")
                        self.assertAlmostEqual(r.frequencyErrorHz, -200e3, delta=200)

    def test_hprf_length_and_truncation(self):
        for length in (0, 1025, 4095):
            data = demod.UwbHrpDataConfig(codeIndex=25, sfdNumber=2, meanPrfMHz=249.6, constraintLength=7)
            bits = np.unpackbits(np.arange(length, dtype=np.uint16).astype(np.uint8), bitorder="little")
            wave = hprf_packet(bits, data)
            r = demod.demodulate(demod.UWB_HRP, wave, sampleRate=1996.8e6, phyMode=1,
                                 syncLength=32, data_config=data)
            self.check_packet(r, bits, cl7=True)
            self.assertEqual(r.psduLengthBytes, length)
            if length:
                r = demod.demodulate(demod.UWB_HRP, wave[:-4096], sampleRate=1996.8e6,
                                     phyMode=1, syncLength=32, data_config=data)
                self.assertFalse(r.psduDecoded)
                self.assertNotEqual(r.dataError, "")

    def test_uncoded_legacy_payloads(self):
        bits = np.unpackbits(np.arange(40, dtype=np.uint8), bitorder="little")
        for prf, rate in ((3.9, 6810), (15.6, 27240)):
            with self.subTest(prf=prf, rate=rate):
                cfg = hrp.HrpConfig(mode="802.15.4a", code_index=3, mean_prf=prf, data_rate_kbps=rate)
                phr = hrp.secded_encode(hrp.build_phr(cfg, len(bits) // 8))
                encoded = np.r_[hrp.conv_encode(np.r_[phr, 0, 0]), hrp.rs_encode(bits)]
                chips = np.r_[hrp.shr_chips(cfg), hrp.bpm_bpsk_modulate(
                    encoded, cfg, cfg.chips_per_burst(850), cfg.chips_per_burst(rate))]
                self.check_packet(native(hrp.pulse_shape(chips, 4), cfg), bits)

    def test_invalid_configuration_and_inputs(self):
        cfg = hrp.HrpConfig()
        bits = np.zeros(32, np.uint8)
        wave = hrp.generate(bits, cfg)["waveform"]
        for data in (demod.UwbHrpDataConfig(codeIndex=0), demod.UwbHrpDataConfig(sfdNumber=4),
                     demod.UwbHrpDataConfig(meanPrfMHz=float("nan"))):
            with self.subTest(data=data):
                r = demod.demodulate(demod.UWB_HRP, wave, sampleRate=cfg.sample_rate, data_config=data)
                self.assertFalse(r.psduDecoded)
                self.assertNotEqual(r.dataError, "")
        for wave in (np.zeros(20000, complex), np.full(20000, complex(float("nan"), 0))):
            r = native(wave, cfg)
            self.assertFalse(r.ok)
            self.assertNotEqual(r.error, "")
        with self.assertRaises(ValueError):
            demod.demodulate(demod.WIFI_A, [], data_config=demod.UwbHrpDataConfig())
        data = demod.UwbHrpDataConfig(codeIndex=25, sfdNumber=2, meanPrfMHz=249.6,
                                      extraStsGapLength=128)
        wave = hprf_packet(bits, data)
        r = demod.demodulate(demod.UWB_HRP, wave, sampleRate=cfg.sample_rate, phyMode=1, syncLength=32,
                             data_config=data)
        self.assertTrue(r.ok, r.error)
        self.assertFalse(r.psduDecoded)
        self.assertEqual(r.dataError, "invalid or unsupported HRP data configuration")


if __name__ == "__main__":
    unittest.main()
