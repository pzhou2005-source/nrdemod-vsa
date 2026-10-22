import ctypes
import os
from pathlib import Path
import unittest

import numpy as np

import nrdemod
from nrdemod import _ffi, _demod_ffi, demod


class NrApiTest(unittest.TestCase):
    def test_abi_and_defaults(self):
        self.assertEqual(_ffi.lib.nrd_abi_version(), 2)
        self.assertEqual(_demod_ffi.lib.demod_abi_version(), 2)
        config = nrdemod.Config()
        self.assertEqual(config.cpCfoGuardFraction, 0.0)
        self.assertFalse(config.removeIqOffsetAfterCfo)
        self.assertTrue(config.dmrsEnabled)
        self.assertEqual(demod.Config(steadyFrequencyDeviation=True)._to_c(demod.BT_LE).
                         steadyFrequencyDeviation, 1)

    def test_resource_arrays_and_guard(self):
        config = nrdemod.Config(sampleRate=1.92e6, fftSize=128, numResourceBlocks=1,
                               modulation=nrdemod.QPSK, ssbSearch=False, dmrsEnabled=False,
                               burstSearch=False, timingSearch=False, removeIqOffset=False,
                               cpCfoGuardFraction=1/3, removeIqOffsetAfterCfo=True)
        iq = nrdemod.generate_test_signal(config, num_symbols=28)
        baseline = nrdemod.demodulate(iq, config)
        self.assertTrue(baseline.ok, baseline.error)
        count = baseline.numSymbols*baseline.numResourceBlocks*12
        config.decisionRotations = [1+0j]*count
        config.excludedResources = [False]*count
        raw, keep = config._to_c()
        self.assertEqual(raw.numDecisionRotations, count)
        self.assertEqual(raw.numExcludedResources, count)
        self.assertEqual(len(keep), 2)
        result = nrdemod.demodulate(iq, config)
        self.assertTrue(result.ok, result.error)
        self.assertEqual(result.rmsEvmPercent, baseline.rmsEvmPercent)
        for guard in (-0.01, 0.5, 1.0, float("nan")):
            result = nrdemod.demodulate(iq, config, cpCfoGuardFraction=guard)
            self.assertFalse(result.ok)

    def test_nbiot_presets_and_plain_downlink(self):
        uplink = nrdemod.nbiot_config(1.92e6, uplink=True)
        self.assertTrue(uplink.transformPrecoding)
        self.assertFalse(uplink.burstSearch)
        self.assertEqual(uplink.carrierOffsetHz, 7500)
        self.assertFalse(uplink.dmrsEnabled)
        self.assertEqual(uplink.syncSearchSymbols, 14)
        config = nrdemod.nbiot_config(1.92e6, burstSearch=False, modulation=nrdemod.QPSK,
                         removeIqOffset=False)
        iq = nrdemod.generate_test_signal(config, num_symbols=28)
        result = nrdemod.demodulate_nbiot(iq, 1.92e6, burstSearch=False, modulation=nrdemod.QPSK,
                         removeIqOffset=False)
        self.assertTrue(result.ok, result.error)
        self.assertLess(result.rmsEvmPercent, 0.1)

    def test_npss_frame_and_dc(self):
        signs = [1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1]
        frame = [0j]*512
        for symbol in range(28):
            spectrum = np.zeros(128, dtype=np.complex128)
            if symbol % 14 >= 3:
                for carrier in range(12):
                    value = np.exp(1j*(np.pi/4 + ((symbol*7+carrier*5) % 4)*np.pi/2))
                    if symbol < 14:
                        value = 0j if carrier == 11 else signs[symbol-3]*np.exp(
                            -5j*np.pi*carrier*(carrier+1)/11)
                    spectrum[(carrier-6) % 128] = value
            samples = np.fft.ifft(spectrum)
            for point in range(-10 if symbol % 7 == 0 else -9, 128):
                frame.append(samples[point % 128]*np.exp(1j*np.pi*point/128))
        frame = np.asarray(frame+[0j]*512, dtype=np.complex64)
        for dc in (0j, 0.003-0.002j):
            result = nrdemod.demodulate_nbiot(frame+dc, 1.92e6, downlink_frame=True, maxSymbols=28)
            self.assertTrue(result.ok, result.error)
            self.assertGreater(result.syncCorrelation, 0.99)
            self.assertLess(result.rmsEvmPercent, 0.1)
            self.assertEqual(result.numSymbols, 28)
            self.assertEqual(result.timingOffset, 512)

    def test_npss_rejects_invalid_inputs(self):
        iq = np.zeros(5000, dtype=np.complex64)
        for settings in ({"maxSymbols": 13}, {"maxSymbols": 154},
                         {"maxSymbols": 28, "uplink": True}, {"maxSymbols": 28},
                         {"maxSymbols": 28, "sample_rate": float("nan")}):
            options = {"sample_rate": 1.92e6, "downlink_frame": True, **settings}
            self.assertFalse(nrdemod.demodulate_nbiot(iq, **options).ok)

    @unittest.skipUnless(os.environ.get("NRDEMOD_NBIOT_UL_CAPTURE_DIR"), "external NB captures not configured")
    def test_full_uplink_captures(self):
        paths = sorted(Path(os.environ["NRDEMOD_NBIOT_UL_CAPTURE_DIR"]).glob("*.txt"))
        self.assertEqual(len(paths), 14)
        for path in paths:
            with path.open() as stream:
                header = {}
                for line in stream:
                    if line.strip() == "Y":
                        break
                    key, _, value = line.partition("\t")
                    header[key] = value.strip()
                rows = np.loadtxt(stream)
            rate = 1/float(header["XDelta"])
            iq = rows[:, 0]+1j*rows[:, 1]
            self.assertEqual(iq.size, 38400)
            for cfo in (0, -2000, 2000):
                with self.subTest(capture=path.name, cfo=cfo):
                    signal = iq*np.exp(2j*np.pi*cfo*np.arange(iq.size)/rate)
                    result = nrdemod.demodulate_nbiot(signal, rate, uplink=True)
                    self.assertTrue(result.ok, result.error)
                    self.assertGreaterEqual(result.numSymbols, 270)
                    self.assertLess(result.rmsEvmPercent, 1.1383)
                    self.assertLess(abs(result.frequencyErrorHz-cfo), 10)


if __name__ == "__main__":
    unittest.main()