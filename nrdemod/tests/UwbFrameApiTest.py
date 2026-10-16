"""Run after generate_uwb_frame_vectors; MATLAB is not needed for decoding."""

import ctypes
import json
import unittest
from pathlib import Path

import numpy as np

from nrdemod import _demod_ffi, demod


VECTORS = Path(__file__).resolve().parents[1] / "build" / "frame-tests" / "vectors"


class UwbFrameApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (VECTORS / "codebook.json").exists():
            raise unittest.SkipTest("Generate reference vectors with generate_uwb_frame_vectors.m first")

    def test_independent_frames(self):
        paths = sorted(VECTORS.glob("vector_*.json"))
        self.assertGreaterEqual(len(paths), 77)
        for path in paths:
            metadata = json.loads(path.read_text())
            with self.subTest(vector=path.stem, label=metadata["label"]):
                iq = np.fromfile(path.with_suffix(".bin"), dtype="<c8")
                options = demod.UwbFrameConfig(**metadata["frame"])
                config = demod.Config(
                    sampleRate=metadata["sampleRate"], phyMode=metadata["mode"],
                    syncLength=metadata["syncLength"],
                    chipRateHz=metadata["sampleRate"] / options.samplesPerPulse,
                )
                result = demod.demodulate_uwb_frame(iq, config, options)
                self.assertTrue(result.ok, result.error)
                self.assertTrue(result.phrValid)
                self.assertTrue(result.psduValid)
                expected = metadata["bytes"]
                if isinstance(expected, int):
                    expected = [expected]
                self.assertEqual(list(result.psduBytes), expected)
                self.assertTrue(result.evmValid)
                self.assertLess(result.shrEvmRmsPercent, 0.1)
                self.assertLess(result.phrEvmRmsPercent, 0.1)
                self.assertLess(result.psduEvmRmsPercent, 0.1)
                self.assertEqual(result.macFcsChecked, options.macFcsLength != 0)
                if options.macFcsLength:
                    self.assertTrue(result.macFcsValid)

    def test_all_preamble_tables(self):
        codes = json.loads((VECTORS / "codebook.json").read_text())
        signs = np.array([0, 1, 0, -1, 1, 0, 0, -1])
        self.assertEqual(len(codes), 32)
        for index, code in enumerate(codes, 1):
            with self.subTest(code=index):
                spread = 16 if index <= 8 else 4
                symbol = np.zeros(len(code) * spread * 4, dtype=np.complex64)
                symbol[::spread * 4] = code
                iq = np.concatenate((np.zeros(2048), np.tile(symbol, 64),
                                     (signs[:, None] * symbol).ravel(), np.zeros(4096)))
                config = demod.Config(sampleRate=1996.8e6, phyMode=1 if index >= 25 else 0)
                options = demod.UwbFrameConfig(
                    codeIndex=index, legacy=index < 25,
                    legacyMeanPrfMHz=15.6 if index <= 8 else 62.4,
                )
                result = demod.demodulate_uwb_frame(iq, config, options)
                self.assertTrue(result.sfdFound, result.error)
                self.assertGreater(result.sfdRho, 0.99999)
                self.assertEqual(result.syncStart, 2048)

    def test_old_c_option_layouts(self):
        metadata = json.loads((VECTORS / "vector_019.json").read_text())
        iq = np.fromfile(VECTORS / "vector_019.bin", dtype="<f4")
        config = demod.Config(sampleRate=metadata["sampleRate"], phyMode=1,
                              syncLength=metadata["syncLength"])._to_c(demod.UWB_HRP)
        values = vars(demod.UwbFrameConfig(**metadata["frame"])).copy()
        values["stsKeyHex"] = values["stsKeyHex"].encode("ascii")
        values["stsIvHex"] = values["stsIvHex"].encode("ascii")
        for field_count, expected_size in [(9, 48), (12, 64)]:
            with self.subTest(size=expected_size):
                class OldOptions(ctypes.Structure):
                    _fields_ = _demod_ffi.UwbFrameConfig._fields_[:field_count]
                self.assertEqual(ctypes.sizeof(OldOptions), expected_size)
                old = OldOptions(structSize=expected_size, **{
                    name: values[name] for name, _ in OldOptions._fields_[1:]
                })
                pointer = ctypes.cast(ctypes.byref(old), ctypes.POINTER(_demod_ffi.UwbFrameConfig))
                handle = _demod_ffi.lib.demod_uwb_frame(
                    iq.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), iq.size // 2,
                    ctypes.byref(config), pointer,
                )
                self.assertTrue(handle)
                try:
                    result = demod.Result(demod.UWB_HRP, handle)
                finally:
                    _demod_ffi.lib.demod_result_free(handle)
                self.assertTrue(result.ok, result.error)
                self.assertEqual(list(result.psduBytes), metadata["bytes"])


if __name__ == "__main__":
    unittest.main(verbosity=2)