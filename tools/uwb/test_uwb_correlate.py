"""Regression tests for UWB correlation verdicts; no VSA or native DLL required."""
import math
import contextlib
import io
import unittest
from unittest.mock import patch

import uwb_correlate as uwb


class CorrelationTests(unittest.TestCase):
    def setUp(self):
        self.pr = {"mode": "BPRF", "sp": 0, "generator": "keysight"}
        self.nat = {"native_status": "sync", "native_cfo_hz": 0.0, "native_data_status": "decoded",
                    "native_payload_hex": "0123"}
        self.py = {"py_status": "decoded", "py_cfo_hz": 0.0, "py_payload_match": True, "py_payload_hex": "0123"}
        self.vsa = {"ShrNrmse": 0.1, "PhrNrmse": 0.01, "PsduNrmse": 0.01, "FreqErr": 0.0,
                    "DecodedPayloadHex": "0123"}

    def verdict(self, cfo=0.0):
        return uwb.correlate({}, self.pr, 1996.8e6, None, self.nat, self.py, self.vsa, cfo)

    def test_good_frame(self):
        self.assertTrue(self.verdict()["pass"])

    def test_zero_nrmse_is_valid(self):
        self.vsa.update(ShrNrmse=0, PhrNrmse=0, PsduNrmse=0)
        self.assertTrue(self.verdict()["pass"])

    def test_missing_or_invalid_required_metrics_fail(self):
        for key in ("ShrNrmse", "PhrNrmse", "PsduNrmse"):
            for value in (None, math.nan, math.inf, -1):
                with self.subTest(key=key, value=value):
                    old = self.vsa[key]
                    self.vsa[key] = value
                    self.assertFalse(self.verdict()["pass"])
                    self.vsa[key] = old

    def test_threshold(self):
        self.vsa["PsduNrmse"] = uwb.NRMSE_DATA_MAX
        self.assertTrue(self.verdict()["pass"])
        self.vsa["PsduNrmse"] += 0.0001
        self.assertFalse(self.verdict()["pass"])

    def test_dual_no_lock_is_not_a_pass(self):
        self.nat["native_status"] = "no-burst"
        self.vsa["ShrNrmse"] = 100
        self.assertFalse(self.verdict()["pass"])

    def test_python_failure_is_not_ignored(self):
        for status in ("no-psdu", "error ValueError", "decoded (RS failure)", "shr-mismatch"):
            with self.subTest(status=status):
                self.py["py_status"] = status
                self.assertFalse(self.verdict()["pass"])

    def test_payload_mismatch(self):
        self.py["py_payload_match"] = False
        self.assertFalse(self.verdict()["pass"])

    def test_missing_expected_payload_comparison_fails(self):
        self.pr["payload_hex"] = "00"
        del self.py["py_payload_match"]
        self.assertFalse(self.verdict()["pass"])

    def test_vsa_payload_is_compared_even_when_nrmse_passes(self):
        self.pr["payload_hex"] = "0123"
        self.vsa["DecodedPayloadHex"] = "0123"
        self.assertTrue(self.verdict()["pass"])
        self.vsa["DecodedPayloadHex"] = "0122"
        result = self.verdict()
        self.assertFalse(result["pass"])
        self.assertEqual(result["vsa_payload_bit_errors"], 1)
        del self.vsa["DecodedPayloadHex"]
        self.assertFalse(self.verdict()["pass"])

    def test_vsa_and_python_payloads_are_compared_without_tx_reference(self):
        self.py["py_payload_hex"] = "0123"
        self.vsa["DecodedPayloadHex"] = "0123"
        self.assertTrue(self.verdict()["pass"])
        self.vsa["DecodedPayloadHex"] = "0023"
        self.assertFalse(self.verdict()["pass"])

    def test_vsa_octet_bit_order(self):
        self.assertEqual(uwb.decoded_payload([0x6A, 0xF6, 0x22]), bytes.fromhex("566f44"))
        self.assertEqual(uwb.decoded_payload([]), b"")
        for invalid in ([256], [-1], [math.nan], [1.5], [[0]]):
            with self.subTest(raw=invalid), self.assertRaises(ValueError):
                uwb.decoded_payload(invalid)

    def test_payload_error_count_includes_length_difference(self):
        self.assertEqual(uwb.payload_bit_errors(b"\x00\xff", b"\x01"), 9)
        self.assertEqual(uwb.payload_bit_errors(b"", b"\x00"), 8)

    def test_matching_wrong_cfo_fails(self):
        self.nat["native_cfo_hz"] = self.py["py_cfo_hz"] = self.vsa["FreqErr"] = 200000
        self.assertFalse(self.verdict()["pass"])
        self.assertTrue(self.verdict(200000)["pass"])

    def test_missing_or_invalid_cfo_fails(self):
        for value in (None, math.nan, math.inf, "invalid"):
            self.vsa["FreqErr"] = value
            self.assertFalse(self.verdict()["pass"])

    def test_sp3_requires_sts_but_no_data_metrics(self):
        self.pr["sp"] = 3
        self.nat["native_data_status"] = "shr-ok"
        self.py["py_status"] = "shr-ok"
        self.vsa = {"ShrNrmse": 0.1, "StsNrmse": 0.1, "FreqErr": 0}
        self.assertTrue(self.verdict()["pass"])
        del self.vsa["StsNrmse"]
        self.assertFalse(self.verdict()["pass"])

    def test_sp1_requires_sts(self):
        self.pr["sp"] = 1
        self.assertFalse(self.verdict()["pass"])
        self.vsa["StsNrmse"] = 0.1
        self.assertTrue(self.verdict()["pass"])

    def test_vsa_runtime_error_fails(self):
        self.vsa = {"_error": "timeout after 900s"}
        self.assertFalse(self.verdict()["pass"])

    def test_skipped_vsa_is_not_comparable(self):
        self.vsa = {"_error": "VSA skipped (--no-vsa)"}
        self.assertIsNone(self.verdict()["pass"])
        self.py["py_status"] = "error ValueError"
        self.assertFalse(self.verdict()["pass"])

    def test_unsupported_is_not_a_pass(self):
        self.pr["mode"] = "EHPRF"
        self.nat = {"native_status": "unsupported (EHPRF)"}
        self.py = {"py_status": "unsupported (EHPRF)"}
        self.vsa = {"_error": "unsupported: no EHPRF mode"}
        self.assertIsNone(self.verdict()["pass"])

    def test_native_error_with_vsa_skipped_fails(self):
        self.nat = {"native_status": "error OSError"}
        self.vsa = {"_error": "VSA skipped (--no-vsa)"}
        self.assertFalse(self.verdict()["pass"])

    def test_native_acquisition_alone_cannot_pass(self):
        del self.nat["native_data_status"]
        self.assertFalse(self.verdict()["pass"])

    def test_native_payload_must_match(self):
        for value in (None, "", "0122", "012300"):
            self.nat["native_payload_hex"] = value
            self.assertFalse(self.verdict()["pass"])

    def test_native_data_error_cannot_pass(self):
        self.nat["native_data_error"] = "Reed-Solomon decode failed"
        self.assertFalse(self.verdict()["pass"])

    def test_hprf_requires_native_payload_decoding(self):
        self.pr["mode"] = "HPRF"
        self.assertTrue(self.verdict()["pass"])
        del self.nat["native_data_status"]
        self.assertFalse(self.verdict()["pass"])

    def test_hprf_compares_against_vsa_when_tx_reference_unavailable(self):
        self.pr["mode"] = "HPRF"
        self.py = {"py_status": "unsupported (HPRF)"}
        self.assertTrue(self.verdict()["pass"])
        self.vsa["DecodedPayloadHex"] = "0122"
        self.assertFalse(self.verdict()["pass"])
        self.vsa = {"_error": "VSA skipped (--no-vsa)"}
        result = self.verdict()
        self.assertIsNone(result["pass"])
        self.assertIsNone(result["native_payload_match"])

    def test_skip_unsupported_excludes_only_ehprf(self):
        items = [{"source": "generated", "stem": mode} for mode in ("4a", "BPRF", "HPRF", "EHPRF")]
        output = io.StringIO()
        with patch.object(uwb, "discover", return_value=items), patch.object(
                uwb, "params", side_effect=lambda item: {"mode": item["stem"]}), contextlib.redirect_stdout(output):
            self.assertEqual(uwb.main(["--list", "--skip-unsupported"]), 0)
        text = output.getvalue()
        self.assertIn("skipping 1 unsupported EHPRF waveforms", text)
        self.assertIn("3 waveforms", text)
        self.assertIn("generated 4a", text)
        self.assertNotIn("generated EHPRF", text)

    def test_generated_hprf124_export_has_requested_physical_prf(self):
        from scipy.signal import find_peaks

        items = uwb.discover(["UWB_4z_HPRF_ch5_CI27_124M8_PSDU100"])
        if not items:
            self.skipTest("run gen_uwb in MATLAB to generate the export fixture")
        item = items[0]
        metadata = uwb.read_inputs(item["para"])
        iq, fs, _ = uwb.L.parse_waveform(item["txt"])
        spp = int(metadata["SamplesPerPulse"])
        self.assertEqual(int(metadata["ConstraintLength"]), 7)
        pad = round(2e-6 * fs)
        # Last 800 CL7 symbols, including termination: 16 pulses per 64 grid chips.
        data = iq[-pad - 800 * 64 * spp:-pad]
        peaks, _ = find_peaks(abs(data), height=0.5 * max(abs(data)), distance=6)
        self.assertEqual(len(peaks), 12800)
        self.assertAlmostEqual(len(peaks) * fs / len(data), float(metadata["MeanPRF"]) * 1e6)
        self.assertEqual(float(metadata["MatlabSampleRate"]), 998.4e6)
        self.assertIn("SampleRateCorrection", metadata)


if __name__ == "__main__":
    unittest.main()
