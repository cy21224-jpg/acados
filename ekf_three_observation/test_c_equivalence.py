"""Validate the Python EKF against the exact uploaded EKF2 and STM32 helpers.

Run from the repository root with NumPy/SciPy/Matplotlib/CasADi installed:
    EKF_PYTHON_FILE="$PWD/DRL_EKF_func_series.py" \
    EKF_C_FILE="/path/to/function_kalmanfilter.c" \
    EKF_STM32_PROJECT="/path/to/3linc_DRL_EKF_MPFC" \
    python3 ekf_three_observation/test_c_equivalence.py

The STM32 project must contain Core/Src/RIM_functions.c and its Core/Inc header.
Test artifacts are written beside this script; source C and firmware stay intact.

No firmware source is edited. Test-only headers supply the global sampling_time;
the complete uploaded C translation unit and existing RIM_functions.c are built.
"""
from pathlib import Path
import contextlib
import ctypes as ct
import hashlib
import importlib.util
import io
import json
import os
import subprocess
import unittest

import numpy as np

TESTS = Path(__file__).resolve().parent
ROOT = TESTS.parent


def first_existing(candidates):
    return next((p for p in candidates if p.exists()), candidates[0])


PYTHON_FILE = Path(os.environ.get("EKF_PYTHON_FILE", str(ROOT / "DRL_EKF_func_series.py")))
C_SOURCE = Path(os.environ.get("EKF_C_FILE", os.environ.get("EKF_C_SOURCE", str(first_existing([
    TESTS / "function_kalmanfilter.c", ROOT / "function_kalmanfilter.c",
    ROOT / "Core/Src/function_kalmanfilter.c",
    Path("/workspace/attachments/ea866555-f44d-468b-ba21-9f6cc337a272/function_kalmanfilter.c"),
])))))
SDK = Path(os.environ.get("EKF_STM32_PROJECT", str(first_existing([
    ROOT / "project", ROOT / "3linc_DRL_EKF_MPFC",
    ROOT if (ROOT / "Core/Src/RIM_functions.c").is_file() else ROOT / "missing-project",
    Path("/workspace/work/mpfc_latest_analysis/original/3linc_DRL_EKF_MPFC"),
]))))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_python():
    spec = importlib.util.spec_from_file_location("latest_ekf", PYTHON_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.DRL_EKF_func_series


def build_native():
    headers = TESTS / "native_headers"
    headers.mkdir(exist_ok=True)
    (headers / "param_3link.h").write_text("extern float sampling_time;\n")
    (headers / "function_3link.h").write_text("/* Unused by the uploaded EKF2 body. */\n")
    (TESTS / "sampling_time.c").write_text("float sampling_time;\n")
    output = {}
    for optimization in ["O0", "O2"]:
        library = TESTS / ("uploaded_EKF2_" + optimization + ".so")
        command = ["gcc", "-std=c11", "-" + optimization, "-ffp-contract=off", "-shared", "-fPIC",
                   "-I" + str(headers), "-I" + str(SDK / "Core/Inc"),
                   str(C_SOURCE), str(SDK / "Core/Src/RIM_functions.c"),
                   str(TESTS / "sampling_time.c"), "-lm", "-o", str(library)]
        result = subprocess.run(command, text=True, capture_output=True)
        (TESTS / ("native_build_" + optimization + ".log")).write_text(result.stdout + result.stderr)
        result.check_returncode()
        lib = ct.CDLL(str(library))
        ptr = ct.POINTER(ct.c_float)
        lib.EKF2.argtypes = [ptr, ptr, ct.c_float, ptr, ptr, ptr, ptr, ptr, ptr]
        lib.EKF2.restype = None
        output[optimization] = lib
    return output


def native_step(lib, y, u, mass, x, P, R, Q, T):
    arrays = [np.ascontiguousarray(v, dtype=np.float32) for v in [y, u, x, P, R, Q]]
    xo = np.empty(3, dtype=np.float32)
    po = np.empty((3, 3), dtype=np.float32)
    ptr = ct.POINTER(ct.c_float)
    ct.c_float.in_dll(lib, "sampling_time").value = float(T)
    args = [a.ctypes.data_as(ptr) for a in arrays]
    lib.EKF2(args[0], args[1], float(mass), args[2], args[3], args[4], args[5], xo.ctypes.data_as(ptr), po.ctypes.data_as(ptr))
    return xo, po


def prediction(x, u, mass, T):
    f = np.float32
    x, u = np.asarray(x, dtype=f), np.asarray(u, dtype=f)
    mass, T = f(mass), f(T)
    pred = np.array([x[0] - T*x[2]*x[1]/mass, T*x[0]+x[1], x[2]], dtype=f)
    pred[0] += (T/mass)*u[0]
    return pred


def observations(pred):
    return np.array([pred[0], pred[1], pred[2]*pred[1]], dtype=np.float32)


class NativeEquivalence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.libraries = build_native()
        cls.filter_class = load_python()
        cls.filter = cls.filter_class.__new__(cls.filter_class)
        cls.metrics = {"reference_function": "EKF2", "python_method": "EKF", "arithmetic": "IEEE-754 float32, scalar loop order",
            "source_sha256": sha(C_SOURCE), "helpers_sha256": sha(SDK / "Core/Src/RIM_functions.c"),
            "compiler": subprocess.check_output(["gcc", "--version"], text=True).splitlines()[0],
            "native_builds": ["-O0 -ffp-contract=off", "-O2 -ffp-contract=off"],
            "firmware_modified": False, "stm32_bitwise_equivalence_measured": False, "cases": {},
            "comparisons_per_native_build": 0, "max_abs_state_error": 0.0, "max_abs_covariance_error": 0.0}

    @classmethod
    def tearDownClass(cls):
        cls.metrics["source_unchanged"] = sha(C_SOURCE) == cls.metrics["source_sha256"]
        cls.metrics["helpers_unchanged"] = sha(SDK / "Core/Src/RIM_functions.c") == cls.metrics["helpers_sha256"]
        (TESTS / "c_equivalence_results.json").write_text(json.dumps(cls.metrics, indent=2) + "\n")

    def compare(self, y, u, mass, x, P, R, Q, T, *, nonfinite=False):
        arrays = [np.asarray(v, dtype=np.float32) for v in [y, u, x, P, R, Q]]
        y, u, x, P, R, Q = arrays
        saved = [a.copy() for a in arrays]
        self.filter.T = np.float32(T)
        with np.errstate(all="ignore"), contextlib.redirect_stdout(io.StringIO()):
            px, pp = self.filter.EKF(y, u, mass, x, P, R, Q)
        px, pp = np.asarray(px, dtype=np.float32), np.asarray(pp, dtype=np.float32)
        self.assertEqual(px.shape, (3,))
        self.assertEqual(pp.shape, (3, 3))
        for label, lib in self.libraries.items():
            cx, cp = native_step(lib, y, u, mass, x, P, R, Q, T)
            for actual, expected in [(px, cx), (pp, cp)]:
                if nonfinite:
                    np.testing.assert_array_equal(np.isnan(actual), np.isnan(expected), err_msg=label)
                    np.testing.assert_array_equal(np.isposinf(actual), np.isposinf(expected), err_msg=label)
                    np.testing.assert_array_equal(np.isneginf(actual), np.isneginf(expected), err_msg=label)
                    finite = np.isfinite(expected)
                    np.testing.assert_array_equal(actual[finite].view(np.uint32), expected[finite].view(np.uint32), err_msg=label)
                else:
                    self.assertTrue(np.all(np.isfinite(actual)))
                    np.testing.assert_array_equal(actual.view(np.uint32), expected.view(np.uint32), err_msg=label)
            if np.all(np.isfinite(px)) and np.all(np.isfinite(cx)):
                self.metrics["max_abs_state_error"] = max(self.metrics["max_abs_state_error"], float(np.max(np.abs(px-cx))))
            if np.all(np.isfinite(pp)) and np.all(np.isfinite(cp)):
                self.metrics["max_abs_covariance_error"] = max(self.metrics["max_abs_covariance_error"], float(np.max(np.abs(pp-cp))))
        self.metrics["comparisons_per_native_build"] += 1
        for actual, before in zip(arrays, saved):
            np.testing.assert_array_equal(actual, before)
        return px, pp

    def test_inverse_pivot_threshold_matches_C_double_comparison(self):
        pivot = np.float32(1e-14)
        for value in [np.nextafter(pivot, np.float32(0.)), pivot,
                      np.nextafter(pivot, np.float32(np.inf))]:
            matrix = np.diag(np.array([1., 1., value], dtype=np.float32))
            pinv = np.empty((3, 3), dtype=np.float32)
            with contextlib.redirect_stdout(io.StringIO()):
                pstatus = self.filter.inv_mat(3, matrix, pinv)
            for label, lib in self.libraries.items():
                lib.inv_mat.argtypes = [ct.c_int, ct.POINTER(ct.c_float), ct.POINTER(ct.c_float)]
                lib.inv_mat.restype = ct.c_int
                cinv = np.empty((3, 3), dtype=np.float32)
                cstatus = lib.inv_mat(3, matrix.ctypes.data_as(ct.POINTER(ct.c_float)), cinv.ctypes.data_as(ct.POINTER(ct.c_float)))
                self.assertEqual(pstatus, cstatus, label)
                np.testing.assert_array_equal(pinv.view(np.uint32), cinv.view(np.uint32))
        self.metrics["cases"]["inverse_threshold_boundary"] = "status and inverse match at float32(1e-14) and adjacent float32 values; C double threshold preserved"

    def test_constructor_measurement_dimension(self):
        instance = self.filter_class()
        self.assertEqual(np.asarray(instance.Rkf).shape, (3, 3))
        self.assertEqual(np.asarray(instance.Qkf).shape, (3, 3))
        self.assertEqual(np.asarray(instance.x_init).size, 3)
        self.metrics["cases"]["constructor"] = "three-state and three-observation covariance shapes"

    def test_500_random_correlated_cases(self):
        rng = np.random.default_rng(778123)
        for case in range(500):
            x = [rng.uniform(-.4, .4), rng.uniform(-.04, .04), rng.uniform(-200, 1500)]
            A = rng.normal(size=(3, 3)); D = np.diag([.03, .003, 3.])
            P = D @ (A @ A.T + np.eye(3)) @ D
            A = rng.normal(size=(3, 3)); Q = 1e-3*(A @ A.T + np.eye(3))
            A = rng.normal(size=(3, 3)); D = np.diag([.01, .002, .1])
            R = D @ (A @ A.T + np.eye(3)) @ D
            mass = rng.choice([.1, .2, .9, 1., 2., 10.])
            T = rng.choice([.001, .005, .01, .02, .04])
            u = rng.uniform(-2, 2, size=3)
            y = observations(prediction(x, u, mass, T)) + rng.normal(size=3)*[.02, .005, .3]
            self.compare(y, u, mass, x, P, R, Q, T)
        self.metrics["cases"]["random_500"] = "bitwise equal state and covariance for both native builds; signed x/k, correlated P/Q/R, nonzero u[0:3], variable T/M"

    def test_zero_innovation_noncontact_origin_and_signed_stiffness(self):
        for x in [[.2, -.02, 100.], [0., 0., 100.], [.1, .01, -50.], [-.1, .005, 200.]]:
            mass, T = .9, .01
            u = [1.2, .3, -.4]
            pred = prediction(x, u, mass, T)
            y = observations(pred)
            px, _ = self.compare(y, u, mass, x, np.eye(3), np.eye(3)*1e-3, np.eye(3)*1e-3, T)
            np.testing.assert_array_equal(px.view(np.uint32), pred.view(np.uint32))
        self.metrics["cases"]["zero_innovation"] = "negative position, x=0, signed stiffness; bilateral k*x preserved; nonlinear h produces zero correction"

    def test_only_force_input_affects_deterministic_prediction(self):
        inputs = [[.8, 0., 0.], [.8, 15., -800.]]
        answers = []
        for u in inputs:
            answers.append(self.compare([.01, .002, .7], u, .9, [.02, .003, 100.], np.eye(3), np.eye(3)*.01, np.eye(3)*.001, .01))
        for a, b in zip(answers[0], answers[1]):
            np.testing.assert_array_equal(a.view(np.uint32), b.view(np.uint32))
        self.metrics["cases"]["deterministic_input_mapping"] = "u1/u2 finite do not affect state/covariance; Bd differs from Bd_KF"

    def test_force_observation_changes_stiffness(self):
        outputs = []
        for force in [.2, 1.2]:
            outputs.append(self.compare([0., .01, force], [1., 0., 0.], 1., [0., .01, 80.], np.diag([.01, .001, 100.]), np.diag([.01, .001, .001]), np.eye(3)*.001, .01)[0])
        self.assertGreater(abs(float(outputs[1][2]-outputs[0][2])), .01)
        self.metrics["cases"]["force_observation"] = {"k_with_force_0_2": float(outputs[0][2]), "k_with_force_1_2": float(outputs[1][2])}

    def test_predicted_observation_jacobian_and_nonlinear_output(self):
        instance = self.filter_class.__new__(self.filter_class)
        instance.T = .01
        original = instance.transpose_mat
        captured = []
        def observe(row, col, matrix, output):
            captured.append(np.array(matrix, dtype=np.float32))
            return original(row, col, matrix, output)
        instance.transpose_mat = observe
        x = [.3, -.001, 400.]; u = [1.1, 5., 6.]; mass = .9
        pred = prediction(x, u, mass, instance.T)
        y = observations(pred)
        result, _ = instance.EKF(y, u, mass, x, np.eye(3), np.eye(3)*.01, np.eye(3)*.001)
        np.testing.assert_array_equal(captured[2], [[1., 0., 0.], [0., 1., 0.], [0., pred[2], pred[1]]])
        np.testing.assert_array_equal(np.array(result, dtype=np.float32).view(np.uint32), pred.view(np.uint32))
        self.assertNotEqual(float(captured[2][2] @ pred), float(y[2]))
        self.metrics["cases"]["nonlinear_H_h"] = "H uses predicted k/x; innovation uses k*x rather than H*x (which doubles force)"

    def test_300_step_independent_recurrence(self):
        rng = np.random.default_rng(321998)
        px = np.array([.01, .008, 80.], dtype=np.float32)
        pp = np.diag([.1, .01, 100.]).astype(np.float32)
        cstates = {label: (px.copy(), pp.copy()) for label in self.libraries}
        true = np.array([0., .01, 100.], dtype=np.float32)
        T, mass = np.float32(.01), np.float32(.9)
        R = np.diag([1e-4, 1e-6, .01]).astype(np.float32)
        Q = (np.eye(3)*1e-3).astype(np.float32)
        self.filter.T = T
        for step in range(300):
            u = np.array([1.+.2*np.sin(step*.03), 0., 0.], dtype=np.float32)
            true = prediction(true, u, mass, T)
            y = (observations(true) + rng.normal(size=3)*[.001, .0001, .01]).astype(np.float32)
            with np.errstate(all="ignore"):
                px, pp = self.filter.EKF(y, u, mass, px, pp, R, Q)
            px, pp = np.asarray(px, dtype=np.float32), np.asarray(pp, dtype=np.float32)
            for label, lib in self.libraries.items():
                cx, cp = native_step(lib, y, u, mass, *cstates[label], R, Q, T)
                np.testing.assert_array_equal(px.view(np.uint32), cx.view(np.uint32), err_msg=f"{label} step {step}")
                np.testing.assert_array_equal(pp.view(np.uint32), cp.view(np.uint32), err_msg=f"{label} step {step}")
                cstates[label] = (cx, cp)
        self.metrics["cases"]["independent_recurrence_300"] = "bitwise equal at every feedback step, separate C and Python evolving state and covariance"

    def test_column_vectors_and_no_input_mutation(self):
        self.filter.T = .01
        arrays = [np.array([.01, .003, .4])[:, None], np.array([1., 0., 0.])[:, None], np.array([0., .002, 100.])[:, None], np.eye(3), np.eye(3)*.01, np.eye(3)*.001]
        originals = [a.copy() for a in arrays]
        answer = self.filter.EKF(arrays[0], arrays[1], .9, arrays[2], *arrays[3:])
        self.assertEqual(np.asarray(answer[0]).shape, (3,))
        self.assertEqual(np.asarray(answer[1]).shape, (3, 3))
        for a, b in zip(arrays, originals): np.testing.assert_array_equal(a, b)
        with self.assertRaises(ValueError):
            self.filter.EKF([0., 0.], [0., 0., 0.], .9, [0., 0., 100.], np.eye(3), np.eye(3), np.eye(3))
        self.metrics["cases"]["python_interface"] = "column vectors accepted; input arrays unchanged; old two-observation shape rejected"

    def test_singular_negative_mass_and_nonfinite_parity(self):
        zero = np.zeros((3, 3))
        # C ignores the inverse failure return and continues with partial inverse.
        self.compare([.1, .02, .8], [1., 0., 0.], .9, [0., .01, 100.], zero, zero, zero, .01)
        self.compare([.1, .02, .8], [1., 2., 3.], -.9, [0., .01, 100.], np.eye(3), np.eye(3)*.01, np.eye(3)*.001, .01)
        self.compare([np.nan, .02, .8], [1., 0., 0.], .9, [0., .01, 100.], np.eye(3), np.eye(3)*.01, np.eye(3)*.001, .01, nonfinite=True)
        self.compare([.1, .02, .8], [1., 0., 0.], 0., [0., .01, 100.], np.eye(3), np.eye(3)*.01, np.eye(3)*.001, .01, nonfinite=True)
        self.metrics["cases"]["C_error_semantics"] = "ignored inverse failure reproduced; negative mass accepted as in C; NaN/zero-mass classifications agree"


if __name__ == "__main__":
    unittest.main(verbosity=2)
