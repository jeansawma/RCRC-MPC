"""Independent continuous model and constrained optimizer verification."""
import ctypes
import pathlib
import subprocess
import numpy as np
import pytest
from scipy.linalg import expm
from scipy.optimize import minimize, LinearConstraint, Bounds

ROOT = pathlib.Path(__file__).resolve().parents[1]

@pytest.fixture(scope="module")
def oracle(tmp_path_factory):
    target = tmp_path_factory.mktemp("oracle") / "oracle.so"
    subprocess.run(["gcc", "-std=c11", "-O2", "-shared", "-fPIC",
                    "-Ifirmware/include", "tests/oracle_bridge.c", "firmware/src/control.c",
                    "-lm", "-o", str(target)], cwd=ROOT, check=True)
    library = ctypes.CDLL(str(target))
    array = np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags="C_CONTIGUOUS")
    library.oracle_run.argtypes = [array, array]
    library.oracle_run.restype = ctypes.c_int
    return library.oracle_run

def test_model_and_qp_against_scipy(oracle):
    rng = np.random.default_rng(723)
    largest_action_error = 0.0
    for _ in range(60):
        n = int(rng.integers(2, 17))
        ts = float(rng.choice([500, 1000, 2000, 5000]))
        r1, r2 = rng.uniform(300, 900, 2)
        c1, c2 = rng.uniform(60e-6, 200e-6, 2)
        ref = rng.uniform(1, 15)
        slew = rng.uniform(.03, .4)
        q, penalty = 1., rng.uniform(.03, .5)
        v1, v2 = rng.uniform(0, 18, 2)
        bus = rng.uniform(20, 29)
        previous = rng.uniform(0, 1)
        settings = np.array([n,ts,r1,r2,c1,c2,ref,slew,q,penalty,v1,v2,bus,previous], np.float32)
        output = np.zeros(9, np.float32)
        assert oracle(settings, output) == 0, (settings, output)
        # Use actual float input values so only numerical implementations differ.
        _,ts,r1,r2,c1,c2,ref,slew,q,penalty,v1,v2,bus,previous = settings.astype(float)
        continuous = np.array([[-(1/r1+1/r2+1/55700)/c1, 1/(r2*c1)],
                               [1/(r2*c2), -(1/r2+1/55700)/c2]])
        augmented = np.zeros((3,3)); augmented[:2,:2] = continuous
        augmented[0,2] = 1/(r1*c1)
        discrete = expm(augmented*ts*1e-6)
        a,b = discrete[:2,:2],discrete[:2,2]
        np.testing.assert_allclose(output[:4].reshape(2,2), a, rtol=2e-6, atol=2e-8)
        np.testing.assert_allclose(output[4:6], b, rtol=2e-6, atol=2e-8)
        # Build horizon predictions independently with powers, then solve by SLSQP.
        free = np.array([(np.linalg.matrix_power(a,k+1) @ [v1,v2])[1] for k in range(n)])
        response = np.zeros((n,n))
        for k in range(n):
            for j in range(k+1):
                response[k,j] = (np.linalg.matrix_power(a,k-j) @ b)[1]*bus
        difference = np.eye(n)-np.eye(n,k=-1)
        offset = np.zeros(n); offset[0] = previous
        def objective(u):
            e=free+response@u-ref; du=difference@u-offset
            return q*(e@e)+penalty*(du@du)
        def gradient(u):
            return 2*q*response.T@(free+response@u-ref)+2*penalty*difference.T@(difference@u-offset)
        solution = minimize(objective, np.full(n,previous), jac=gradient,
                            bounds=Bounds(0,1), constraints=LinearConstraint(difference,offset-slew,offset+slew),
                            method="SLSQP", options={"ftol":1e-10,"maxiter":300})
        assert solution.success, solution.message
        difference_action=abs(float(output[6])-solution.x[0])
        largest_action_error=max(largest_action_error,difference_action)
        assert difference_action < 2e-3, (settings,output,solution.x)
        assert output[7] < 2e-3
    print(f"60 randomized QPs; maximum first-action error={largest_action_error:.6g}")
