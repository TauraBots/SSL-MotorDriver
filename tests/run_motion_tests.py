"""Host C++ checks: python tests/run_motion_tests.py --cxx /path/to/g++"""
import argparse
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--cxx", default="g++")
args = parser.parse_args()
cases = {
    "test_acceleration_limiter": ["acceleration_limiter"],
    "test_yaw_kinematics": ["omni_kinematics"],
    "test_omni_motion": ["acceleration_limiter", "omni_kinematics"],
    "test_motion_pipeline": ["acceleration_limiter", "omni_kinematics", "app_c_api", "app", "motor", "encoder"],
    "test_pid_autotune": ["pid_autotune"],
}
with tempfile.TemporaryDirectory(prefix="ssl-motion-tests-") as build:
    for name, sources in cases.items():
        exe = Path(build) / (name + ".exe")
        command = [args.cxx, "-std=c++14", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-I" + str(root / "tests/host"), "-I" + str(root / "Core/Inc"),
                   str(root / "tests" / (name + ".cpp"))]
        command += [str(root / "Core/Src" / (s + ".cpp")) for s in sources]
        subprocess.run(command + ["-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(name + ": PASS", flush=True)
