#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)

    if sys.platform == "win32":
        unreal_editor_python_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "ThirdParty", "Python3", "Win64", "python.exe"))
    elif sys.platform == "darwin":
        unreal_editor_python_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "ThirdParty", "Python3", "Mac", "bin", "python3"))
    elif sys.platform == "linux":
        unreal_editor_python_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "ThirdParty", "Python3", "Linux", "bin", "python3"))
    else:
        assert False

    python_package_dirs = [
        os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "third_party", "msgpack-rpc-python")),
        os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "python"))]

    for python_package_dir in python_package_dirs:
        # see https://dev.epicgames.com/community/learning/tutorials/lJly/python-install-modules-with-pip-unreal-engine-5-tutorial for more details
        # disable WARNING: you are using pip version X.X.X; however, version Y.Y.Y is available.
        # disable WARNING: the script X is installed in path 'path/to/UE_5.2/Engine/Binaries/ThirdParty/Python3/...', which is not on PATH.
        cmd = [unreal_editor_python_bin, "-m", "pip", "install", "--disable-pip-version-check", "--no-warn-script-location", "-e", python_package_dir]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    spear.log("Done.")
