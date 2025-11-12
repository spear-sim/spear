#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--conda_script")
parser.add_argument("--cxx_compiler")
parser.add_argument("--unreal_engine_dir") # only required on Linux
parser.add_argument("--conda_env", default="spear-env")
args = parser.parse_args()


if __name__ == "__main__":

    #
    # define build variables
    #

    if sys.platform == "win32":

        assert args.cxx_compiler is None

        # optimize agressively for speed, enable exceptions with standard C++ stack unwinding and assume extern "C" code never throws, disable RTTI
        cxx_compiler = "cl"
        common_cxx_flags = "/std:c++20 /O2 /EHsc /GR-"
        cmake_cxx_flags = common_cxx_flags

        cmd_prefix = f"conda activate {args.conda_env} & "

    elif sys.platform == "darwin":

        if args.cxx_compiler is None:
            cxx_compiler = "clang++"
        else:
            cxx_compiler = args.cxx_compiler

        common_cxx_flags = f"-std=c++20 -O3 -stdlib=libc++ -mmacosx-version-min=10.14"
        cmake_cxx_flags = common_cxx_flags

        if args.conda_script:
            if os.path.exists(args.conda_script):
                spear.log("Found conda script at: ", args.conda_script)
                conda_script = args.conda_script
            assert conda_script is not None

        else:
            # see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script_candidates = [
                os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh")),  # anaconda shell install
                os.path.join(os.sep, "opt", "anaconda3", "etc", "profile.d", "conda.sh"),            # anaconda graphical install
                os.path.expanduser(os.path.join("~", "miniconda3", "etc", "profile.d", "conda.sh")), # miniconda shell install
                os.path.join(os.sep, "opt", "miniconda3", "etc", "profile.d", "conda.sh")]           # miniconda graphical install

            conda_script = None
            for conda_script_candidate in conda_script_candidates:
                if os.path.exists(conda_script_candidate):
                    spear.log("Found conda script at: ", conda_script_candidate)
                    conda_script = conda_script_candidate
                    break
            assert conda_script is not None

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    elif sys.platform == "linux":

        assert args.unreal_engine_dir is not None
        assert os.path.exists(args.unreal_engine_dir)
        assert args.cxx_compiler is None

        unreal_engine_dir = os.path.realpath(args.unreal_engine_dir)

        linux_clang_path_template = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Extras", "ThirdPartyNotUE", "SDKs", "HostLinux", "Linux_x64", "*clang*"))
        linux_clang_paths = glob.glob(linux_clang_path_template)
        assert len(linux_clang_paths) == 1
        linux_clang_path = linux_clang_paths[0]

        spear.log("Found Unreal clang: ", linux_clang_path)

        linux_clang_bin_dir      = os.path.realpath(os.path.join(linux_clang_path, "x86_64-unknown-linux-gnu", "bin"))
        linux_libcpp_include_dir = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "include", "c++", "v1"))
        linux_libcpp_lib_dir     = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "lib", "Unix", "x86_64-unknown-linux-gnu"))

        cxx_compiler = os.path.join(linux_clang_bin_dir, "clang++")

        common_cxx_flags = f"-std=c++20 -O3 -fexperimental-library -nostdinc++ -I\'{linux_libcpp_include_dir}\' -Wno-reserved-macro-identifier -stdlib=libc++ -L\'{linux_libcpp_lib_dir}\' -lc++"
        cmake_cxx_flags = f"{common_cxx_flags}"

        if args.conda_script:
            if os.path.exists(args.conda_script):
                spear.log("Found conda script at: ", args.conda_script)
                conda_script = args.conda_script
            assert conda_script is not None

        else:
            # see https://docs.anaconda.com/anaconda/user-guide/faq
            conda_script_candidates = [
                os.path.expanduser(os.path.join("~", "anaconda3", "etc", "profile.d", "conda.sh")),
                os.path.expanduser(os.path.join("~", "miniconda3", "etc", "profile.d", "conda.sh"))]

            conda_script = None
            for conda_script_candidate in conda_script_candidates:
                if os.path.exists(conda_script_candidate):
                    spear.log("Found conda script at: ", conda_script_candidate)
                    conda_script = conda_script_candidate
                    break
            assert conda_script is not None

        cmd_prefix = f". {conda_script}; conda activate {args.conda_env}; "

    else:
        assert False

    #
    # spear_ext
    #

    spear.log("Building and installing the spear_ext Python extension module...")

    # we need shell=True because we want to run in a specific anaconda env
    cmd = \
        cmd_prefix + \
        f'pip install -e "{os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "python_ext"))}" ' + \
        f'-C cmake.define.CMAKE_CXX_COMPILER="{cxx_compiler}" ' + \
        f'-C cmake.define.CMAKE_CXX_FLAGS="{cmake_cxx_flags}"'

    spear.log("Executing: ", cmd)
    subprocess.run(cmd, shell=True, check=True)

    spear.log("Successfully built and installed the spear_ext Python extension module.")
    spear.log("Done.")
