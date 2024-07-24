#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os 
import shutil
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--third_party_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "third_party")))
    parser.add_argument("--unreal_engine_dir")
    parser.add_argument("--num_parallel_jobs", type=int, default=1)
    parser.add_argument("--cxx_compiler")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    assert os.path.exists(args.third_party_dir)
    third_party_dir = os.path.realpath(args.third_party_dir)

    #
    # define build variables
    #

    if args.verbose:
        verbose_makefile = "ON"
    else:
        verbose_makefile = "OFF"

    if sys.platform == "linux" and args.cxx_compiler is None:
        assert os.path.exists(args.unreal_engine_dir)
        unreal_engine_dir        = os.path.realpath(args.unreal_engine_dir)
        linux_clang_bin_dir      = os.path.join(unreal_engine_dir, "Engine", "Extras", "ThirdPartyNotUE", "SDKs", "HostLinux", "Linux_x64", "v21_clang-15.0.1-centos7", "x86_64-unknown-linux-gnu", "bin")
        linux_libcpp_lib_dir     = os.path.join(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "lib", "Unix", "x86_64-unknown-linux-gnu")
        linux_libcpp_include_dir = os.path.join(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "include", "c++", "v1")

    if sys.platform == "win32":
        platform_dir = "Win64"
        cxx_flags = "'/std:c++20 /EHsc'"
    elif sys.platform == "darwin":
        platform_dir = "Mac"
        cxx_flags = "'-std=c++20 -mmacosx-version-min=10.14'"
    elif sys.platform == "linux":
        platform_dir = "Linux"
        cxx_flags = "'-std=c++20 -stdlib=libc++'"
    else:
        assert False

    if args.cxx_compiler is None:
        if sys.platform == "win32":
            cxx_compiler = "cl"
        elif sys.platform == "darwin":
            cxx_compiler = "clang++"
        elif sys.platform == "linux":
            cxx_compiler = os.path.join(linux_clang_bin_dir, "clang++")
            cxx_flags = f"'-std=c++20 -stdlib=libc++ -nostdinc++ -I{linux_libcpp_include_dir} -L{linux_libcpp_lib_dir}'"
        else:
            assert False
    else:
        cxx_compiler = args.cxx_compiler

    #
    # Boost
    #

    spear.log("Building Boost...")

    boost_dir   = os.path.realpath(os.path.join(third_party_dir, "boost"))
    include_dir = os.path.realpath(os.path.join(third_party_dir, "boost", "boost"))

    if os.path.isdir(include_dir):
        spear.log(f"Directory exists, removing: {include_dir}")
        shutil.rmtree(include_dir, ignore_errors=True)

    spear.log(f"Changing directory to working: {boost_dir}")
    os.chdir(boost_dir)

    if sys.platform == "win32":

        cmd = ["bootstrap.bat"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["b2", "headers"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    elif sys.platform in ["darwin", "linux"]:

        cmd = ["./bootstrap.sh"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["./b2", "headers"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    else:
        assert False

    #
    # rpclib
    #

    spear.log("Building rpclib...")

    build_dir = os.path.realpath(os.path.join(third_party_dir, "rpclib", "BUILD", platform_dir))

    if os.path.isdir(build_dir):
        spear.log(f"Directory exists, removing: {build_dir}")
        shutil.rmtree(build_dir, ignore_errors=True)

    spear.log(f"Creating directory and changing to working: {build_dir}")
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)

    if sys.platform == "win32":

        cmd = [
            "cmake",
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_CXX_FLAGS=" + cxx_flags,
            "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_makefile,
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "darwin":

        cmd = [
            "cmake",
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_CXX_FLAGS=" + cxx_flags,
            "-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
            "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_makefile,
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "linux":
        cmd = [
            "cmake",
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_CXX_FLAGS=" + cxx_flags,
            "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_makefile,
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    else:
        assert False

    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log("Built rpclib successfully.")

    #
    # yaml-cpp
    #

    spear.log("Building yaml-cpp...")
    build_dir = os.path.realpath(os.path.join(third_party_dir, "yaml-cpp", "BUILD", platform_dir))

    if os.path.isdir(build_dir):
        spear.log("Directory exists, removing: " + build_dir)
        shutil.rmtree(build_dir, ignore_errors=True)

    spear.log("Creating directory and changing to working: " + build_dir)
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)

    if sys.platform == "win32":

        cmd = [
            "cmake",
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_CXX_FLAGS=" + cxx_flags,
            "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_makefile,
            "-DYAML_CPP_BUILD_TESTS=OFF",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "darwin":

        cmd = [
            "cmake",
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_CXX_FLAGS=" + cxx_flags,
            "-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
            "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_makefile,
            "-DYAML_CPP_BUILD_TESTS=OFF",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "linux":
        cmd = [
            "cmake",
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_CXX_FLAGS=" + cxx_flags,
            "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_makefile,
            "-DYAML_CPP_BUILD_TESTS=OFF",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    else:
        assert False

    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log("Built yaml-cpp successfully.")

    spear.log("Done.")
