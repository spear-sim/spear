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
    parser.add_argument("--num_parallel_jobs", type=int, default=1)
    parser.add_argument("--c_compiler")
    parser.add_argument("--cxx_compiler")
    args = parser.parse_args()

    #
    # define build variables
    #

    third_party_dir = os.path.realpath(args.third_party_dir)

    if args.c_compiler is None:
        if sys.platform == "win32":
            c_compiler = "cl"
        elif sys.platform in ["darwin", "linux"]:
            c_compiler = "clang"
        else:
            assert False
    else:
        c_compiler = args.c_compiler

    if args.cxx_compiler is None:
        if sys.platform == "win32":
            cxx_compiler = "cl"
        elif sys.platform in ["darwin", "linux"]:
            cxx_compiler = "clang++"
        else:
            assert False
    else:
        cxx_compiler = args.cxx_compiler

    if sys.platform == "win32":
        platform_dir = "Win64"
    elif sys.platform == "darwin":
        platform_dir = "Mac"
    elif sys.platform == "linux":
        platform_dir = "Linux"

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

        cmd = [
            "bootstrap.bat"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "b2",
            "headers"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    elif sys.platform == "darwin":

        cmd = [
            "./bootstrap.sh"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "./b2",
            "headers"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    elif sys.platform == "linux":

        cmd = [
            "./bootstrap.sh"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "./b2",
            "headers"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    else:
        assert False

    #
    # Eigen
    #

    spear.log("Building Eigen...")

    build_dir = os.path.realpath(os.path.join(third_party_dir, "libeigen", "BUILD", platform_dir))

    if os.path.isdir(build_dir):
        spear.log(f"Directory exists, removing: {build_dir}")
        shutil.rmtree(build_dir, ignore_errors=True)

    spear.log(f"Creating directory and changing to working: {build_dir}")
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)

    cmd = [
        "cmake",
        "-DCMAKE_C_COMPILER=" + c_compiler,
        "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
        "-DCMAKE_INSTALL_PREFIX=" + build_dir,
        os.path.join("..", "..")]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    cmd = [
        "cmake",
        "--build",
        ".",
        "--target",
        "install"]

    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log("Built Eigen successfully.")

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
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "cmake",
            "--build",
            ".",
            "--config",
            "Release",
            "-j",
            f"{args.num_parallel_jobs}"]

    elif sys.platform == "darwin":

        cmd = [
            "cmake",
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_CXX_FLAGS='-mmacosx-version-min=10.14'",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "cmake",
            "--build",
            ".",
            "-j",
            f"{args.num_parallel_jobs}"]

    elif sys.platform == "linux":

        cmd = [
            "cmake",
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release" ,
            "-DCMAKE_CXX_FLAGS='-stdlib=libc++'",
            "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "cmake",
            "--build",
            ".",
            "-j",
            f"{args.num_parallel_jobs}"]

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
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "cmake",
            "--build",
            ".",
            "--config",
            "Release",
            "-j",
            "{0}".format(args.num_parallel_jobs)]

    elif sys.platform == "darwin":

        cmd = [
            "cmake",
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_CXX_FLAGS='-mmacosx-version-min=10.14'",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "cmake",
            "--build",
            ".",
            "-j",
            f"{args.num_parallel_jobs}"]

    elif sys.platform == "linux":

        cmd = [
            "cmake",
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release" ,
            "-DCMAKE_CXX_FLAGS='-stdlib=libc++'",
            "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = [
            "cmake",
            "--build",
            ".",
            "-j",
            f"{args.num_parallel_jobs}"]

    else:
        assert False

    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log("Built yaml-cpp successfully.")
    spear.log("Done.")
