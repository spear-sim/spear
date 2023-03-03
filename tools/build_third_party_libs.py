#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os 
import shutil
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
    # Eigen
    #

    print("[SPEAR | build_third_party_libs.py] Building Eigen...")

    build_dir = os.path.realpath(os.path.join(third_party_dir, "libeigen", "BUILD", platform_dir))

    if os.path.isdir(build_dir):
        print(f"[SPEAR | build_third_party_libs.py] Directory exists, removing: {build_dir}")
        shutil.rmtree(build_dir, ignore_errors=True)

    print(f"[SPEAR | build_third_party_libs.py] Creating directory and changing to working: {build_dir}")
    os.makedirs(build_dir)
    os.chdir(build_dir)

    cmd = [
        "cmake",
        "-DCMAKE_C_COMPILER=" + c_compiler,
        "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
        "-DCMAKE_INSTALL_PREFIX=" + build_dir,
        os.path.join("..", "..")]
    print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "cmake",
        "--build",
        ".",
        "--target",
        "install"]

    print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    print("[SPEAR | build_third_party_libs.py] Built Eigen successfully.")
    print()

    #
    # rbdl
    #

    print("[SPEAR | build_third_party_libs.py] Building rbdl...")

    build_dir = os.path.realpath(os.path.join(third_party_dir, "rbdl", "BUILD", platform_dir))
    eigen3_include_dir = os.path.realpath(os.path.join(third_party_dir, "libeigen", "BUILD", platform_dir, "include", "eigen3"))

    if os.path.isdir(build_dir):
        print(f"[SPEAR | build_third_party_libs.py] Directory exists, removing: {build_dir}")
        shutil.rmtree(build_dir, ignore_errors=True)

    print(f"[SPEAR | build_third_party_libs.py] Creating directory and changing to working: {build_dir}")
    os.makedirs(build_dir)
    os.chdir(build_dir)

    if sys.platform == "win32":

        cmd = [
            "cmake",
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release" ,
            "-DRBDL_BUILD_STATIC=ON",
            "-DRBDL_BUILD_ADDON_URDFREADER=ON",
            "-DCMAKE_CXX_FLAGS='/bigobj /DNOMINMAX /EHs'",
            "-DEIGEN3_INCLUDE_DIR=" + eigen3_include_dir,
            os.path.join("..", "..")]

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

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
            "-DCMAKE_BUILD_TYPE=Release" ,
            "-DRBDL_BUILD_STATIC=ON",
            "-DRBDL_BUILD_ADDON_URDFREADER=ON",
            "-DEIGEN3_INCLUDE_DIR=" + eigen3_include_dir,
            os.path.join("..", "..")]

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

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
            "-DCMAKE_BUILD_TYPE=Release",
            "-DRBDL_BUILD_STATIC=ON",
            "-DRBDL_BUILD_ADDON_URDFREADER=ON",
            "-DCMAKE_CXX_FLAGS='-stdlib=libc++'",
            "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            "-DEIGEN3_INCLUDE_DIR=" + eigen3_include_dir,
            os.path.join("..", "..")]

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

        cmd = [
            "cmake",
            "--build",
            ".",
            "-j",
            f"{args.num_parallel_jobs}"]

    else:
        assert False
    
    print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    print("[SPEAR | build_third_party_libs.py] Built rbdl successfully.")
    print()

    #
    # rpclib
    #

    print("[SPEAR | build_third_party_libs.py] Building rpclib...")

    build_dir = os.path.realpath(os.path.join(third_party_dir, "rpclib", "BUILD", platform_dir))

    if os.path.isdir(build_dir):
        print(f"[SPEAR | build_third_party_libs.py] Directory exists, removing: {build_dir}")
        shutil.rmtree(build_dir, ignore_errors=True)

    print(f"[SPEAR | build_third_party_libs.py] Creating directory and changing to working: {build_dir}")
    os.makedirs(build_dir)
    os.chdir(build_dir)

    if sys.platform == "win32":

        cmd = [
            "cmake",
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release",
            os.path.join("..", "..")]

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

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

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

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

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

        cmd = [
            "cmake",
            "--build",
            ".",
            "-j",
            f"{args.num_parallel_jobs}"]

    else:
        assert False

    print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    print("[SPEAR | build_third_party_libs.py] Built rpclib successfully.")
    print()

    #
    # yamp-cpp
    #

    print("[SPEAR | build_third_party_libs.py] Building yaml-cpp...")
    build_dir = os.path.realpath(os.path.join(third_party_dir, "yaml-cpp", "BUILD", platform_dir))

    if os.path.isdir(build_dir):
        print("[SPEAR | build_third_party_libs.py] Directory exists, removing: " + build_dir)
        shutil.rmtree(build_dir, ignore_errors=True)

    print("[SPEAR | build_third_party_libs.py] Creating directory and changing to working: " + build_dir)
    os.makedirs(build_dir)
    os.chdir(build_dir)

    if sys.platform == "win32":

        cmd = [
            "cmake",
            "-DCMAKE_C_COMPILER=" + c_compiler,
            "-DCMAKE_CXX_COMPILER=" + cxx_compiler,
            "-DCMAKE_BUILD_TYPE=Release",
            os.path.join("..", "..")]

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

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

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

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

        print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

        cmd = [
            "cmake",
            "--build",
            ".",
            "-j",
            f"{args.num_parallel_jobs}"]

    else:
        assert False

    print(f"[SPEAR | build_third_party_libs.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    print("[SPEAR | build_third_party_libs.py] Built yaml-cpp successfully.")
    print()

    print("[SPEAR | build_third_party_libs.py] Done.")
