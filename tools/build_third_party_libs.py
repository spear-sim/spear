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
    parser.add_argument("--boost_toolset")
    parser.add_argument("--cxx_compiler")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    assert os.path.exists(args.third_party_dir)
    third_party_dir = os.path.realpath(args.third_party_dir)

    #
    # define build variables
    #

    if sys.platform == "win32":
        if args.boost_toolset is None:
            boost_toolset = "msvc"
        else:
            boost_toolset = args.boost_toolset

        if args.cxx_compiler is None:
            cxx_compiler = "cl"
        else:
            cxx_compiler = args.cxx_compiler

        platform_dir = "Win64"
        cxx_flags = "/std:c++20 /EHsc"

    elif sys.platform == "darwin":
        if args.boost_toolset is None:
            boost_toolset = "clang"
        else:
            boost_toolset = args.boost_toolset

        if args.cxx_compiler is None:
            cxx_compiler = "clang++"
        else:
            cxx_compiler = args.cxx_compiler

        platform_dir = "Mac"
        cxx_flags = f"-std=c++20 -stdlib=libc++ -mmacosx-version-min=10.14"

    elif sys.platform == "linux":
        assert os.path.exists(args.unreal_engine_dir)
        assert args.cxx_compiler is None
        assert args.boost_toolset is None

        unreal_engine_dir        = os.path.realpath(args.unreal_engine_dir)
        linux_clang_bin_dir      = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Extras", "ThirdPartyNotUE", "SDKs", "HostLinux", "Linux_x64", "v22_clang-16.0.6-centos7", "x86_64-unknown-linux-gnu", "bin"))
        linux_libcpp_include_dir = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "include", "c++", "v1"))

        boost_toolset = "clang"
        cxx_compiler = os.path.join(linux_clang_bin_dir, "clang++")

        platform_dir = "Linux"
        cxx_flags = f"-std=c++20 -stdlib=libc++ -nostdinc++ -I{linux_libcpp_include_dir}"

    else:
        assert False

    if args.verbose:
        boost_verbose_build_flag = "-d+4"
        cmake_verbose_makefile = "ON"

    else:
        boost_verbose_build_flag = ""
        cmake_verbose_makefile = "OFF"

    #
    # boost
    #

    spear.log("Building boost...")

    boost_dir = os.path.realpath(os.path.join(third_party_dir, "boost"))
    user_config_file = os.path.realpath(os.path.join(boost_dir, "user-config.jam"))

    # explicitly deep clean because "./b2 --clean-all" and "./b2 stage --clean" will leave files behind

    remove_dirs = [
        os.path.realpath(os.path.join(boost_dir, "bin.v2")),
        os.path.realpath(os.path.join(boost_dir, "boost")),
        os.path.realpath(os.path.join(boost_dir, "stage"))]

    remove_files = [
        os.path.realpath(os.path.join(boost_dir, "project-config.jam")),
        os.path.realpath(os.path.join(boost_dir, "user-config.jam"))]

    for d in remove_dirs:
        if os.path.exists(d):
            spear.log("Directory exists, removing: ", d)
            shutil.rmtree(d, ignore_errors=True)

    for f in remove_files:
        if os.path.exists(f):
            spear.log("File exists, removing: ", f)
            os.remove(f)

    # create a config file because there is does not appear to be any other way to specify a custom compiler path, see:
    #     https://www.boost.org/build/doc/html/bbv2/overview/configuration.html

    user_config_str = f"using {boost_toolset} : : {cxx_compiler} ;"
    spear.log(f"Creating boost config file: {user_config_file}")
    spear.log(f"    {user_config_str}")

    with open(user_config_file, "w") as f:
        f.write(user_config_str)

    # build

    spear.log(f"Changing directory to working: {boost_dir}")
    os.chdir(boost_dir)

    if sys.platform == "win32":

        cmd = ["bootstrap.bat"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = f'b2 --with-test toolset={boost_toolset} --user-config={user_config_file} link=static cxxflags="{cxx_flags}" {boost_verbose_build_flag}'
        spear.log(f"Executing: {cmd}")
        subprocess.run(cmd, shell=True, check=True) # need shell=True to handle cxxflags

    elif sys.platform == "darwin":

        cmd = ["./bootstrap.sh"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["./b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = f'./b2 --with-test toolset={boost_toolset} --user-config={user_config_file} link=static architecture=arm+x86 cxxflags="{cxx_flags}" {boost_verbose_build_flag}'
        spear.log(f"Executing: {cmd}")
        subprocess.run(cmd, shell=True, check=True) # need shell=True to handle cxxflags

    elif sys.platform == "linux":

        cmd = ["./bootstrap.sh"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["./b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = f'./b2 -a --with-test toolset={boost_toolset} --user-config={user_config_file} link=static cxxflags="{cxx_flags} -fPIC" {boost_verbose_build_flag}'
        spear.log(f"Executing: {cmd}")
        subprocess.run(cmd, shell=True, check=True) # need shell=True to handle cxxflags

    else:
        assert False

    spear.log("Built boost successfully.")

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
            f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
            f"-DCMAKE_CXX_FLAGS='{cxx_flags}'",
            f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "darwin":

        cmd = [
            "cmake",
            f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
            f"-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
            f"-DCMAKE_CXX_FLAGS='{cxx_flags}'",
            f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "linux":
        cmd = [
            "cmake",
            f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
            f"-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            f"-DCMAKE_CXX_FLAGS='{cxx_flags}'",
            f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
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
            f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
            f"-DCMAKE_CXX_FLAGS='{cxx_flags}'",
            f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
            "-DYAML_CPP_BUILD_TESTS=OFF",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "darwin":

        cmd = [
            "cmake",
            f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
            f"-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
            f"-DCMAKE_CXX_FLAGS='{cxx_flags}'",
            f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
            "-DYAML_CPP_BUILD_TESTS=OFF",
            os.path.join("..", "..")]

        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["cmake", "--build", ".", "--config", "Release", "-j", f"{args.num_parallel_jobs}"]

    elif sys.platform == "linux":
        cmd = [
            "cmake",
            f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
            f"-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            f"-DCMAKE_CXX_FLAGS='{cxx_flags}'",
            f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
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
