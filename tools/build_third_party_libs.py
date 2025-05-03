#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
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
    parser.add_argument("--boost_toolset_version")
    parser.add_argument("--cxx_compiler")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    assert os.path.exists(args.third_party_dir)
    third_party_dir = os.path.realpath(args.third_party_dir)

    #
    # define build variables
    #

    if sys.platform == "win32":

        assert args.boost_toolset is None
        assert args.boost_toolset_version is None
        assert args.cxx_compiler is None

        boost_toolset = "msvc"
        boost_toolset_version = "14.3"
        cxx_compiler = "cl"

        # On Windows, we don't include cxx_compiler in our generated user-config.jam file to avoid the
        # following warning:
        #     warning: Did not find command for MSVC toolset. If you have Visual Studio 2017 installed you
        #     will need to specify the full path to the command, set VS150COMNTOOLS for your installation, or
        #     build from the 'Visual Studio Command Prompt for VS 2017'.
        boost_user_config_str = f"using {boost_toolset} : {boost_toolset_version} ;\n"

        platform_dir = "Win64"

        common_cxx_flags = "/std:c++20 /EHsc /GR-" # enable exceptions with standard C++ stack unwinding and assume extern "C" code never throws, disable RTTI
        boost_cxx_flags = common_cxx_flags
        cmake_cxx_flags = common_cxx_flags

    elif sys.platform == "darwin":

        if args.boost_toolset is None:
            boost_toolset = "clang"
        else:
            boost_toolset = args.boost_toolset

        if args.boost_toolset_version is None:
            boost_toolset_version = ""
        else:
            boost_toolset_version = args.boost_toolset_version

        if args.cxx_compiler is None:
            cxx_compiler = "clang++"
        else:
            cxx_compiler = args.cxx_compiler

        boost_user_config_str = f"using {boost_toolset} : {boost_toolset_version} : {cxx_compiler} ;\n"

        platform_dir = "Mac"

        common_cxx_flags = f"-std=c++20 -stdlib=libc++ -mmacosx-version-min=10.14"
        boost_cxx_flags = common_cxx_flags
        cmake_cxx_flags = common_cxx_flags

    elif sys.platform == "linux":

        assert args.unreal_engine_dir is not None
        assert os.path.exists(args.unreal_engine_dir)
        assert args.boost_toolset is None
        assert args.boost_toolset_version is None
        assert args.cxx_compiler is None

        unreal_engine_dir = os.path.realpath(args.unreal_engine_dir)

        linux_clang_path_template = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Extras", "ThirdPartyNotUE", "SDKs", "HostLinux", "Linux_x64", "*clang*"))
        linux_clang_paths = glob.glob(linux_clang_path_template)
        assert len(linux_clang_paths) == 1
        linux_clang_path = linux_clang_paths[0]

        spear.log(f"Found Unreal clang: {linux_clang_path}")

        linux_clang_bin_dir      = os.path.realpath(os.path.join(linux_clang_path, "x86_64-unknown-linux-gnu", "bin"))
        linux_libcpp_include_dir = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "include", "c++", "v1"))
        linux_libcpp_lib_dir     = os.path.realpath(os.path.join(unreal_engine_dir, "Engine", "Source", "ThirdParty", "Unix", "LibCxx", "lib", "Unix", "x86_64-unknown-linux-gnu"))

        boost_toolset = "clang"
        boost_toolset_version = ""
        cxx_compiler = os.path.join(linux_clang_bin_dir, "clang++")

        boost_user_config_str = f"using {boost_toolset} : {boost_toolset_version} : {cxx_compiler} ;\n"

        platform_dir = "Linux"

        common_cxx_flags = f"-nostdinc++ -I{linux_libcpp_include_dir} -Wno-reserved-macro-identifier"
        boost_cxx_flags = f"-std=c++03 {common_cxx_flags}" # need to compile Boost against C++03 or older to avoid "error: undefined symbol: __isoc23_sscanf" when building the SpearSim Unreal project on Linux
        cmake_cxx_flags = f"-std=c++20 {common_cxx_flags}"

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

    remove_files = []

    for d in remove_dirs:
        if os.path.exists(d):
            spear.log("Directory exists, removing: ", d)
            shutil.rmtree(d, ignore_errors=True)

    for f in remove_files:
        if os.path.exists(f):
            spear.log("File exists, removing: ", f)
            os.remove(f)

    # create a config file because there does not appear to be any other way to specify a custom compiler path, see:
    #     https://www.boost.org/build/doc/html/bbv2/overview/configuration.html

    spear.log(f"Creating boost config file: {user_config_file}")
    spear.log_no_prefix(boost_user_config_str)

    with open(user_config_file, "w") as f:
        f.write(boost_user_config_str)

    # build

    spear.log(f"Changing directory to working: {boost_dir}")
    os.chdir(boost_dir)

    if sys.platform == "win32":

        cmd = ["bootstrap.bat"] # --with-toolset not needed because bootstrap.bat doesn't build boost library code, it only builds the b2 build tool
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = f'b2 --with-test toolset={boost_toolset} --user-config={user_config_file} link=static cxxflags="{boost_cxx_flags}" {boost_verbose_build_flag}'
        spear.log(f"Executing: {cmd}")
        subprocess.run(cmd, shell=True, check=True) # need shell=True to handle cxxflags

    elif sys.platform == "darwin":

        cmd = ["./bootstrap.sh"] # --with-toolset not needed because bootstrap.sh doesn't build boost library code, it only builds the b2 build tool
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["./b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = f'./b2 --with-test toolset={boost_toolset} --user-config={user_config_file} link=static architecture=arm+x86 cxxflags="{boost_cxx_flags}" {boost_verbose_build_flag}'
        spear.log(f"Executing: {cmd}")
        subprocess.run(cmd, shell=True, check=True) # need shell=True to handle cxxflags

    elif sys.platform == "linux":

        cmd = ["./bootstrap.sh"] # --with-toolset not needed because bootstrap.sh doesn't build boost library code, it only builds the b2 build tool
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        cmd = ["./b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        # we don't include -fPIC in cxx_flags for consistency with cmake libraries below, where -fPIC is
        # added to the compilation process via the CMAKE_POSITION_INDEPENDENT_CODE variable
        cmd = f'./b2 -a --with-test toolset={boost_toolset} --user-config={user_config_file} link=static cxxflags="{boost_cxx_flags} -fPIC" {boost_verbose_build_flag}'
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
            f"-DCMAKE_CXX_FLAGS='{cmake_cxx_flags}'",
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
            f"-DCMAKE_CXX_FLAGS='{cmake_cxx_flags}'",
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
            f"-DCMAKE_CXX_FLAGS='{cmake_cxx_flags}'",
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
            f"-DCMAKE_CXX_FLAGS='{cmake_cxx_flags}'",
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
            f"-DCMAKE_CXX_FLAGS='{cmake_cxx_flags}'",
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
            f"-DCMAKE_CXX_FLAGS='{cmake_cxx_flags}'",
            f"-DCMAKE_EXE_LINKER_FLAGS='-stdlib=libc++ -L{linux_libcpp_lib_dir} -lpthread'", # -lpthread needed on some Linux environments
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
