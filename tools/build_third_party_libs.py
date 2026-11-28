#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--boost-toolset")
parser.add_argument("--boost-toolset-version")
parser.add_argument("--cxx-compiler")
parser.add_argument("--unreal-engine-dir") # only required on Linux
parser.add_argument("--skip-boost", action="store_true")
parser.add_argument("--skip-rpclib", action="store_true")
parser.add_argument("--skip-yaml-cpp", action="store_true")
parser.add_argument("--verbose", action="store_true")
parser.add_argument("--third-party-dir", default=spear.path.realpath(os.path.dirname(__file__), "..", "third_party"))
args = parser.parse_args()

assert os.path.exists(args.third_party_dir)


if __name__ == "__main__":

    third_party_dir = spear.path.realpath(args.third_party_dir)

    spear.tools.validate_build_environment()

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

        # optimize aggressively for speed, enable exceptions with standard C++ stack unwinding and assume extern "C" code never throws, disable RTTI
        common_cxx_flags = "/O2 /EHsc /GR-"
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

        common_cxx_flags = f"-O3 -stdlib=libc++ -mmacosx-version-min={spear.tools.macos_deployment_target}"
        boost_cxx_flags = common_cxx_flags
        cmake_cxx_flags = common_cxx_flags

    elif sys.platform == "linux":

        assert args.unreal_engine_dir is not None
        assert os.path.exists(args.unreal_engine_dir)
        assert args.boost_toolset is None
        assert args.boost_toolset_version is None
        assert args.cxx_compiler is None

        unreal_engine_dir = spear.path.realpath(args.unreal_engine_dir)
        unreal_linux_toolchain = spear.tools.get_unreal_linux_toolchain(unreal_engine_dir=unreal_engine_dir)
        linux_libcpp_include_dir = unreal_linux_toolchain["libcpp_include_dir"]

        boost_toolset = "clang"
        boost_toolset_version = ""
        cxx_compiler = unreal_linux_toolchain["cxx_compiler"]

        boost_user_config_str = f"using {boost_toolset} : {boost_toolset_version} : {cxx_compiler} ;\n"

        platform_dir = "Linux"

        common_cxx_flags = f'-O3 -nostdinc++ -I\'{linux_libcpp_include_dir}\' -Wno-reserved-macro-identifier'
        boost_cxx_flags = common_cxx_flags
        cmake_cxx_flags = common_cxx_flags

    else:
        assert False

    if args.verbose:
        boost_verbose_build_flags = ["-d+4"]
        cmake_verbose_makefile = "ON"

    else:
        boost_verbose_build_flags = []
        cmake_verbose_makefile = "OFF"

    #
    # boost
    #

    if not args.skip_boost:

        spear.log("Building boost...")

        boost_dir = spear.path.realpath(third_party_dir, "boost")
        user_config_file = spear.path.realpath(boost_dir, "user-config.jam")

        # explicitly deep clean because "./b2 --clean-all" and "./b2 stage --clean" will leave files behind

        remove_dirs = [
            spear.path.realpath(boost_dir, "bin.v2"),
            spear.path.realpath(boost_dir, "boost"),
            spear.path.realpath(boost_dir, "stage")]

        remove_files = [
            spear.path.realpath(boost_dir, "project-config.jam"),
            spear.path.realpath(boost_dir, "user-config.jam")]

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

        spear.log("Creating boost config file: ", user_config_file)
        spear.log_no_prefix(boost_user_config_str)

        with open(user_config_file, "w") as f:
            f.write(boost_user_config_str)

        # build

        spear.log("Changing directory to working: ", boost_dir)
        os.chdir(boost_dir)

        if sys.platform == "win32":

            cmd = ["bootstrap.bat"] # --with-toolset not needed because bootstrap.bat doesn't build boost library code, it only builds the b2 build tool
            spear.tools.run_cmd(cmd=cmd)

            cmd = ["b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
            spear.tools.run_cmd(cmd=cmd)

            cmd = ["b2", "--with-test", f"toolset={boost_toolset}", f"--user-config={user_config_file}", "link=static", f"cxxflags={boost_cxx_flags}"] + boost_verbose_build_flags
            spear.tools.run_cmd(cmd=cmd)

        elif sys.platform == "darwin":

            cmd = ["./bootstrap.sh"] # --with-toolset not needed because bootstrap.sh doesn't build boost library code, it only builds the b2 build tool
            spear.tools.run_cmd(cmd=cmd)

            cmd = ["./b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
            spear.tools.run_cmd(cmd=cmd)

            cmd = ["./b2", "--with-test", f"toolset={boost_toolset}", f"--user-config={user_config_file}", "link=static", "architecture=arm+x86", f"cxxflags={boost_cxx_flags}"] + boost_verbose_build_flags
            spear.tools.run_cmd(cmd=cmd)

        elif sys.platform == "linux":

            cmd = ["./bootstrap.sh"] # --with-toolset not needed because bootstrap.sh doesn't build boost library code, it only builds the b2 build tool
            spear.tools.run_cmd(cmd=cmd)

            cmd = ["./b2", "headers", f"toolset={boost_toolset}", f"--user-config={user_config_file}"]
            spear.tools.run_cmd(cmd=cmd)

            # we don't include -fPIC in cxx_flags for consistency with cmake libraries below, where -fPIC is
            # added to the compilation process via the CMAKE_POSITION_INDEPENDENT_CODE variable
            cmd = ["./b2", "-a", "--with-test", f"toolset={boost_toolset}", f"--user-config={user_config_file}", "link=static", f"cxxflags={boost_cxx_flags} -fPIC"] + boost_verbose_build_flags
            spear.tools.run_cmd(cmd=cmd)

        else:
            assert False

        spear.log("Successfully built boost.")

    #
    # rpclib
    #

    if not args.skip_rpclib:

        spear.log("Building rpclib...")

        build_dir = spear.path.realpath(third_party_dir, "rpclib", "BUILD", platform_dir)

        if os.path.isdir(build_dir):
            spear.log("Directory exists, removing: ", build_dir)
            shutil.rmtree(build_dir, ignore_errors=True)

        spear.log("Creating directory if it does not already exist and changing to working: ", build_dir)
        os.makedirs(build_dir, exist_ok=True)
        os.chdir(build_dir)

        if sys.platform == "win32":

            cmd = [
                "cmake",
                f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
                f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags}",
                f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
                os.path.join("..", "..")]

            spear.tools.run_cmd(cmd=cmd)

            cmd = ["cmake", "--build", ".", "--config", "Release"]

        elif sys.platform == "darwin":

            cmd = [
                "cmake",
                f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
                "-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
                f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags}",
                f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
                os.path.join("..", "..")]

            spear.tools.run_cmd(cmd=cmd)

            cmd = ["cmake", "--build", ".", "--config", "Release"]

        elif sys.platform == "linux":
            cmd = [
                "cmake",
                f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
                f"-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
                f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags}",
                f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
                os.path.join("..", "..")]

            spear.tools.run_cmd(cmd=cmd)

            cmd = ["cmake", "--build", ".", "--config", "Release"]

        else:
            assert False

        spear.tools.run_cmd(cmd=cmd)

        spear.log("Successfully built rpclib.")

    #
    # yaml-cpp
    #

    if not args.skip_yaml_cpp:

        spear.log("Building yaml-cpp...")
        build_dir = spear.path.realpath(third_party_dir, "yaml-cpp", "BUILD", platform_dir)

        if os.path.isdir(build_dir):
            spear.log("Directory exists, removing: ", build_dir)
            shutil.rmtree(build_dir, ignore_errors=True)

        spear.log("Creating directory if it does not already exist and changing to working: ", build_dir)
        os.makedirs(build_dir, exist_ok=True)
        os.chdir(build_dir)

        if sys.platform == "win32":

            cmd = [
                "cmake",
                f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
                f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags}",
                f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
                "-DYAML_CPP_BUILD_TESTS=OFF",
                "-DYAML_CPP_BUILD_TOOLS=OFF",
                "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
                os.path.join("..", "..")]

            spear.tools.run_cmd(cmd=cmd)

            cmd = ["cmake", "--build", ".", "--config", "Release"]

        elif sys.platform == "darwin":

            cmd = [
                "cmake",
                f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
                "-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
                f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags}",
                f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
                "-DYAML_CPP_BUILD_TESTS=OFF",
                "-DYAML_CPP_BUILD_TOOLS=OFF",
                "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
                os.path.join("..", "..")]

            spear.tools.run_cmd(cmd=cmd)

            cmd = ["cmake", "--build", ".", "--config", "Release"]

        elif sys.platform == "linux":

            cmd = [
                "cmake",
                f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
                f"-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
                f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags}",
                f"-DCMAKE_VERBOSE_MAKEFILE={cmake_verbose_makefile}",
                "-DYAML_CPP_BUILD_TESTS=OFF",
                "-DYAML_CPP_BUILD_TOOLS=OFF",
                "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
                os.path.join("..", "..")]

            spear.tools.run_cmd(cmd=cmd)

            cmd = ["cmake", "--build", ".", "--config", "Release"]

        else:
            assert False

        spear.tools.run_cmd(cmd=cmd)

        spear.log("Successfully built yaml-cpp.")

    spear.log("Done.")
