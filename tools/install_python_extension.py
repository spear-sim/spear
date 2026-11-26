#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import spear
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--cxx-compiler")
parser.add_argument("--unreal-engine-dir") # only required on Linux
parser.add_argument("--debug", action="store_true")
parser.add_argument("--wheel", action="store_true")
parser.add_argument("--conda-env", default="spear-env")
args = parser.parse_args()


if __name__ == "__main__":

    spear.tools.validate_build_environment()

    #
    # define build variables
    #

    if sys.platform == "win32":

        assert args.cxx_compiler is None

        cxx_compiler = "cl"

        if args.debug:
            optimization_flags = "/Od /Zi /DDEBUG /D_DEBUG"
        else:
            optimization_flags = "/O2"

        common_cxx_flags = f"/std:c++20 {optimization_flags} /EHsc /GR-"
        cmake_cxx_flags = common_cxx_flags
        cmake_cxx_standard_libraries = ""
        cmake_exe_linker_flags = ""

        cmd_env_vars = {}

    elif sys.platform == "darwin":

        if args.cxx_compiler is None:
            cxx_compiler = "clang++"
        else:
            cxx_compiler = args.cxx_compiler

        if args.debug:
            optimization_flags = "-O0 -g -fno-omit-frame-pointer -DDEBUG"
        else:
            optimization_flags = "-O3"

        common_cxx_flags = f"-std=c++20 {optimization_flags} -stdlib=libc++ -mmacosx-version-min=11.0"
        cmake_cxx_flags = common_cxx_flags
        cmake_cxx_standard_libraries = ""
        cmake_exe_linker_flags = ""

        cmd_env_vars = {"MACOSX_DEPLOYMENT_TARGET": "11.0"}

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

        assert os.path.isdir(linux_libcpp_include_dir) and os.listdir(linux_libcpp_include_dir)
        assert os.path.isdir(linux_libcpp_lib_dir) and os.listdir(linux_libcpp_lib_dir)

        spear.log("Found Unreal libc++ include dir: ", linux_libcpp_include_dir)
        spear.log("Found Unreal libc++ lib dir:     ", linux_libcpp_lib_dir)

        cxx_compiler = os.path.join(linux_clang_bin_dir, "clang++")

        # Don't use -fexperimental-library here, because this will attempt to link against a libc++ library
        # that doesn't ship with UE. Instead, enable the _LIBCPP_ENABLE_EXPERIMENTAL macro directly, which is
        # sufficient to enable std::ranges, but won't attempt to link against the missing library. This
        # low-level build customization is not necessary when building our Unreal plugins, because the Unreal
        # Build Tool handles this automatically.
        if args.debug:
            optimization_flags = "-O0 -g -fno-omit-frame-pointer -DDEBUG"
        else:
            optimization_flags = "-O3"

        common_cxx_flags = f"-std=c++20 {optimization_flags} -D_LIBCPP_ENABLE_EXPERIMENTAL -nostdinc++ -I\'{linux_libcpp_include_dir}\' -Wno-reserved-macro-identifier -stdlib=libc++"
        cmake_cxx_flags = common_cxx_flags

        # UE's libc++ and libc++abi are static libraries, so they must appear after spear_ext's object files
        # on the link line. CMake appends CMAKE_CXX_STANDARD_LIBRARIES to the end of the link line, but it
        # inserts CMAKE_CXX_FLAGS before the object files, so we pass these libraries here rather than in
        # cmake_cxx_flags.
        cmake_cxx_standard_libraries = f"-L\'{linux_libcpp_lib_dir}\' -lc++ -lc++abi"

        # Before building spear_ext, CMake checks that the compiler works by building and linking a small test
        # executable. This test executable is built with cmake_cxx_flags above, which includes -stdlib=libc++,
        # so clang automatically adds -lc++ when linking it. But the test executable is not built with
        # cmake_cxx_standard_libraries, so the -L path to UE's libc++ is missing, and the link fails because
        # UE's libc++ isn't in clang's default search path. CMake does use CMAKE_EXE_LINKER_FLAGS when linking
        # the test executable, so we pass the -L path there too. This flag has no effect on spear_ext itself,
        # because spear_ext is a shared library rather than an executable.
        cmake_exe_linker_flags = f"-L\'{linux_libcpp_lib_dir}\'"

        cmd_env_vars = {}

    else:
        assert False

    #
    # spear_ext
    #

    python_ext_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "python_ext"))
    build_dir = os.path.realpath(os.path.join(python_ext_dir, "BUILD"))

    if args.wheel:
        spear.log("Building the spear_ext Python extension module...")
        cmd = ["pip", "wheel", python_ext_dir, "-w", build_dir]
    else:
        spear.log("Building and installing the spear_ext Python extension module...")
        cmd = ["pip", "install", "-e", python_ext_dir]

    cmd = cmd + [
        "-C", f"cmake.define.CMAKE_CXX_COMPILER={cxx_compiler}",
        "-C", f"cmake.define.CMAKE_CXX_FLAGS={cmake_cxx_flags}"]

    if cmake_cxx_standard_libraries:
        cmd = cmd + ["-C", f"cmake.define.CMAKE_CXX_STANDARD_LIBRARIES={cmake_cxx_standard_libraries}"]

    if cmake_exe_linker_flags:
        cmd = cmd + ["-C", f"cmake.define.CMAKE_EXE_LINKER_FLAGS={cmake_exe_linker_flags}"]

    if args.debug:
        cmd = cmd + ["-C", "cmake.define.CMAKE_BUILD_TYPE=Debug"]

    spear.tools.run_conda_cmd(conda_env=args.conda_env, cmd=cmd, env=os.environ | cmd_env_vars)

    # generate dSYM debug symbols on macOS
    if args.debug and sys.platform == "darwin":
        python_ext_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "python_ext"))
        so_files = glob.glob(os.path.join(python_ext_dir, "**", "*.so"), recursive=True)
        so_files = [ so_file for so_file in so_files if ".dSYM" not in so_file ] # skip DWARF files inside previously generated .dSYM bundles
        for so_file in so_files:
            cmd = ["dsymutil", so_file, "-o", f"{so_file}.dSYM"]
            spear.tools.run_cmd(cmd=cmd)

    if args.wheel:
        spear.log("Successfully built the spear_ext Python extension module.")
    else:
        spear.log("Successfully built and installed the spear_ext Python extension module.")

    spear.log("Done.")
