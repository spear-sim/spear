#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

cmake_minimum_required(VERSION 3.15...3.26)

project(spear_ext LANGUAGES CXX)

if (NOT SKBUILD)
    message(WARNING "\
    This CMake file is meant to be executed using 'scikit-build'. Running
    it directly will almost certainly not produce the desired result. If
    you are a user trying to install this package, please use the command
    below, which will install all necessary build dependencies, compile
    the package in an isolated environment, and then install it.
    =====================================================================
     $ pip install .
    =====================================================================
    If you are a software developer, and this is your own package, then
    it is usually much more efficient to install the build dependencies
    in your environment once and use the following command that avoids
    a costly creation of a new virtual environment at every compilation:
    =====================================================================
     $ pip install nanobind scikit-build-core[pyproject]
     $ pip install --no-build-isolation -ve .
    =====================================================================
    You may optionally add -Ceditable.rebuild=true to auto-rebuild when
    the package is imported. Otherwise, you need to re-run the above
    after editing C++ files.")
endif()

# Try to import all Python components potentially needed by nanobind
find_package(Python 3.11
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

# CMake variables
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Additional include paths
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../cpp/unreal_plugins/SpCore/Source
    ${CMAKE_CURRENT_LIST_DIR}/../cpp/unreal_plugins/SpServices/Source
    ${CMAKE_CURRENT_LIST_DIR}/../third_party/boost
    ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/include)

# Additional library paths
if(WIN32)
    link_directories(
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/boost/stage/lib
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/BUILD/Win64/Release)
elseif(APPLE)
    link_directories(
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/boost/stage/lib
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/BUILD/Mac)
elseif(UNIX)
    link_directories(
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/boost/stage/lib
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/BUILD/Linux)
else()
    message(FATAL_ERROR "ERROR: Unexpected platform.")
endif()

# Additional libraries. We need to link against unit_test_framework because SpCore/Boost.h uses
# boost::debug::under_debugger from <boost/test/debug.hpp>, even though we don't use Boost's unit testing
# functionality. See SpCore.Build.cs.
if(WIN32)
    link_libraries(libboost_unit_test_framework-vc143-mt-x64-1_90)
elseif(APPLE)
    link_libraries(boost_unit_test_framework)
elseif(UNIX)
    link_libraries(boost_unit_test_framework)
else()
    message(FATAL_ERROR "ERROR: Unexpected platform.")
endif()

link_libraries(rpc)

# We are now ready to compile the actual extension module
nanobind_add_module(
    # Name of the extension
    spear_ext

    # Target the stable ABI for Python 3.12+, which reduces
    # the number of binary wheels that must be built. This
    # does nothing on older Python versions
    STABLE_ABI

    # Build libnanobind statically and merge it into the
    # extension (which itself remains a shared library)
    #
    # If your project builds multiple extensions, you can
    # replace this flag by NB_SHARED to conserve space by
    # reusing a shared libnanobind across libraries
    NB_STATIC

    # Source code goes here. We also compile the SpCore source files that participate in SP_PLATFORM_PURE builds.
    cpp/spear_ext.cpp
    ../cpp/unreal_plugins/SpCore/Source/SpCore/Assert.cpp
    ../cpp/unreal_plugins/SpCore/Source/SpCore/Log.cpp
    ../cpp/unreal_plugins/SpCore/Source/SpCore/Platform.cpp)

# nanobind_add_module(...) sets CXX_VISIBILITY_PRESET to hidden, which prevents boost::stacktrace from resolving
# function names when printing the call stack for a failed SP_ASSERT, because boost::stacktrace resolves names via
# dladdr(...) on macOS and Linux, which only sees exported symbols. So we export our symbols in debug builds, which
# makes call stacks readable. We don't do this in optimized builds, because inlining and stripping cause dladdr(...)
# to report the nearest exported symbol instead of the actual function, which produces plausible but incorrect names.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set_target_properties(spear_ext PROPERTIES CXX_VISIBILITY_PRESET default)
endif()

# Define the _API macros used by the SPEAR plugin headers that we include. Inside Unreal, UnrealBuildTool
# defines these macros to export and import symbols across plugin modules. Here, we compile those headers
# directly into the extension, so we define the macros to be empty.
target_compile_definitions(spear_ext PRIVATE SPCORE_API=)

# Suppress MSVC's deprecation warnings for C runtime functions like freopen(...), which are used by SPEAR plugin
# headers and source files that we include. UnrealBuildTool defines this macro for every module on Windows, so we do
# the same here. This needs to be defined on the command line, rather than in a header, because it only takes effect
# if it's defined before the first C runtime header is included.
if(MSVC)
    target_compile_definitions(spear_ext PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Install directive for scikit-build-core
install(TARGETS spear_ext LIBRARY DESTINATION spear_ext)
