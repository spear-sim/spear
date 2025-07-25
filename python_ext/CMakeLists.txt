#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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
    ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/include)

# Additional library paths
if(WIN32)
    link_directories(
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/BUILD/Win64/Release)
elseif(APPLE)
    link_directories(
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/BUILD/Mac)
elseif(UNIX)
    link_directories(
        ${CMAKE_CURRENT_LIST_DIR}/../third_party/rpclib/BUILD/Linux)
else()
    message(FATAL_ERROR "ERROR: Unexpected platform.")
endif()

# Additional libraries
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

    # Source code goes here
    cpp/spear_ext.cpp)

# Install directive for scikit-build-core
install(TARGETS spear_ext LIBRARY DESTINATION spear_ext)
