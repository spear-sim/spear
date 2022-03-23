#!/bin/bash
set -x
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

MIN_CMAKE_VERSION=3.5.1

function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

#check cmake version on system
if ! which cmake; then
    cmake_ver=0
else
    cmake_ver=$(cmake --version 2>&1 | head -n1 | cut -d ' ' -f3 | awk '{print $NF}')
fi

if version_less_than_equal_to $cmake_ver $MIN_CMAKE_VERSION; then
    echo "Required version of cmake is $MIN_CMAKE_VERSION. You have $cmake_ver. Please install the required cmake version and try again."
    exit 1
else
    echo "Already have good version of cmake: $cmake_ver"
fi

#rpclib
#compile lib
pushd ../rpclib >/dev/null

if [ -d build ]; then
    rm -rf build
fi

mkdir build

pushd ./build >/dev/null

if [ "$(uname)" == "Linux" ]
then
    export CC=clang
    export CXX=clang++
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_CXX_FLAGS="-stdlib=libc++" -DCMAKE_BUILD_TYPE=Release ..
else
    cmake -DCMAKE_BUILD_TYPE=Release ..
fi

cmake --build . --

popd >/dev/null # rpclib build
popd >/dev/null # rpclib

#yaml-cpp
#compile lib
pushd ../yaml-cpp > /dev/null

if [ -d build ]; then
    rm -rf build
fi

mkdir build

pushd ./build > /dev/null

if [ "$(uname)" == "Linux" ]
then
    export CC=clang
    export CXX=clang++
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_CXX_FLAGS="-stdlib=libc++" -DCMAKE_BUILD_TYPE=Release ..
else
    cmake -DCMAKE_BUILD_TYPE=Release ..
fi

cmake --build . --

popd >/dev/null # yaml-cpp build
popd >/dev/null # yaml-cpp

#rbdl
#compile lib
pushd ../rbdl > /dev/null

if [ -d build ]; then
    rm -rf build
fi

mkdir build

pushd ./build > /dev/null

if [ "$(uname)" == "Linux" ]
then
    export CC=clang
    export CXX=clang++
    cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_STATIC=ON -D RBDL_BUILD_ADDON_URDFREADER=ON -D CMAKE_CXX_COMPILER="clang++" -D CMAKE_CXX_FLAGS="-fPIC -stdlib=libc++" ..
else
    cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_STATIC=ON -D RBDL_BUILD_ADDON_URDFREADER=ON ..
fi

cmake --build . --

popd >/dev/null # rbdl build
popd >/dev/null # rbdl

popd >/dev/null # ${SCRIPT_DIR}

set +x
