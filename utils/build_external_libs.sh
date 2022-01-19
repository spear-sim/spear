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
# compile lib
pushd ../code/thirdparty/rpclib >/dev/null

if [ -d build ]; then
    rm -rf build
fi

mkdir build

pushd ./build >/dev/null

if [ "$(uname)" == "Linux" ]
then
    export CC=clang
    export CXX=clang++
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_CXX_FLAGS="-stdlib=libc++" ..
else
    cmake ..
fi

cmake --build . --

popd >/dev/null
popd >/dev/null

#remove existing content
if [ -d "../code/unreal_plugins/UnrealRL/Source/ThirdParty/rpclib/" ]; then
   rm -rf ../code/unreal_plugins/UnrealRL/Source/ThirdParty/rpclib/
fi

#make dir
mkdir -p ../code/unreal_plugins/UnrealRL/Source/ThirdParty/rpclib/include/
mkdir -p ../code/unreal_plugins/UnrealRL/Source/ThirdParty/rpclib/lib/

#copy contents
cp -r ../code/thirdparty/rpclib/include/* ../code/unreal_plugins/UnrealRL/Source/ThirdParty/rpclib/include/
cp ../code/thirdparty/rpclib/build/librpc.a ../code/unreal_plugins/UnrealRL/Source/ThirdParty/rpclib/lib/

#asio
#remove existing content
if [ -d "../code/unreal_plugins/UnrealRL/Source/ThirdParty/asio" ]; then
   rm -rf ../code/unreal_plugins/UnrealRL/Source/ThirdParty/asio
fi

#make dir
mkdir -p ../code/unreal_plugins/UnrealRL/Source/ThirdParty/asio/

#copy contents
cp -r ../code/thirdparty/asio/asio/include/* ../code/unreal_plugins/UnrealRL/Source/ThirdParty/asio/

popd >/dev/null

set +x
