@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

REM //---------- check VS version
if "%VisualStudioVersion%" == "" (
    echo(
    echo You need to run this command from x64 Native Tools Command Prompt for VS 2019.
    goto :buildfailed_nomsg
)
if "%VisualStudioVersion%" lss "16.0" (
    echo(
    echo Please install/upgrade your Visual Studio.
    echo Current version is %VisualStudioVersion%. Need greater than or equal to 16.0
    goto :buildfailed_nomsg
)

REM //---------- Build rpclib ------------
ECHO Starting cmake to build rpclib...
if exist %ROOT_DIR%\..\rpclib\build rmdir /Q /S %ROOT_DIR%\..\rpclib\build
IF NOT EXIST %ROOT_DIR%\..\rpclib\build mkdir %ROOT_DIR%\..\rpclib\build
cd %ROOT_DIR%\..\rpclib\build
cmake -G"Visual Studio 16 2019" ..
cmake --build . --config Release

if ERRORLEVEL 1 goto :buildfailed

REM //---------- Build yaml-cpp -------------
ECHO Starting cmake to build yaml-cpp
if exist %ROOT_DIR%\..\yaml-cpp\build rmdir /Q /S %ROOT_DIR%\..\yaml-cpp\build
IF NOT EXIST %ROOT_DIR%\..\yaml-cpp\build mkdir %ROOT_DIR%\..\yaml-cpp\build
cd %ROOT_DIR%\..\yaml-cpp\build
cmake -G"Visual Studio 16 2019" ..
cmake --build . --config Release

if ERRORLEVEL 1 goto :buildfailed

REM //---------- Build rbdl -------------
ECHO Starting cmake to build rbdl
if exist %ROOT_DIR%\..\rbdl\build rmdir /Q /S %ROOT_DIR%\..\rbdl\build
IF NOT EXIST %ROOT_DIR%\..\rbdl\build mkdir %ROOT_DIR%\..\rbdl\build
cd %ROOT_DIR%\..\rbdl\build
cmake -G"Visual Studio 16 2019" -DRBDL_BUILD_STATIC=ON -DRBDL_BUILD_ADDON_URDFREADER=ON ..
cmake --build . --config Release

if ERRORLEVEL 1 goto :buildfailed

exit /b 0

:buildfailed_nomsg
chdir /d %ROOT_DIR% 

:buildfailed
echo(
echo #### Build failed - see messages above. 1>&2
