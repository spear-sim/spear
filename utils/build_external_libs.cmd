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
if exist %ROOT_DIR%\..\code\thirdparty\rpclib\build rmdir /Q /S %ROOT_DIR%\..\code\thirdparty\rpclib\build
IF NOT EXIST %ROOT_DIR%\..\code\thirdparty\rpclib\build mkdir %ROOT_DIR%\..\code\thirdparty\rpclib\build
cd %ROOT_DIR%\..\code\thirdparty\rpclib\build
cmake -G"Visual Studio 16 2019" ..
cmake --build . --config Release


if ERRORLEVEL 1 goto :buildfailed

REM //---------- copy rpclib binaries and include folder to plugin folder ----------
ECHO Copying rpclib include and lib files to UnrealRL plugin 
set RPCLIB_TARGET_LIB=%ROOT_DIR%\..\code\unreal_plugins\UnrealRL\Source\ThirdParty\rpclib\lib
if exist %RPCLIB_TARGET_LIB% rmdir /Q /S %RPCLIB_TARGET_LIB%
if NOT exist %RPCLIB_TARGET_LIB% mkdir %RPCLIB_TARGET_LIB%

set RPCLIB_TARGET_INCLUDE=%ROOT_DIR%\..\code\unreal_plugins\UnrealRL\Source\ThirdParty\rpclib\include
if exist %RPCLIB_TARGET_INCLUDE% rmdir /Q /S %RPCLIB_TARGET_INCLUDE%
if NOT exist %RPCLIB_TARGET_INCLUDE% mkdir %RPCLIB_TARGET_INCLUDE%

robocopy /MIR ..\include %RPCLIB_TARGET_INCLUDE%
copy /y Release\rpc.lib %RPCLIB_TARGET_LIB%

cd %ROOT_DIR%

REM //---------- ASIO -------------
ECHO Copying asio files to UnrealRL plugin
set ASIO_TARGET_INCLUDE=%ROOT_DIR%\..\code\unreal_plugins\UnrealRL\Source\ThirdParty\asio
if exist %ASIO_TARGET_INCLUDE% rmdir /Q /S %ASIO_TARGET_INCLUDE%
if NOT exist %ASIO_TARGET_INCLUDE% mkdir %ASIO_TARGET_INCLUDE%

robocopy /MIR ..\code\thirdparty\asio\asio\include %ASIO_TARGET_INCLUDE% 

exit /b 0

:buildfailed_nomsg
chdir /d %ROOT_DIR% 

:buildfailed
echo(
echo #### Build failed - see messages above. 1>&2
