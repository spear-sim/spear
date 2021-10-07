@echo off

REM //---------- apply clang-format to all required .h and .cpp files ----------
FOR /F "tokens=*" %%G IN ('dir /b /s ..\code\*.h ..\code\*.cpp ^| find /v /i "thirdparty"') DO (clang-format.exe -i -style=file --verbose %%G)
