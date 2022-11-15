#pragma once

#include "IgnoreCompilerWarnings.h"

// When compiling on Windows, asio will include Windows headers, which will define
// macros that conflict with Unreal Engine code. So we wrap our asio include with
// PreWindowsApi.h and PostWindowsApi.h.
#ifdef _MSC_VER
#include <Windows/PreWindowsApi.h>
#endif

BEGIN_IGNORE_COMPILER_WARNINGS
#include <asio.hpp>
END_IGNORE_COMPILER_WARNINGS

#ifdef _MSC_VER
#include <Windows/PostWindowsApi.h>
#endif
