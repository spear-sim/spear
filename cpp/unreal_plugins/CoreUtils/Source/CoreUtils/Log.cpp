//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "CoreUtils/Log.h"

#include <iostream>
#include <string>
#include <vector>

#include <CoreMinimal.h>

#include "CoreUtils/Unreal.h"
#include "CoreUtils/Std.h"

DECLARE_LOG_CATEGORY_EXTERN(LogSpear, Log, All);
DEFINE_LOG_CATEGORY(LogSpear);

void Log::logCurrentFunction(const std::filesystem::path& current_file, const std::string& current_function)
{
    log(current_file, getCurrentFunctionAbbreviated(current_function));
}

void Log::logStdout(const std::string& str)
{
    std::cout << str << std::endl;
}

void Log::logUnreal(const std::string& str)
{
    UE_LOG(LogSpear, Log, TEXT("%s"), *Unreal::toFString(str));
}

std::string Log::getPrefix(const std::filesystem::path& current_file)
{
    return "[SPEAR | " + getCurrentFileAbbreviated(current_file) + "] ";
}

std::string Log::getCurrentFileAbbreviated(const std::filesystem::path& current_file)
{
    return current_file.filename().string();
}

std::string Log::getCurrentFunctionAbbreviated(const std::string& current_function)
{
    // This function expects function strings in the following format, i.e., the format used by the BOOST_CURRENT_FUNCTION macro:
    //     __cdecl MyClass::MyClass(const class MyInputType1 &, const class MyInputType2 &, ...)
    //     MyReturnType __cdecl MyClass::myFunction<MyReturnType>(const class MyInputType1 &, const class MyInputType2 &, ...)
    // so we tokenize using " ()<>" as our string of separator tokens, and select the token after __cdecl as our abbreviated
    // function name.
    std::vector<std::string> tokens = Std::tokenize(current_function, " ()<>");
    int index = Std::index(tokens, "__cdecl");
    SP_ASSERT(index != -1);
    SP_ASSERT(tokens.size() > index+1);
    return tokens.at(index+1);
}
