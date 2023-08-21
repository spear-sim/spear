//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

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
    // We need to use TEXT() instead of *Unreal::toFString() because the * operator doesn't return a const pointer
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
    //
    // This function expects an input string in the format used by the BOOST_CURRENT_FUNCTION macro, which can vary depending on the compiler.
    //
    // On MSVC, the strings are in the following format, i.e., the format used by the BOOST_CURRENT_FUNCTION macro:
    //     __cdecl MyClass::MyClass(const class MyInputType1 &, const class MyInputType2 &, ...)
    //     MyReturnType __cdecl MyClass::myFunction<MyReturnType>(const class MyInputType1 &, const class MyInputType2 &, ...)
    //
    // On Clang, this function expects function strings in the following format:
    //     MyClass::MyClass(const MyInputType1 &, const MyInputType2 &, ...)
    //     virtual MyReturnType MyClass::myFunction()
    //
    // The most robust strategy for obtaining a sensible abbreviated function name seems to be the following: tokenize into coarse tokens, then
    // find a coarse token that contains "(", then tokenize the coarse token into fine tokens, then return the 0th fine token.
    //

    std::vector<std::string> coarse_tokens = Std::tokenize(current_function, " ");
    for (auto& coarse_token : coarse_tokens) {
        if (Std::containsSubstring(coarse_token, "(")) {
            std::vector<std::string> fine_tokens = Std::tokenize(coarse_token, " ()<>");
            SP_ASSERT(fine_tokens.size() > 0);
            return fine_tokens.at(0);
        }
    }

    SP_ASSERT(false);
    return "";
}
