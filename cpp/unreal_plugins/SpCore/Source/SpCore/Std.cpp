//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/Std.h"

#include <string>
#include <vector>

#include "SpCore/Boost.h"

bool Std::contains(const std::string& string, const std::string& substring)
{
    return string.find(substring) != std::string::npos;
}

std::vector<std::string> Std::tokenize(const std::string& string, const std::string& separators)
{
    boost::tokenizer<boost::char_separator<char>> tokenizer(string, boost::char_separator<char>(separators.c_str()));
    return std::vector<std::string>(tokenizer.begin(), tokenizer.end());
}

std::string Std::toLower(const std::string& string)
{
    return boost::algorithm::to_lower_copy(string);
}
