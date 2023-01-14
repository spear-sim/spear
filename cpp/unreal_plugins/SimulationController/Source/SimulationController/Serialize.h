//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

class Serialize
{
public:
    template <typename TSrc>
    static std::vector<uint8_t> toUint8(const std::vector<TSrc>& src)
    {
        // copy src to dest
        std::vector<uint8_t> dest;
        if (src.size() > 0) {
            auto data = reinterpret_cast<const uint8_t*>(&src.at(0));
            size_t size = src.size() * sizeof(TSrc);
            dest.assign(data, data + size);
        }
        return dest;
    }
};
