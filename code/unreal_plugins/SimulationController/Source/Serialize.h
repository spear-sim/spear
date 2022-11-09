#pragma once

#include <vector>

class Serialize
{
public:
    template <typename T>
    static std::vector<uint8_t> toUint8(const std::vector<T>& src)
    {
        // copy src to dest
        std::vector<uint8_t> dest;
        if (src.size() > 0) {
            const uint8_t* begin_itr = reinterpret_cast<const uint8_t*>(&src.at(0));
            size_t size = src.size() * sizeof(T);
            dest.assign(begin_itr, begin_itr + size);
        }
        return dest;
    }
};
