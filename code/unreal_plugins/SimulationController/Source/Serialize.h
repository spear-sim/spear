#pragma once

#include <vector>

class Serialize
{
public:
    // Function returns serialized uint8_t vector by copying data from input src vector.
    template <typename T>
    static std::vector<uint8_t> toUint8(const std::vector<T>& src)
    {
        std::vector<uint8_t> dest;

        // copy src to dest
        const uint8_t* begin_itr = reinterpret_cast<const uint8_t*>(&src.at(0));
        size_t size = src.size() * sizeof(T);
        dest.assign(begin_itr, begin_itr + size);

        return dest;
    }
};
