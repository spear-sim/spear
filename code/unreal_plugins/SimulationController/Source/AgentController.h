#pragma once

#include <map>
#include <string>
#include <vector>

struct Box;

class SIMULATIONCONTROLLER_API AgentController
{
public:

    AgentController() = default;
    virtual ~AgentController() = default;

    virtual void applyAction(const std::map<std::string, std::vector<float>>& action) = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getObservation() const = 0;
    virtual std::map<std::string, Box> getObservationSpace() const = 0;
    virtual std::map<std::string, Box> getActionSpace() const = 0;

protected:

    // Function returns serialized uint8_t vector by copying data from input src vector.
    template <typename T>
    static std::vector<uint8_t> serializeToUint8(const std::vector<T>& src)
    {
        std::vector<uint8_t> dest;

        // copy src to dest
        const uint8_t* begin_itr = reinterpret_cast<const uint8_t*>(&src.at(0));
        size_t size = src.size() * (sizeof(T) / sizeof(uint8_t));
        dest.assign(begin_itr, begin_itr + size);

        return dest;
    }
};
