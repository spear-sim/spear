#pragma once

#include <map>
#include <string>
#include <vector>

class SIMULATIONCONTROLLER_API AgentController
{
public:
    AgentController() = default;
    virtual ~AgentController() = default;

    virtual void applyAction(const std::map<std::string, std::vector<float>>& action) = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getObservation() = 0;
    virtual std::map<std::string, struct Box> getObservationSpace() = 0;
    virtual std::map<std::string, struct Box> getActionSpace() = 0;

protected:
    /**
     * Template function to fill @param dest with @param src.
     * This function clears any existing data. Potentially dangerous if T is not a primitive datatype.
     */
    template <typename T>
    void serializeToUint8(const std::vector<T>& src, std::vector<uint8_t>& dest)
    {
        // clear dest before serializing
        dest.clear();

        // copy src to dest
        const uint8_t* begin_itr = reinterpret_cast<const uint8_t*>(&src.at(0));
        size_t size = src.size() * (sizeof(T) / sizeof(uint8_t));
        dest.assign(begin_itr, begin_itr + size);
    }
};
