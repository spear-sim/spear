#pragma once

#include <map>
#include <string>
#include <vector>

class UWorld;

struct Box;

class Agent
{
public:

    // An Agent class must spawn new objects in the constructor if they are intended to be
    // findable by other classes. An Agent class must not attempt to find object references
    // in the constructor, because these objects might not be spawned yet. Use findObjectReferences(...)
    // instead.
    Agent() = default;
    virtual ~Agent() = default;

    virtual void findObjectReferences(UWorld* world) = 0;
    virtual void cleanUpObjectReferences() = 0;

    virtual std::map<std::string, Box> getActionSpace() const = 0;
    virtual std::map<std::string, Box> getObservationSpace() const = 0;
    virtual std::map<std::string, Box> getStepInfoSpace() const = 0;   
 
    virtual void applyAction(const std::map<std::string, std::vector<float>>& action) = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getObservation() const = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getStepInfo() const = 0;
    
    virtual void reset() = 0;
    virtual bool isReady() const = 0;
};
