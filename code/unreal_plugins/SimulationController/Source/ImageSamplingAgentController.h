    #pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

class AActor;
class ARecastNavMesh;
class CameraSensor;
class UWorld;

struct Box;

class ImageSamplingAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    ImageSamplingAgentController(UWorld* world);
    ~ImageSamplingAgentController();
    
    void findObjectReferences(UWorld* world) override;
    void cleanUpObjectReferences() override;

    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    std::map<std::string, Box> getStepInfoSpace() const override;

    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;

    void reset() override;
    bool isReady() const override;
    
private:
    AActor* camera_actor_ = nullptr; 
    std::unique_ptr<CameraSensor> camera_sensor_ = nullptr;
    ARecastNavMesh* nav_mesh_ = nullptr;
    UWorld* world_ = nullptr;

    void rebuildNavSystem();
};
