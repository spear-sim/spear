#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "AgentController.h"
#include "CameraSensor.h"

class AActor;
class ARecastNavMesh;
class UNavigationSystemV1;
class UWorld;

struct Box;

class CameraAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    CameraAgentController(UWorld* world);
    ~CameraAgentController();
    
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

    void buildNavMesh(UNavigationSystemV1* nav_sys);

    std::map<std::string, std::vector<float>> action_;
    AActor* camera_actor_ = nullptr; 
    std::unique_ptr<CameraSensor> camera_sensor_ = nullptr;
    ARecastNavMesh* nav_mesh_ = nullptr;
    UWorld* world_ = nullptr;    
};
