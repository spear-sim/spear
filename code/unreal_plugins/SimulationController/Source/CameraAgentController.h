#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "AgentController.h"
#include "CameraSensor.h" // not sure why CameraSensor can't be forward declared but this causes incomplete type errors on macOS

class ACameraActor;
class ARecastNavMesh;
class UNavigationSystemV1;
class UWorld;

struct Box;

class CameraAgentController : public AgentController
{
public:

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

    ACameraActor* camera_actor_ = nullptr;
    ARecastNavMesh* nav_mesh_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_ = nullptr;

    std::map<std::string, std::vector<float>> action_;
};