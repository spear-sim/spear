#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

<<<<<<< HEAD:code/unreal_plugins/SimulationController/Source/CameraAgentController.h
#include "AgentController.h"
=======
#include "Agent.h"
>>>>>>> d0f7e077158b87b4e4e98a5ceed23f4c74dc8871:code/unreal_plugins/SimulationController/Source/CameraAgent.h
#include "CameraSensor.h" // not sure why CameraSensor can't be forward declared but this causes incomplete type errors on macOS

class ACameraActor;
class ARecastNavMesh;
class UNavigationSystemV1;
class UWorld;

struct Box;

class CameraAgent : public Agent
{
public:

<<<<<<< HEAD:code/unreal_plugins/SimulationController/Source/CameraAgentController.h
    CameraAgentController(UWorld* world);
    ~CameraAgentController();
=======
    CameraAgent(UWorld* world);
    ~CameraAgent();
>>>>>>> d0f7e077158b87b4e4e98a5ceed23f4c74dc8871:code/unreal_plugins/SimulationController/Source/CameraAgent.h
    
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

    void buildNavMesh();

    ACameraActor* camera_actor_ = nullptr;
<<<<<<< HEAD:code/unreal_plugins/SimulationController/Source/CameraAgentController.h
=======

    UNavigationSystemV1* nav_sys_ = nullptr;
>>>>>>> d0f7e077158b87b4e4e98a5ceed23f4c74dc8871:code/unreal_plugins/SimulationController/Source/CameraAgent.h
    ARecastNavMesh* nav_mesh_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_ = nullptr;

    std::map<std::string, std::vector<float>> action_;
<<<<<<< HEAD:code/unreal_plugins/SimulationController/Source/CameraAgentController.h
};
=======
};
>>>>>>> d0f7e077158b87b4e4e98a5ceed23f4c74dc8871:code/unreal_plugins/SimulationController/Source/CameraAgent.h
