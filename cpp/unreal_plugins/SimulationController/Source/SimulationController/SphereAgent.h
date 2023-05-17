//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h>
#include <Engine/EngineBaseTypes.h>
#include <Math/Rotator.h>

#include "CoreUtils/ArrayDesc.h"
#include "SimulationController/Agent.h"

class AActor;
class ACameraActor;
class AStaticMeshActor;
class UStaticMeshComponent;
class UWorld;

class CameraSensor;
class UTickEventComponent;

class SphereAgent : public Agent
{
public:
    SphereAgent(UWorld* world);
    ~SphereAgent();
 
    void findObjectReferences(UWorld* world) override;
    void cleanUpObjectReferences() override;

    std::map<std::string, ArrayDesc> getActionSpace() const override;
    std::map<std::string, ArrayDesc> getObservationSpace() const override;
    std::map<std::string, ArrayDesc> getStepInfoSpace() const override;

    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;

    void reset() override;
    bool isReady() const override;

    void postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick);

private:
    AStaticMeshActor* static_mesh_actor_ = nullptr;
    ACameraActor* camera_actor_ = nullptr;
    AActor* parent_actor_ = nullptr;

    UStaticMeshComponent* static_mesh_component_ = nullptr;

    FRotator rotation_ = FRotator::ZeroRotator;

    UTickEventComponent* tick_event_component_ = nullptr;
    FDelegateHandle tick_event_handle_;

    std::unique_ptr<CameraSensor> camera_sensor_;
};
