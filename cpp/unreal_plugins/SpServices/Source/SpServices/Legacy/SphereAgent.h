//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <memory> // std::unique_ptr
#include <string>
#include <vector>

#include <Math/Rotator.h>

#include "SpCore/ArrayDesc.h" // TODO: remove

#include "SpServices/Legacy/Agent.h"
#include "SpServices/Legacy/StandaloneComponent.h"
#include "SpServices/Legacy/TickComponent.h"

class ACameraActor;
class AStaticMeshActor;
class UStaticMeshComponent;
class UWorld;

class CameraSensor;

class SphereAgent : public Agent
{
public:
    SphereAgent() = delete;
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

private:
    AStaticMeshActor* static_mesh_actor_ = nullptr;
    ACameraActor* camera_actor_ = nullptr;

    UStaticMeshComponent* static_mesh_component_ = nullptr;
    std::unique_ptr<StandaloneComponent<UTickComponent>> tick_component_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_;

    FRotator rotation_ = FRotator::ZeroRotator;
};
