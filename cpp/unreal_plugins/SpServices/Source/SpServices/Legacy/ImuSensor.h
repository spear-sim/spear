//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <memory> // std::unique_ptr
#include <string>
#include <vector>

#include <Math/Vector.h>

#include "SpCore/Legacy/ArrayDesc.h" // TODO: remove

#include "SpServices/Legacy/StandaloneComponent.h"
#include "SpServices/Legacy/TickComponent.h"

class UPrimitiveComponent;

class ImuSensor 
{
public:
    ImuSensor() = delete;
    ImuSensor(UPrimitiveComponent* component);
    ~ImuSensor();

    // Used by Agents.
    std::map<std::string, ArrayDesc> getObservationSpace() const;
    std::map<std::string, std::vector<uint8_t>> getObservation() const;

    // Linear acceleration minus gravity (i.e., will report +980 cm/s^2 for a stationary body aligned with the world-frame origin) in the body frame in cm/s^2.
    FVector linear_acceleration_body_ = FVector::ZeroVector;

    // Angular velocity in the body frame in rad/s.
    FVector angular_velocity_body_ = FVector::ZeroVector;

private:
    UPrimitiveComponent* primitive_component_ = nullptr;
    std::unique_ptr<StandaloneComponent<UTickComponent>> tick_component_ = nullptr;

    FVector previous_linear_velocity_world_ = FVector::ZeroVector;
};
