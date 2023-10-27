//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS
#include <WheeledVehiclePawn.h>

#include "CoreUtils/ArrayDesc.h"

#include "VehiclePawn.generated.h"

class FObjectInitializer;
class UBoxComponent;
class UCameraComponent;
class UInputComponent;

class UInputActionComponent;
class UVehicleMovementComponent;

UCLASS()
class VEHICLE_API AVehiclePawn : public AWheeledVehiclePawn
{
    GENERATED_BODY()
public:
    AVehiclePawn(const FObjectInitializer& object_initializer);
    ~AVehiclePawn();

    // APawn interface
    void BeginPlay() override;

    UPROPERTY(EditAnywhere, DisplayName = "Vehicle Movement Component")
    UVehicleMovementComponent* MovementComponent = nullptr;
    UPROPERTY(EditAnywhere, DisplayName = "Camera Component")
    UCameraComponent* CameraComponent = nullptr;
    UPROPERTY(EditAnywhere, DisplayName = "IMU Component")
    UBoxComponent* ImuComponent = nullptr;

    // Used by VehicleAgent. Note that VehicleAgent must set action_components_ and observation_components_ before using this interface.
    std::map<std::string, ArrayDesc> getActionSpace() const;
    std::map<std::string, ArrayDesc> getObservationSpace() const;
    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action);
    std::map<std::string, std::vector<uint8_t>> getObservation() const;
    std::vector<std::string> action_components_;
    std::vector<std::string> observation_components_;

private:
    void applyAction(const std::map<std::string, std::vector<double>>& action);

    UInputActionComponent* input_action_component_ = nullptr;
};
