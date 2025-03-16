//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS
#include <WheeledVehiclePawn.h>

#include "SpCore/Legacy/ArrayDesc.h" // TODO: remove

#include "VehiclePawn.generated.h"

class FObjectInitializer;
class UBoxComponent;
class UCameraComponent;

class USpStableNameComponent;
class USpUserInputComponent;
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

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="SP Stable Name Component")
    USpStableNameComponent* SpStableNameComponent = nullptr;

    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="SP User Input Component")
    USpUserInputComponent* SpUserInputComponent = nullptr;
    
    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="Camera Component")
    UCameraComponent* CameraComponent = nullptr;
    
    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="IMU Component")
    UBoxComponent* ImuComponent = nullptr;
    
    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="Vehicle Movement Component")
    UVehicleMovementComponent* MovementComponent = nullptr;

    // Used by VehicleAgent. Note that VehicleAgent must call setActionComponents(...) and setObservationComponents(...) before using the rest of this interface.
    void setActionComponents(const std::vector<std::string>& action_components);
    void setObservationComponents(const std::vector<std::string>& observation_components);
    std::map<std::string, ArrayDesc> getActionSpace() const;
    std::map<std::string, ArrayDesc> getObservationSpace() const;
    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action);
    std::map<std::string, std::vector<uint8_t>> getObservation() const;

private:
    void applyAction(const std::map<std::string, std::vector<double>>& action);

    std::vector<std::string> action_components_;
    std::vector<std::string> observation_components_;
};
