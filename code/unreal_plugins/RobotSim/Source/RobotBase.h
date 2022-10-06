#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "common_utils/RobotSimSettings.hpp"
#include "common_utils/UniqueValueMap.hpp"
// TODO move camera and pawn event to here
class RobotBase {
public:
    /**
     * @brief Destroy the Robot Base object
     *
     */
    virtual ~RobotBase(){};

    /**
     * @brief
     *
     * @param componentName
     * @return USceneComponent*
     */
    virtual USceneComponent* GetComponent(FString componentName) = 0;

    /**
     * @brief Get the Component Reference Transform object
     *
     * @param componentName
     * @param translation
     * @param rotation
     */
    virtual void GetComponentReferenceTransform(FString componentName, FVector& translation, FRotator& rotation) = 0;

    /**
     * @brief Get the Pawn object
     *
     * @return APawn*
     */
    virtual APawn* GetPawn() = 0;

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    virtual bool PawnUsesNedCoords() = 0;

    /**
     * @brief
     *
     * @param position
     * @param orientation
     * @param teleport
     */
    virtual void TeleportToLocation(FVector position, FQuat orientation, bool teleport)
    {
        if (teleport) {
            this->GetPawn()->SetActorLocationAndRotation(position, orientation, false, nullptr, ETeleportType::TeleportPhysics);
        }

        else {
            this->GetPawn()->SetActorLocationAndRotation(position, orientation, true);
        }
    }
};
