#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

class RobotBase
{
public:
    virtual ~RobotBase(){};

    virtual USceneComponent* GetComponent(FString componentName) = 0;
    virtual void GetComponentReferenceTransform(FString componentName,
                                                FVector& translation,
                                                FRotator& rotation) = 0;
    virtual APawn* GetPawn() = 0;
    virtual bool PawnUsesNedCoords() = 0;
    virtual void
    TeleportToLocation(FVector position, FQuat orientation, bool teleport)
    {
        if (teleport)
            this->GetPawn()->SetActorLocationAndRotation(
                position, orientation, false, nullptr,
                ETeleportType::TeleportPhysics);
        else
            this->GetPawn()->SetActorLocationAndRotation(position, orientation,
                                                         true);
    }
};