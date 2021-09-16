#pragma once

#include "CoreMinimal.h"

class ROBOTSIM_API UrdfOrigin
{
public:
    FVector Origin = FVector::ZeroVector;
    FRotator RollPitchYaw = FRotator::ZeroRotator;
};