#pragma once

#include "CoreMinimal.h"
#include "VehicleWheel.h"
#include "OpenBotWheel.generated.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UCLASS()
class UOpenBotWheel : public UVehicleWheel
{
    GENERATED_BODY()

public:
    UOpenBotWheel();
};

PRAGMA_ENABLE_DEPRECATION_WARNINGS
