#pragma once

#include <CoreMinimal.h>
#include <VehicleWheel.h>

#include "IgnoreCompilerWarnings.h"

#include "OpenBotWheel.generated.h"

// Need to wrap this entire class in BEGIN_IGNORE_COMPILER_WARNINGS/END_IGNORE_COMPILER_WARNINGS because it inherits from a deprecated class that interacts with Unreal's code generation functionality
BEGIN_IGNORE_COMPILER_WARNINGS
UCLASS()
class UOpenBotWheel : public UVehicleWheel
{
    GENERATED_BODY()
public:
    UOpenBotWheel();
};
END_IGNORE_COMPILER_WARNINGS
