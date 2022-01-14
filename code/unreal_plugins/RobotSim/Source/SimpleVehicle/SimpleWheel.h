#pragma once

#include "CoreMinimal.h"
#include "VehicleWheel.h"
#include "SimpleWheel.generated.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS

UCLASS()
class USimpleWheel : public UVehicleWheel
{
    GENERATED_BODY()

public:
    USimpleWheel();
};

PRAGMA_ENABLE_DEPRECATION_WARNINGS
