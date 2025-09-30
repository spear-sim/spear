//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Math/Rotator.h>
#include <Math/Transform.h>
#include <Math/Vector.h>
#include <Math/Vector2D.h>
#include <UObject/Object.h> // UObject

#include "SpCore/Log.h"

#include "SpSpecialStructs.generated.h"

UCLASS()
class USpSpecialStructs : public UObject
{
    GENERATED_BODY()

// The UPROPERTIES below are required to support the Unreal::findSpecialStructByName(...) interface.
private:
    UPROPERTY()
    FRotator FRotator_;

    UPROPERTY()
    FTransform FTransform_;

    UPROPERTY()
    FVector FVector_;

    UPROPERTY()
    FVector2D FVector2D_;
};
