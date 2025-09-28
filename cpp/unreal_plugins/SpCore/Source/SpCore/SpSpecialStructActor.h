//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Transform.h>
#include <Math/Vector.h>

#include "SpCore/Log.h"

#include "SpSpecialStructActor.generated.h"

UCLASS()
class ASpSpecialStructActor : public AActor
{
    GENERATED_BODY()
public:
    ASpSpecialStructActor() { SP_LOG_CURRENT_FUNCTION(); }
    ~ASpSpecialStructActor() override { SP_LOG_CURRENT_FUNCTION(); }

private:

    // The UPROPERTIES below are required to support the Unreal::findSpecialStructByName(...) interface.

    UPROPERTY()
    FRotator FRotator_;

    UPROPERTY()
    FTransform FTransform_;

    UPROPERTY()
    FVector FVector_;
};
