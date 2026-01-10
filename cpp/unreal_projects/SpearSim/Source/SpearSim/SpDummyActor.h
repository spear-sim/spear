//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/Actor.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY

#include "SpDummyActor.generated.h"

// There needs to be at least one UCLASS in the SpearSim module, otherwise live recompilation in the editor
// doesn't behave as expected.

UCLASS()
class ASpDummyActor : public AActor
{
    GENERATED_BODY()
public: 
    ASpDummyActor();
    ~ASpDummyActor();
};
