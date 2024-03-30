//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealClassRegistrar.h"

#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMeshActor.h>

#include "SpCore/EngineActor.h"

void UnrealClassRegistrar::initialize()
{
    registerActorClass<AStaticMeshActor>("AStaticMeshActor");
    registerComponentClass<UStaticMeshComponent>("UStaticMeshComponent");

    registerActorClass<AEngineActor>("AEngineActor");
}

void UnrealClassRegistrar::terminate() {}
