//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "Components/ActorComponent.h"

#include "SpearComponent.generated.h"

UCLASS()
class USpearComponent : public UActorComponent
{
	GENERATED_BODY();

public:
	USpearComponent(const FObjectInitializer& object_initializer);
	~USpearComponent();
};