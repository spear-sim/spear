//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "Components/ActorComponent.h"

#include "SpearComponent.generated.h"

class FObjectInitializer;

UCLASS(ClassGroup = (SPEAR), meta = (BlueprintSpawnableComponent))
class COREUTILS_API USpearComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	USpearComponent(const FObjectInitializer& object_initializer);
	~USpearComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SPEAR", DisplayName = "Parent Actor Label Name")
	FString parent_actor_label_name_;
};
