#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "DefaultGoalActor.generated.h"

UCLASS()
class AGoalActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGoalActor(const FObjectInitializer& ObjectInitializer);
	
	UPROPERTY()
	class USceneComponent* SceneComponent;
	
};
