#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "DefaultGoalActor.generated.h"

UCLASS()
class ADefaultGoalActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADefaultGoalActor(const FObjectInitializer& ObjectInitializer);
	
	UPROPERTY()
	class USceneComponent* Scene;

protected:
	// Called when the game starts or when spawned
	// virtual void BeginPlay() override;

public:	
	// Called every frame
	// virtual void Tick(float DeltaTime) override;
	
};
