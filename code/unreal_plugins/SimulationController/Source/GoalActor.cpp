#include <GoalActor.h>

AGoalActor::AGoalActor(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer)
{
	PrimaryActorTick.bCanEverTick = false;
	
	SceneComponent = ObjectInitializer.CreateDefaultSubobject<USceneComponent>(this, TEXT("Scene"));
	SceneComponent->SetMobility(EComponentMobility::Movable);
	SetRootComponent(SceneComponent);
}
