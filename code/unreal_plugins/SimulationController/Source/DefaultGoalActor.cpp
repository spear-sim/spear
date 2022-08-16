#include <DefaultGoalActor.h>

ADefaultGoalActor::ADefaultGoalActor(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer)
{
	PrimaryActorTick.bCanEverTick = false;
	
	Scene = ObjectInitializer.CreateDefaultSubobject<USceneComponent>(this, TEXT("Scene"));
	Scene->SetMobility(EComponentMobility::Movable);
	SetRootComponent(Scene);
}
