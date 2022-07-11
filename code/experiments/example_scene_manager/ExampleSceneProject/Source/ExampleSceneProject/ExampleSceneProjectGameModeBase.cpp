#include "ExampleSceneProjectGameModeBase.h"

#include "ExamplePawn.h"

AExampleSceneProjectGameModeBase::AExampleSceneProjectGameModeBase(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    DefaultPawnClass = AExamplePawn::StaticClass();
}