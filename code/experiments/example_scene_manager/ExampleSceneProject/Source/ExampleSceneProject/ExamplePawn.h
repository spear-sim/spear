#pragma once

#include "CoreMinimal.h"
#include "GameFramework/SpectatorPawn.h"
#include "Camera/PlayerCameraManager.h"

#include "ExamplePawn.generated.h"

UCLASS()
class EXAMPLESCENEPROJECT_API AExamplePawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    AExamplePawn();
    virtual void BeginPlay() override;

	void test();
private:

    bool stat;
};
