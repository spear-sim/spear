#pragma once

#include "CoreMinimal.h"
#include "GameFramework/SpectatorPawn.h"
#include "Camera/PlayerCameraManager.h"

#include "VWDoorManager.h"

#include "ExamplePawn.generated.h"

UCLASS()
class EXAMPLESCENEPROJECT_API AExamplePawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    AExamplePawn();
    virtual void BeginPlay() override;

	void test();

	void switchDoor();

	void switchRenderingMode();
private:

    int rendering_mode_;

    bool door_stat_ = true;

};
