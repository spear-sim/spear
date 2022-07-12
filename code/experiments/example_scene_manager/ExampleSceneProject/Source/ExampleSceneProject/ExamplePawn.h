#pragma once

#include "CoreMinimal.h"
#include "GameFramework/SpectatorPawn.h"
#include "Camera/PlayerCameraManager.h"

#include "DoorManager.h"

#include "ExamplePawn.generated.h"

UCLASS()
class EXAMPLESCENEPROJECT_API AExamplePawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    AExamplePawn();
    virtual void BeginPlay() override;
    // test only
	void test();
    //example showing how to open and close doors in InteriorSim scene
	void switchDoor();
    // changing between different rendering mode
	void switchRenderingMode();
    // load .pak and open new level from it. Note that InteriorSim scene is only available in RobotProject
	void switchScene();
    // change physical material for correct physcial property such as friction and density
	void switchPhysicalMaterial();
private:

    int rendering_mode_;

    bool door_stat_ = true;

};
