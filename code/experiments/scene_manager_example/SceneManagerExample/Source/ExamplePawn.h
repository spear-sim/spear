#pragma once

#include <CoreMinimal.h>
#include <GameFramework/SpectatorPawn.h>
#include <Camera/PlayerCameraManager.h>


#include "ExamplePawn.generated.h"

UCLASS()
class SCENEMANAGEREXAMPLE_API AExamplePawn : public ASpectatorPawn
{
    GENERATED_BODY()
public:
    AExamplePawn();
    virtual void BeginPlay() override;
    // test only
	void test();
    // change physical material for correct physcial property such as friction and density
	void switchPhysicalMaterial();

private:
    int physical_material_stat_ = 0;
};
