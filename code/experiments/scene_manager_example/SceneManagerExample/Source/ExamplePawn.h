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
    // test only
    void test2();
    // change physical material for correct physical property such as friction and density
    void switchPhysicalMaterial();
    // create a new physical material with specific friction and density, then assign to all floor actors
    void switchNewPhysicalMaterial();

private:
    int physical_material_stat_ = 0;
};
