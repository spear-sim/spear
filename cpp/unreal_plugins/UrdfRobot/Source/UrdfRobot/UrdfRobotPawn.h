//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h> // FString
#include <GameFramework/Pawn.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "UrdfRobotPawn.generated.h"

class FObjectInitializer;
class UCameraComponent;

class USpStableNameComponent;
class UUrdfRobotComponent;

UCLASS()
class URDFROBOT_API AUrdfRobotPawn : public APawn
{
    GENERATED_BODY()
public:
    AUrdfRobotPawn(const FObjectInitializer& object_initializer);
    ~AUrdfRobotPawn();

    // This function recursively creates and configures the component hierarchy for an entire URDF robot. This
    // initialization needs to happen outside of the constructor because we need to be able to pass in additional
    // data. In particular, we need to pass in a URDF file in cases where our config system is not initialized.
    // With this use case in mind, we choose to expose this function as a UFUNCTION with no arguments for maximum
    // flexibility. A user can set UrdfFile interactively in the editor, and then click on the Initialize
    // button to call the function. Alternatively, a user can set UrdfFile in code and then call Initialize()
    // directly. If the config system is initialized, the URDF file specified in the config system will be loaded,
    // and setting UrdfFile will have no effect.
    UFUNCTION(CallInEditor, Category="SPEAR")
    void Initialize();

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="SP Stable Name Component")
    USpStableNameComponent* SpStableNameComponent = nullptr;

    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="URDF file used for initialization")
    FString UrdfFile;

    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="URDF Robot Component")
    UUrdfRobotComponent* UrdfRobotComponent = nullptr;

    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="Camera Component")
    UCameraComponent* CameraComponent = nullptr;
};
