//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <GameFramework/Pawn.h>

#include "UrdfRobotPawn.generated.h"

class UCameraComponent;
class UInputComponent;

class UUrdfRobotComponent;

struct KeyboardActionDesc
{
    std::string axis_;
    std::map<std::string, std::vector<double>> action_;
};

UCLASS()
class URDFROBOT_API AUrdfRobotPawn : public APawn
{
    GENERATED_BODY()
public:
    AUrdfRobotPawn();
    ~AUrdfRobotPawn();

    // APawn interface
    void SetupPlayerInputComponent(UInputComponent* input_component) override;
    void Tick(float delta_time) override;

    // Debug interface. If the config system is initialized, the component hierarchy will be
    // initialized via the constructor, and there is no need to call this function. But this
    // this function is useful in situations where config system is not initialized, e.g.,
    // when running the Unreal Editor. For example, this function could be called via a UFUNCTION
    // (backed by a button in the editor) to spawn a AUrdfRobotPawn and then initialize it using
    // the URDF file provided as input. See DebugWidget for details.
    void initialize(const std::string& urdf_file);

    // use UPROPERTY to enable inspecting and editing in the Unreal Editor
    UPROPERTY(EditAnywhere, DisplayName = "URDF Robot Component")
    UUrdfRobotComponent* urdf_robot_component_ = nullptr;
    UCameraComponent* camera_component_ = nullptr;

private:
    std::vector<KeyboardActionDesc> keyboard_action_descs_;
};
