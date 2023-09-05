//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "CoreUtils/PlayerInputComponent.h"

#include "UrdfJointComponent.generated.h"

class UUrdfLinkComponent;
class UUrdfJointPlayerInputComponent;
struct UrdfJointDesc;

// enum values must match UrdfJointType in UrdfParser.h
UENUM()
enum class EJointType
{
    Invalid    UMETA(DisplayName = "Invalid"),
    Revolute   UMETA(DisplayName = "Revolute"),
    Continuous UMETA(DisplayName = "Continuous"),
    Prismatic  UMETA(DisplayName = "Prismatic"),
    Fixed      UMETA(DisplayName = "Fixed"),
    Floating   UMETA(DisplayName = "Floating"),
    Planar     UMETA(DisplayName = "Planar")
};

// enum values must match UrdfJointType in UrdfParser.h
UENUM()
enum class EJointControlType
{
    NotActuated         UMETA(DisplayName = "Not Actuated"),
    Position            UMETA(DisplayName = "Position"),
    Velocity            UMETA(DisplayName = "Velocity"),
    PositionAndVelocity UMETA(DisplayName = "Position and Velocity"),
    Torque              UMETA(DisplayName = "Torque"),
};

// We need to use UCLASS(ClassGroup = "SPEAR", meta=(BlueprintSpawnableComponent)) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup = "SPEAR", meta=(BlueprintSpawnableComponent))
class URDFROBOT_API UUrdfJointComponent : public UPhysicsConstraintComponent
{
    GENERATED_BODY()
public:
    UUrdfJointComponent();
    ~UUrdfJointComponent();

    // UActorComponent interface
    void BeginPlay() override;

    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Joint Type")
    EJointType JointType = EJointType::Invalid;
    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Control Type")
    EJointControlType JointControlType = EJointControlType::NotActuated;
    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Enable Keyboard Control")
    bool EnableKeyboardControl = false;

    // TODO (MR): support linear translation offsets
    //UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Linear Translation Offset")
    //FVector LinearTranslationOffset = FVector::ZeroVector;

    // This function configures all of the joint's properties, including configuring the joint to operate on the input parent link and child link.
    void initialize(const UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link, UUrdfLinkComponent* child_link);

    // Apply a named action (e.g., "add_torque_in_radians") to the joint using a given vector of input data. Used by UUrdfRobotComponent.
    void applyAction(const std::string& action_component_name, const std::vector<double>& action_component_data, bool assert_if_action_is_inconsistent = true);

private:
    UStaticMeshComponent* parent_static_mesh_component_ = nullptr;
    UStaticMeshComponent* child_static_mesh_component_ = nullptr;
    UPlayerInputComponent* player_input_component_ = nullptr;
};
