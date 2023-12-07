//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <PhysicsEngine/PhysicsConstraintComponent.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UENUM, UMETA, UPROPERTY

#include "UrdfJointComponent.generated.h"

class UStaticMeshComponent;

class UInputActionComponent;
class UUrdfLinkComponent;
class UUrdfJointPlayerInputComponent;
struct ArrayDesc;
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

// enum values must match UrdfJointControlType in UrdfParser.h
UENUM()
enum class EJointControlType
{
    NotActuated         UMETA(DisplayName = "Not Actuated"),
    Position            UMETA(DisplayName = "Position"),
    Velocity            UMETA(DisplayName = "Velocity"),
    PositionAndVelocity UMETA(DisplayName = "Position and Velocity"),
    Torque              UMETA(DisplayName = "Torque"),
};

// enum values must match UrdfJointInterfaceType in UrdfParser.h

UENUM()
enum class EJointInterfaceType
{
    NoInterface UMETA(DisplayName = "No Interface"),
    Set         UMETA(DisplayName = "Set"),
    AddTo       UMETA(DisplayName = "Add To"),
};

// We need to use UCLASS(ClassGroup = "SPEAR", meta=(BlueprintSpawnableComponent)) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup = "SPEAR", meta=(BlueprintSpawnableComponent))
class URDFROBOT_API UUrdfJointComponent : public UPhysicsConstraintComponent
{
    GENERATED_BODY()
public:
    UUrdfJointComponent();
    ~UUrdfJointComponent();

    // UPhysicsConstraintComponent interface
    void BeginPlay() override;

    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Joint Type")
    EJointType JointType = EJointType::Invalid;
    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Control Type")
    EJointControlType JointControlType = EJointControlType::NotActuated;
    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Interface Type")
    EJointInterfaceType JointInterfaceType = EJointInterfaceType::NoInterface;
    UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Enable Keyboard Control")
    bool EnableKeyboardControl = false;
    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Child StaticMeshComponent")
    UStaticMeshComponent* ChildStaticMeshComponent;
    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Parent StaticMeshComponent")
    UStaticMeshComponent* ParentStaticMeshComponent;

    // TODO (MR): support linear translation offsets
    //UPROPERTY(EditAnywhere, Category = "SPEAR", DisplayName = "Linear Translation Offset")
    //FVector LinearTranslationOffset = FVector::ZeroVector;

    // This function configures all of the joint's properties, including configuring the joint to operate on the input parent link and child link.
    void initialize(const UrdfJointDesc* joint_desc, UUrdfLinkComponent* parent_link, UUrdfLinkComponent* child_link);

    // Used by UrdfRobotComponent.
    std::map<std::string, ArrayDesc> getActionSpace() const;
    std::map<std::string, ArrayDesc> getObservationSpace() const;
    void applyActionComponent(const std::pair<std::string, std::vector<double>>& action_component);
    std::map<std::string, std::vector<uint8_t>> getObservation() const;

private:
    UInputActionComponent* input_action_component_ = nullptr;
};
