#pragma once

#include "common_utils/RobotSimSettings.hpp"

#include "CoreMinimal.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "UObject/ConstructorHelpers.h"
#include "Runtime/Engine/Classes/Components/BoxComponent.h"
#include "Runtime/Engine/Classes/Components/SphereComponent.h"
#include "Runtime/Engine/Classes/Components/CapsuleComponent.h"
#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"
#include "Runtime/Engine/Classes/PhysicsEngine/ConstraintInstance.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Core/Public/Internationalization/Regex.h"

#include "PIPCamera.h"
#include "common_utils/UniqueValueMap.hpp"
#include "common_utils/Utils.hpp"
#include "RobotBlueprintLib.h"
#include "RobotApi.h"

#include "UrdfParser/UrdfGeometry.h"
#include "UrdfParser/UrdfParser.h"
#include "UrdfLink.h"

#include "ControlledMotionComponent/ControlledMotionComponentFactory.h"
#include "ControlledMotionComponent/ControlledMotionComponent.h"

#include "MeshGeneration/StaticMeshGenerator.h"
#include "PawnEvents.h"
#include "RobotBase.h"
#include "PhysXIncludes.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"

#include "TaskSpaceControl/InverseKinematicApi.h"
#include "UrdfBotPawn.generated.h"

UCLASS(Blueprintable, meta = (ShortTooltip = "URDF  Pawn"))
class ROBOTSIM_API AUrdfBotPawn : public APawn, public RobotBase
{
    GENERATED_BODY()

public:
    AUrdfBotPawn();
    ~AUrdfBotPawn();

    virtual void BeginPlay() override;
    virtual void Tick(float delta) override;
    virtual void EndPlay(const EEndPlayReason::Type endPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* myComp,
                           class AActor* other,
                           class UPrimitiveComponent* otherComp,
                           bool bSelfMoved,
                           FVector hitLocation,
                           FVector hitNormal,
                           FVector normalImpulse,
                           const FHitResult& hit) override;
    UFUNCTION(BlueprintCallable)
    void InitializeForBeginPlay();

    // TODO: Can this be const?
    PawnEvents* GetPawnEvents();
    TMap<FString, AUrdfLink*> GetLinkComponents() const;
    AUrdfLink* GetRootLinkComponent() const;
    common_utils::UniqueValueMap<std::string, APIPCamera*> GetCameras() const;
    TMap<FString, ControlledMotionComponent*>
    GetControlledMotionComponents() const;
    TMap<FString, ControlledMotionComponent*> GetBotMotionComponents() const;
    virtual USceneComponent* GetComponent(FString componentName);
    virtual AUrdfLink* GetLink(FString linkName);
    virtual APawn* GetPawn() override
    {
        return this;
    }
    virtual void GetComponentReferenceTransform(FString componentName,
                                                FVector& translation,
                                                FRotator& rotation) override;
    virtual bool PawnUsesNedCoords() override
    {
        return false;
    }
    virtual void TeleportToLocation(FVector position,
                                    FQuat orientation,
                                    bool teleport) override;

public:
    // add keyboard control to set urdfbot movement
    void setupInputBindings();
    UFUNCTION(BlueprintCallable)
    void onMoveForward(float Val);
    UFUNCTION(BlueprintCallable)
    void onMoveRight(float Val);
    void onGrabObject();
    // void onDropReleased();
    UFUNCTION()
    bool InitializeForCatchObject();
    UFUNCTION()
    void OnEndEffectorCollision(UPrimitiveComponent* HitComponent,
                                AActor* OtherActor,
                                UPrimitiveComponent* OtherComp,
                                FVector NormalImpulse,
                                const FHitResult& Hit);
    TMap<FString, TTuple<UrdfJointType, UPhysicsConstraintComponent*>>
    getConstraints();

public:
    RobotApi* getRobotApi() const;
    void setLinkForceAndTorque(FString componentName,
                               const FVector& force,
                               const FVector& torque);
    void setJointTorque(FString jointName, const FVector& torque);
    void setDrive(FString jointName, float stiffness, float damping);
    void setDriveTarget(FString jointName, float target);
    void setDriveVelocity(FString jointName, float target);

    // keyboard control stiffness and damping for target control
    // For detail explanation, see
    // https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Joints.html#drives
    void onStiffnessUp();
    void onStiffnessDown();

    void onBaseMove(float value);
    void onBaseRotate(float value);
    void onBrake(float value);

    void onTorsoUp();
    void onTorsoDown();
    void setEndEffector(FString endEffectorName);
    // robot arm control
    void onArmXUp();
    void onArmXDown();
    void onArmYUp();
    void onArmYDown();
    void onArmZUp();
    void onArmZDown();

    void onArmXRotationUp();
    void onArmXRotationDown();
    void onArmYRotationUp();
    void onArmYRotationDown();
    void onArmZRotationUp();
    void onArmZRotationDown();

    void onArmReset();
    void onTest();

    void onManipulator();

    void KeyBoardControl(FVector deltaPos, FVector deltaOri);
    FTransform getJointPose(const FString& jointName);
    FTransform getRelativePose(const FString& baseLinkName,
                               const FString& TargetlinkName);
    FTransform getRelativePoseByLink(AUrdfLink* baseLink,
                                     AUrdfLink* Targetlink);

    TMap<FString, float> GetTargetQPosByLinkName();
    TMap<FString, float> GetCurrentQPosByLinkName();
    void SetTargetQPosByLinkName(const TMap<FString, float>& map);
    void SetTargetQPos(const TMap<FString, float>& map);
    void FindArmChain(TArray<FString>* linkList, TArray<FString>* jointList);

private:
    // For limited translation joints, we need to move the link to the center of
    // the allowable motion before setting the constraint. This is due to a
    // limitation in the PhysX engine - offsets are symmetric from reference
    // poses of joints when initConstraint() is called. Returns a FTransform
    // that undoes the transform. This will be either a pure rotation (for
    // revolute joints) or a pure translation (for prismatic joints) The intent
    // is to call this function to move the component to the center, initialize
    // the joint, then apply the given vector to move the component back.
    FVector MoveChildLinkForLimitedXAxisMotion(
        AUrdfLink* parentLink,
        AUrdfLink* childLink,
        const UrdfJointSpecification& jointSpecification);
    void ConstructFromFile(FString fileName);
    AUrdfLink*
    CreateLinkFromSpecification(const UrdfLinkSpecification& linkSpecification);
    void AttachChildren(AUrdfLink* parentLink,
                        const UrdfLinkSpecification& parentLinkSpecification,
                        AUrdfLink* childLink,
                        const UrdfLinkSpecification& childLinkSpecification,
                        UrdfJointSpecification* jointSpecification);
    FConstraintInstance
    CreateConstraint(const UrdfJointSpecification& jointSpecification);
    void ResizeLink(AUrdfLink* link, UrdfGeometry* geometry);
    UrdfLinkSpecification*
    FindRootNodeSpecification(TMap<FString, UrdfLinkSpecification*> links);
    FConstraintInstance CreateDefaultFixedConstraintInstance();
    bool ConstraintNeedsControlledMotionComponent(
        const UrdfJointSpecification& spec);
    void DrawDebug();
    void MoveAllComponents(FVector translation, FRotator rotation);
    // change joint velocity by delta
    void updateVelocity(FString jointName, float delta);

private:
    UPROPERTY()
    TMap<FString, AUrdfLink*> components_; //所有机器人的link节点
    UPROPERTY()
    TMap<FString, bool> component_visited_as_constraint_parent_;
    UPROPERTY()
    AUrdfLink* root_component_;
    PawnEvents pawn_events_;
    UPROPERTY()
    UStaticMesh* baseBoxMesh_;
    UPROPERTY()
    UStaticMesh* baseCylinderMesh_;
    UPROPERTY()
    UStaticMesh* baseSphereMesh_;
    UPROPERTY()
    TMap<FString, UMaterialInterface*> materials_;
    UPROPERTY()
    TMap<FString, UStaticMesh*> user_static_meshes_;
    TMap<FString, TArray<FRegexPattern>> collision_blacklist_;
    TMap<FString, TTuple<UrdfJointType, UPhysicsConstraintComponent*>>
        constraints_; //所有joint节点， 包括可控制的和fixed的类型。
    TMap<FString, ControlledMotionComponent*>
        controlled_motion_components_; // 所有关于可控制joint节点
    TMap<FString, ControlledMotionComponent*>
        movement_components_; // 所有关于移动的joint节点
    StaticMeshGenerator staticMeshGenerator_;
    InverseKinematicComponent* inverseKinematicComponent =
        new InverseKinematicComponent();
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras_;
    typedef RobotSim::RobotSimSettings RobotSimSettings;
    bool draw_debug_ = false;
    bool mIsClawGrabbed;
    float world_scale_;
    float debug_symbol_scale_ = 0.0f;
    float scale_factor_ = 1.0f;

public:
    UPROPERTY()
    class ASimModeBase* simmode_;

private:
    UPROPERTY()
    class UPhysicsHandleComponent* mPhysicsHandle;
    UPrimitiveComponent* mOtherHitComponent;
    FVector mHitPoint;
    FString mEndEffectorName;
    AUrdfLink* mEndEffectorLink;
    Motor* mLeftWheelJoint;
    Motor* mRightWheelJoint;
    TArray<ControlledMotionComponent*> mManipulatorJointList;
};
