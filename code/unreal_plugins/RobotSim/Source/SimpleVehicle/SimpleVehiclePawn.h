#pragma once

#include "common_utils/RobotSimSettings.hpp"

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Engine/CollisionProfile.h"
#include "SimpleWheeledVehicleMovementComponent.h"

#include "PIPCamera.h"
#include "common_utils/UniqueValueMap.hpp"
#include "common_utils/Utils.hpp"
#include "RobotBlueprintLib.h"

#include "RobotBase.h"
#include "PhysXIncludes.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"

#include "SimpleVehiclePawn.generated.h"

UCLASS(Blueprintable, meta = (ShortTooltip = "Simple Vehicle Pawn"))
class ROBOTSIM_API ASimpleVehiclePawn : public APawn, public RobotBase
{
    GENERATED_BODY()

    /**  The main skeletal mesh associated with this Vehicle */
    UPROPERTY(Category = Vehicle,
              VisibleDefaultsOnly,
              BlueprintReadOnly,
              meta = (AllowPrivateAccess = "true"))
    class USkeletalMeshComponent* Mesh;

    /** vehicle simulation component */
    UPROPERTY(Category = Vehicle,
              VisibleDefaultsOnly,
              BlueprintReadOnly,
              meta = (AllowPrivateAccess = "true"))
    class UWheeledVehicleMovementComponent* VehicleMovement;

    /** Camera component that will be our viewpoint */
    UPROPERTY(Category = Camera,
              VisibleDefaultsOnly,
              BlueprintReadOnly,
              meta = (AllowPrivateAccess = "true"))
    class UCameraComponent* Camera;

public:
    ASimpleVehiclePawn(const FObjectInitializer& ObjectInitializer);
    ~ASimpleVehiclePawn();

    // Begin Pawn interface
    virtual void
    SetupPlayerInputComponent(UInputComponent* InputComponent) override;
    // End Pawn interface

    // Begin Actor interface
    // Called every frame
    virtual void Tick(float DeltaTime) override;

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

public:
    // End Actor interface
    // Begin RobotBase interface
    virtual USceneComponent* GetComponent(FString componentName) override;
    virtual void GetComponentReferenceTransform(FString componentName,
                                                FVector& translation,
                                                FRotator& rotation) override;
    virtual APawn* GetPawn() override
    {
        return this;
    }
    virtual bool PawnUsesNedCoords() override
    {
        return false;
    }
    virtual void TeleportToLocation(FVector position,
                                    FQuat orientation,
                                    bool teleport) override;
    // End RobotBase interface
    void MoveForward(float Val);

    void MoveRight(float Val);

public:
    /** Name of the MeshComponent. Use this name if you want to prevent creation
     * of the component (with ObjectInitializer.DoNotCreateDefaultSubobject). */
    static FName VehicleMeshComponentName;

    /** Name of the VehicleMovement. Use this name if you want to use a
     * different class (with ObjectInitializer.SetDefaultSubobjectClass). */
    static FName VehicleMovementComponentName;

    /** Returns Mesh subobject **/
    class USkeletalMeshComponent* GetMesh() const
    {
        return Mesh;
    }
    /** Returns VehicleMovement subobject **/
    class UWheeledVehicleMovementComponent* GetVehicleMovementComponent() const
    {
        return VehicleMovement;
    }
    /** Returns Camera subobject **/
    class UCameraComponent* GetCamera() const
    {
        return Camera;
    }
};