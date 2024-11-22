//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <BodySetupEnums.h>                // ECollisionTraceFlag
#include <ChaosSolverConfiguration.h>
#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Containers/UnrealString.h>       // FString
#include <Engine/EngineTypes.h>            // EEndPlayReason, FRigidBodyErrorCorrection
#include <GameFramework/Actor.h>
#include <GameFramework/WorldSettings.h>   // FBroadphaseSettings
#include <HAL/Platform.h>                  // int32
#include <Math/UnrealMathUtility.h>        // UE_SMALL_NUMBER
#include <UObject/ObjectMacros.h>          // GENERATED_BODY, UCLASS, UPROPERTY, USTRUCT
#include <PhysicsEngine/PhysicsSettings.h> // ESettingsDOF, ESettingsLockedAxis, FChaosPhysicsSettings, FPhysicalSurfaceName, FPhysicsPredictionSettings
#include <PhysicsSettingsEnums.h>          // EFrictionCombineMode

#include "SpInitializeWorldManager.generated.h"

USTRUCT()
struct FSpPhysicsSettings
{
    GENERATED_BODY()

private:

    //
    // Copied from Engine/Source/Runtime/Engine/Classes/PhysicsEngine/PhysicsSettings.h
    //

    // Settings for Networked Physics Prediction, experimental.
    UPROPERTY(EditAnywhere, Category=Replication, meta=(DisplayName="Physics Prediction (Experimental)"))
    FPhysicsPredictionSettings PhysicsPrediction;

    // Error correction data for replicating simulated physics (rigid bodies)
    UPROPERTY(EditAnywhere, Category=Replication)
    FRigidBodyErrorCorrection PhysicErrorCorrection;

    UPROPERTY()
    TEnumAsByte<ESettingsLockedAxis::Type> LockedAxis_DEPRECATED = ESettingsLockedAxis::None;

    // Useful for constraining all objects in the world, for example if you are making a 2D game using 3D environments.
    UPROPERTY(EditAnywhere, Category=Simulation)
    TEnumAsByte<ESettingsDOF::Type> DefaultDegreesOfFreedom = ESettingsDOF::Full3D;

    // If true, the internal physx face to UE face mapping will not be generated. This is a memory optimization available if you do not rely on face indices returned by scene queries.
    UPROPERTY(EditAnywhere, Category=Optimization)
    bool bSuppressFaceRemapTable = false;

    // If true, store extra information to allow FindCollisionUV to derive UV info from a line trace hit result, using the FindCollisionUV utility
    UPROPERTY(EditAnywhere, Category=Optimization, meta=(DisplayName="Support UV From Hit Results", ConfigRestartRequired=true))
    bool bSupportUVFromHitResults = false; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // If true, physx will not update unreal with any bodies that have moved during the simulation. This should only be used if you have no physx simulation or you are manually updating the unreal data via polling physx.
    UPROPERTY(EditAnywhere, Category=Optimization)
    bool bDisableActiveActors = false;

    // Whether to disable generating KS pairs, enabling this makes switching between dynamic and static slower for actors - but speeds up contact generation by early rejecting these pairs
    UPROPERTY(EditAnywhere, Category=Optimization)
    bool bDisableKinematicStaticPairs = false; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // Whether to disable generating KK pairs, enabling this speeds up contact generation, however it is required when using APEX destruction.
    UPROPERTY(EditAnywhere, Category=Optimization)
    bool bDisableKinematicKinematicPairs = false; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // If true CCD will be ignored. This is an optimization when CCD is never used which removes the need for physx to check it internally.
    UPROPERTY(EditAnywhere, Category=Simulation)
    bool bDisableCCD = false; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // Min Delta Time below which anim dynamics and rigidbody nodes will not simulate.
    UPROPERTY(EditAnywhere, Category=Framerate, meta=(ClampMin="0.0", UIMin="0.0", ClampMax="1.0", UIMax="1.0"))
    float AnimPhysicsMinDeltaTime = 0.0f;

    // Whether to simulate anim physics nodes in the tick where they're reset.
    UPROPERTY(EditAnywhere, Category=Simulation)
    bool bSimulateAnimPhysicsAfterReset = false;

    // Min Physics Delta Time; the simulation will not step if the delta time is below this value
    UPROPERTY(EditAnywhere, Category=Framerate, meta=(ClampMin="0.0", UIMin="0.0", ClampMax="0.0001", UIMax="0.0001"))
    float MinPhysicsDeltaTime = UE_SMALL_NUMBER;

    // Max Physics Delta Time to be clamped.
    UPROPERTY(EditAnywhere, Category=Framerate, meta=(ClampMin="0.0013", UIMin="0.0013", ClampMax="1.0", UIMax="1.0"))
    float MaxPhysicsDeltaTime = 1.0f / 30.0f;

    // Whether to substep the physics simulation. This feature is still experimental. Certain functionality might not work correctly
    UPROPERTY(EditAnywhere, Category=Framerate)
    bool bSubstepping = false; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // Whether to substep the async physics simulation. This feature is still experimental. Certain functionality might not work correctly
    UPROPERTY(EditAnywhere, Category=Framerate)
    bool bSubsteppingAsync = false;

    // Whether to tick physics simulation on an async thread. This feature is still experimental. Certain functionality might not work correctly
    UPROPERTY(EditAnywhere, Category=Framerate)
    bool bTickPhysicsAsync = false;

    // If using async, the time step size to tick at. This feature is still experimental. Certain functionality might not work correctly
    UPROPERTY(EditAnywhere, Category=Framerate, meta=(EditCondition="bTickPhysicsAsync"))
    float AsyncFixedTimeStepSize = 1.0f / 30.0f;

    // Max delta time (in seconds) for an individual simulation substep.
    UPROPERTY(EditAnywhere, Category=Framerate, meta=(ClampMin="0.0013", UIMin="0.0013", ClampMax="1.0", UIMax="1.0", EditCondition="bSubStepping"))
    float MaxSubstepDeltaTime = 1.0f / 60.f;

    // Max number of substeps for physics simulation.
    UPROPERTY(EditAnywhere, Category=Framerate, meta=(ClampMin="1", UIMin="1", ClampMax="16", UIMax="16", EditCondition="bSubstepping"))
    int32 MaxSubsteps = 6;

    // Physics delta time smoothing factor for sync scene.
    UPROPERTY(EditAnywhere, Category=Framerate, AdvancedDisplay, meta=(ClampMin="0.0", UIMin="0.0", ClampMax="1.0", UIMax="1.0"))
    float SyncSceneSmoothingFactor = 0.0f;

    // Physics delta time initial average.
    UPROPERTY(EditAnywhere, Category=Framerate, AdvancedDisplay, meta=(ClampMin="0.0013", UIMin="0.0013", ClampMax="1.0", UIMax="1.0"))
    float InitialAverageFrameRate = 1.0f / 60.0f;

    // The number of frames it takes to rebuild the PhysX scene query AABB tree. The bigger the number, the smaller fetchResults takes per frame, but the more the tree deteriorates until a new tree is built
    UPROPERTY(EditAnywhere, Category=Framerate, AdvancedDisplay, meta=(ClampMin="4", UIMin="4"))
    int PhysXTreeRebuildRate = 10;

    // PhysicalMaterial Surface Types
    UPROPERTY(EditAnywhere, Category=PhysicalSurfaces)
    TArray<FPhysicalSurfaceName> PhysicalSurfaces;

    // If we want to Enable MPB or not globally. This is then overridden by project settings if not enabled.
    UPROPERTY(EditAnywhere, Category=Broadphase)
    FBroadphaseSettings DefaultBroadphaseSettings;

    // Minimum velocity delta required on a collinding object for Chaos to send a hit event
    UPROPERTY(EditAnywhere, Category=ChaosPhysics)
    float MinDeltaVelocityForHitEvents = 0.0f;

    // Chaos physics engine settings
    UPROPERTY(EditAnywhere, Category=ChaosPhysics)
    FChaosPhysicsSettings ChaosSettings;

    //
    // Copied from Engine/Source/Runtime/PhysicsCore/Public/PhysicsSettingsCore.h
    //

    // Default gravity.
    UPROPERTY(EditAnywhere, Category=Constants)
    float DefaultGravityZ = -980.0f;

    // Default terminal velocity for Physics Volumes.
    UPROPERTY(EditAnywhere, Category=Constants)
    float DefaultTerminalVelocity = 4000.0;

    // Default fluid friction for Physics Volumes.
    UPROPERTY(EditAnywhere, Category=Constants)
    float DefaultFluidFriction = 0.3f;

    // Amount of memory to reserve for PhysX simulate(), this is per pxscene and will be rounded up to the next 16K boundary
    UPROPERTY(EditAnywhere, Category=Constants, meta=(ClampMin="0", UIMin="0"))
    int32 SimulateScratchMemorySize = 262144;

    // Threshold for ragdoll bodies above which they will be added to an aggregate before being added to the scene
    UPROPERTY(EditAnywhere, Category=Constants, meta=(ClampMin="1", UIMin="1", ClampMax="127", UIMax="127"))
    int32 RagdollAggregateThreshold = 4;

    // Triangles from triangle meshes (BSP) with an area less than or equal to this value will be removed from physics collision data. Set to less than 0 to disable.
    UPROPERTY(EditAnywhere, Category=Constants, AdvancedDisplay, meta=(ClampMin = "-1.0", UIMin = "-1.0", ClampMax = "10.0", UIMax = "10.0"))
    float TriangleMeshTriangleMinAreaThreshold = 5.0f;

    // If set to true, the scene will use enhanced determinism at the cost of a bit more resources. See eENABLE_ENHANCED_DETERMINISM to learn about the specifics
    UPROPERTY(EditAnywhere, Category=Simulation)
    bool bEnableEnhancedDeterminism = false;

    // Enables shape sharing between sync and async scene for static rigid actors
    UPROPERTY(EditAnywhere, Category=Simulation, AdvancedDisplay)
    bool bEnableShapeSharing = false;

    // Enables persistent contact manifolds. This will generate fewer contact points, but with more accuracy. Reduces stability of stacking, but can help energy conservation.
    UPROPERTY(EditAnywhere, Category=Simulation, AdvancedDisplay)
    bool bEnablePCM = true;

    // Enables stabilization of contacts for slow moving bodies. This will help improve the stability of stacking.
    UPROPERTY(EditAnywhere, Category=Simulation, AdvancedDisplay)
    bool bEnableStabilization = false;

    // Whether to warn when physics locks are used incorrectly. Turning this off is not recommended and should only be used by very advanced users.
    UPROPERTY(EditAnywhere, Category=Simulation, AdvancedDisplay)
    bool bWarnMissingLocks = true;

    // Can 2D physics be used (Box2D)?
    UPROPERTY(EditAnywhere, Category=Simulation)
    bool bEnable2DPhysics = false;

    // If true, static meshes will use per poly collision as complex collision by default. If false the default behavior is the same as UseSimpleAsComplex.
    UPROPERTY(config)
    bool bDefaultHasComplexCollision_DEPRECATED = true;
    
    // Minimum relative velocity required for an object to bounce. A typical value for simulation stability is about 0.2 * gravity
    UPROPERTY(EditAnywhere, Category=Simulation, meta=(ClampMin="0", UIMin="0"))
    float BounceThresholdVelocity = 200.0f;

    // Friction combine mode, controls how friction is computed for multiple materials.
    UPROPERTY(EditAnywhere, Category=Simulation)
    TEnumAsByte<EFrictionCombineMode::Type> FrictionCombineMode = EFrictionCombineMode::Type::Average; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // Restitution combine mode, controls how restitution is computed for multiple materials.
    UPROPERTY(EditAnywhere, Category=Simulation)
    TEnumAsByte<EFrictionCombineMode::Type> RestitutionCombineMode = EFrictionCombineMode::Type::Average; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // Max angular velocity that a simulated object can achieve.
    UPROPERTY(EditAnywhere, Category=Simulation)
    float MaxAngularVelocity = 3600.0f; // 10 revolutions per second

    // Max velocity which may be used to depenetrate simulated physics objects. 0 means no maximum.
    UPROPERTY(EditAnywhere, Category=Simulation)
    float MaxDepenetrationVelocity = 0.0f; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // Contact offset multiplier. When creating a physics shape we look at its bounding volume and multiply its minimum value by this multiplier. A bigger number will generate contact points earlier which results in higher stability at the cost of performance.
    UPROPERTY(EditAnywhere, Category=Simulation, meta=(ClampMin="0.001", UIMin="0.001"))
    float ContactOffsetMultiplier = 0.02f;

    // Min Contact offset.
    UPROPERTY(EditAnywhere, Category=Simulation, meta=(ClampMin="0.0001", UIMin="0.0001"))
    float MinContactOffset = 2.0f;

    // Max Contact offset.
    UPROPERTY(EditAnywhere, Category=Simulation, meta=(ClampMin="0.001", UIMin="0.001"))
    float MaxContactOffset = 8.0f;

    // If true, simulate physics for this component on a dedicated server. This should be set if simulating physics and replicating with a dedicated server.
    UPROPERTY(EditAnywhere, Category=Simulation)
    bool bSimulateSkeletalMeshOnDedicatedServer = true;

    // Determines the default physics shape complexity.
    UPROPERTY(EditAnywhere, Category=Simulation)
    TEnumAsByte<ECollisionTraceFlag> DefaultShapeComplexity = CTF_UseSimpleAndComplex; // no initialization guidance but matches the UPhysicsSettings default object with default project settings

    // Options to apply to Chaos solvers on creation
    UPROPERTY(EditAnywhere, Category=ChaosPhysics)
    FChaosSolverConfiguration SolverOptions;

    //
    // No public properties to copy from UDeveloperSettings in Engine/Source/Runtime/DeveloperSettings/Public/Engine/DeveloperSettings.h
    //
};

UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpInitializeWorldManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpInitializeWorldManager();
    ~ASpInitializeWorldManager();

    // AActor interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;
    void Tick(float delta_time) override;

private:
    // Override physics settings
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bOverridePhysicsSettings = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    FSpPhysicsSettings SpPhysicsSettings;

    // Override fixed delta time
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bOverrideFixedDeltaTime = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    double FixedDeltaTime = 1.0/30.0; // Engine/Source/Runtime/Core/Private/Misc/App.cpp

    // Override game paused
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bOverrideGamePaused = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool GamePaused = false;

    // Force skylight update
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bForceSkylightUpdate = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    float ForceSkylightUpdateMaxDurationSeconds = 0.5f;

    // Initialize config system
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bInitializeConfigSystem = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    FString ConfigFile;

    // Execute console commands
    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bExecuteConsoleCommands = false;
    UPROPERTY(EditAnywhere, Category="SPEAR")
    TArray<FString> ConsoleCommands;

    // State needed to force skylight updates across several frames.
    bool force_skylight_update_completed_ = false;
    int force_skylight_update_previous_cvar_value_ = -1;
    float force_skylight_update_duration_seconds_ = 0.0;

    // State needed to terminate the config system.
    bool initialize_config_system_completed_ = false;
};
