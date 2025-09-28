//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <stddef.h> // uint64_t

#include <iostream> // std::cin
#include <memory>   // std::make_unique, std::unique_ptr

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

// Unreal classes
#include <Camera/CameraComponent.h>
#include <Camera/PlayerCameraManager.h>
#include <Components/ActorComponent.h>
#include <Components/BrushComponent.h>
#include <Components/CapsuleComponent.h>
#include <Components/DirectionalLightComponent.h>
#include <Components/ExponentialHeightFogComponent.h>
#include <Components/LightComponent.h>
#include <Components/LightComponentBase.h>
#include <Components/LocalLightComponent.h>
#include <Components/PointLightComponent.h>
#include <Components/PoseableMeshComponent.h>
#include <Components/PrimitiveComponent.h>
#include <Components/RectLightComponent.h>
#include <Components/SceneComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <Components/SkyAtmosphereComponent.h> // ASkyAtmosphere
#include <Components/SkyLightComponent.h>
#include <Components/SpotLightComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Components/VolumetricCloudComponent.h> // AVolumetricCloud
#include <Engine/ExponentialHeightFog.h>
#include <Engine/LocalPlayer.h>
#include <Engine/PostProcessVolume.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/Texture.h>
#include <Engine/Texture2D.h>
#include <Engine/TextureRenderTarget2D.h>
#include <GameFramework/Actor.h>
#include <GameFramework/CharacterMovementComponent.h>
#include <GameFramework/GameUserSettings.h>
#include <GameFramework/PlayerController.h>
#include <GameFramework/PlayerStart.h>
#include <Kismet/GameplayStatics.h>
#include <Kismet/KismetSystemLibrary.h>
#include <LevelSequence.h>
#include <LevelSequenceActor.h>
#include <Materials/Material.h>
#include <Materials/MaterialFunction.h>
#include <Materials/MaterialInstance.h>
#include <Materials/MaterialInstanceConstant.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Rotator.h>
#include <Math/Transform.h>
#include <Math/Vector.h>
#include <NavigationSystem.h>
#include <NavModifierVolume.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>
#include <UObject/Class.h>  // UClass
#include <UObject/Object.h> // UObject

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/UnrealClassRegistrar.h"

void SpCore::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    Config::requestInitialize();

    // Wait for keyboard input, which is useful when attempting to attach a debugger to the running executable.
    if (Config::isInitialized() && Config::get<bool>("SP_CORE.WAIT_FOR_KEYBOARD_INPUT_DURING_INITIALIZATION")) {
        SP_LOG("    Press any key to continue...");
        std::cin.get();
        SP_LOG("    Received keyboard input, continuing...");
    }

    // Initialize shared memory system
    uint64_t shared_memory_initial_unique_id = 0;
    if (Config::isInitialized()) {
        shared_memory_initial_unique_id = Config::get<unsigned int>("SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID");
    }
    SharedMemory::initialize(shared_memory_initial_unique_id);

    // Register Unreal classes to be accessible from Python
    registerClasses();

    // Try to create a shared memory region so we can provide a meaningful error message if it fails.
    try {
        int num_bytes = 1;
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
    } catch (...) {
        SP_LOG("    ERROR: Couldn't create a shared memory region. The Unreal Editor might be open already, or there might be another SpearSim executable running in the background. Close the Unreal Editor and other SpearSim executables, or change SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID to an unused ID, and try launching again.");
        std::rethrow_exception(std::current_exception());
    }
}

void SpCore::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    shared_memory_region_ = nullptr;

    unregisterClasses();
    SharedMemory::terminate();
    Config::terminate();
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpCore::registerClasses()
{
    // Unreal classes
    UnrealClassRegistrar::registerSubsystemProviderClass<ULocalPlayer>("ULocalPlayer");
    UnrealClassRegistrar::registerActorClass<AActor>("AActor");
    UnrealClassRegistrar::registerActorClass<AExponentialHeightFog>("AExponentialHeightFog");
    UnrealClassRegistrar::registerActorClass<ALevelSequenceActor>("ALevelSequenceActor");
    UnrealClassRegistrar::registerActorClass<ANavMeshBoundsVolume>("ANavMeshBoundsVolume");
    UnrealClassRegistrar::registerActorClass<ANavModifierVolume>("ANavModifierVolume");
    UnrealClassRegistrar::registerActorClass<APlayerCameraManager>("APlayerCameraManager");
    UnrealClassRegistrar::registerActorClass<APlayerController>("APlayerController");
    UnrealClassRegistrar::registerActorClass<APostProcessVolume>("APostProcessVolume");
    UnrealClassRegistrar::registerActorClass<ARecastNavMesh>("ARecastNavMesh");
    UnrealClassRegistrar::registerActorClass<ASkyAtmosphere>("ASkyAtmosphere");
    UnrealClassRegistrar::registerActorClass<AStaticMeshActor>("AStaticMeshActor");
    UnrealClassRegistrar::registerActorClass<AVolumetricCloud>("AVolumetricCloud");
    UnrealClassRegistrar::registerComponentClass<UActorComponent>("UActorComponent");
    UnrealClassRegistrar::registerComponentClass<UBrushComponent>("UBrushComponent");
    UnrealClassRegistrar::registerComponentClass<UCameraComponent>("UCameraComponent");
    UnrealClassRegistrar::registerComponentClass<UCapsuleComponent>("UCapsuleComponent");
    UnrealClassRegistrar::registerComponentClass<UCharacterMovementComponent>("UCharacterMovementComponent");
    UnrealClassRegistrar::registerComponentClass<UDirectionalLightComponent>("UDirectionalLightComponent");
    UnrealClassRegistrar::registerComponentClass<UExponentialHeightFogComponent>("UExponentialHeightFogComponent");
    UnrealClassRegistrar::registerComponentClass<ULightComponent>("ULightComponent");
    UnrealClassRegistrar::registerComponentClass<ULightComponentBase>("ULightComponentBase");
    UnrealClassRegistrar::registerComponentClass<ULocalLightComponent>("ULocalLightComponent");
    UnrealClassRegistrar::registerComponentClass<UPhysicsConstraintComponent>("UPhysicsConstraintComponent");
    UnrealClassRegistrar::registerComponentClass<UPointLightComponent>("UPointLightComponent");
    UnrealClassRegistrar::registerComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    UnrealClassRegistrar::registerComponentClass<UPrimitiveComponent>("UPrimitiveComponent");
    UnrealClassRegistrar::registerComponentClass<USceneComponent>("USceneComponent");
    UnrealClassRegistrar::registerComponentClass<USkeletalMeshComponent>("USkeletalMeshComponent");
    UnrealClassRegistrar::registerComponentClass<USkyAtmosphereComponent>("USkyAtmosphereComponent");
    UnrealClassRegistrar::registerComponentClass<USkyLightComponent>("USkyLightComponent");
    UnrealClassRegistrar::registerComponentClass<USpotLightComponent>("USpotLightComponent");
    UnrealClassRegistrar::registerComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    UnrealClassRegistrar::registerComponentClass<URectLightComponent>("URectLightComponent");
    UnrealClassRegistrar::registerComponentClass<UVolumetricCloudComponent>("UVolumetricCloudComponent");
    UnrealClassRegistrar::registerClass<UClass>("UClass");
    UnrealClassRegistrar::registerClass<UGameplayStatics>("UGameplayStatics");
    UnrealClassRegistrar::registerClass<UGameUserSettings>("UGameUserSettings");
    UnrealClassRegistrar::registerClass<UKismetSystemLibrary>("UKismetSystemLibrary");
    UnrealClassRegistrar::registerClass<ULevelSequence>("ULevelSequence");
    UnrealClassRegistrar::registerClass<UMaterial>("UMaterial");
    UnrealClassRegistrar::registerClass<UMaterialFunction>("UMaterialFunction");
    UnrealClassRegistrar::registerClass<UMaterialInstance>("UMaterialInstance");
    UnrealClassRegistrar::registerClass<UMaterialInstanceConstant>("UMaterialInstanceConstant");
    UnrealClassRegistrar::registerClass<UMaterialInstanceDynamic>("UMaterialInstanceDynamic");
    UnrealClassRegistrar::registerClass<UMaterialInterface>("UMaterialInterface");
    UnrealClassRegistrar::registerClass<UNavigationSystemV1>("UNavigationSystemV1");
    UnrealClassRegistrar::registerClass<UObject>("UObject");
    UnrealClassRegistrar::registerClass<UStaticMesh>("UStaticMesh");
    UnrealClassRegistrar::registerClass<UTexture>("UTexture");
    UnrealClassRegistrar::registerClass<UTexture2D>("UTexture2D");
    UnrealClassRegistrar::registerClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");
    UnrealClassRegistrar::registerSpecialStruct<FRotator>("FRotator");
    UnrealClassRegistrar::registerSpecialStruct<FTransform>("FTransform");
    UnrealClassRegistrar::registerSpecialStruct<FVector>("FVector");
}

void SpCore::unregisterClasses()
{
    // Unreal classes
    UnrealClassRegistrar::unregisterSubsystemProviderClass<ULocalPlayer>("ULocalPlayer");
    UnrealClassRegistrar::unregisterActorClass<AActor>("AActor");
    UnrealClassRegistrar::unregisterActorClass<AExponentialHeightFog>("AExponentialHeightFog");
    UnrealClassRegistrar::unregisterActorClass<ALevelSequenceActor>("ALevelSequenceActor");
    UnrealClassRegistrar::unregisterActorClass<ANavMeshBoundsVolume>("ANavMeshBoundsVolume");
    UnrealClassRegistrar::unregisterActorClass<ANavModifierVolume>("ANavModifierVolume");
    UnrealClassRegistrar::unregisterActorClass<APlayerCameraManager>("APlayerCameraManager");
    UnrealClassRegistrar::unregisterActorClass<APlayerController>("APlayerController");
    UnrealClassRegistrar::unregisterActorClass<APostProcessVolume>("APostProcessVolume");
    UnrealClassRegistrar::unregisterActorClass<ARecastNavMesh>("ARecastNavMesh");
    UnrealClassRegistrar::unregisterActorClass<ASkyAtmosphere>("ASkyAtmosphere");
    UnrealClassRegistrar::unregisterActorClass<AStaticMeshActor>("AStaticMeshActor");
    UnrealClassRegistrar::unregisterActorClass<AVolumetricCloud>("AVolumetricCloud");
    UnrealClassRegistrar::unregisterComponentClass<UActorComponent>("UActorComponent");
    UnrealClassRegistrar::unregisterComponentClass<UBrushComponent>("UBrushComponent");
    UnrealClassRegistrar::unregisterComponentClass<UCameraComponent>("UCameraComponent");
    UnrealClassRegistrar::unregisterComponentClass<UCapsuleComponent>("UCapsuleComponent");
    UnrealClassRegistrar::unregisterComponentClass<UCharacterMovementComponent>("UCharacterMovementComponent");
    UnrealClassRegistrar::unregisterComponentClass<UDirectionalLightComponent>("UDirectionalLightComponent");
    UnrealClassRegistrar::unregisterComponentClass<UExponentialHeightFogComponent>("UExponentialHeightFogComponent");
    UnrealClassRegistrar::unregisterComponentClass<ULightComponent>("ULightComponent");
    UnrealClassRegistrar::unregisterComponentClass<ULightComponentBase>("ULightComponentBase");
    UnrealClassRegistrar::unregisterComponentClass<ULocalLightComponent>("ULocalLightComponent");
    UnrealClassRegistrar::unregisterComponentClass<UPhysicsConstraintComponent>("UPhysicsConstraintComponent");
    UnrealClassRegistrar::unregisterComponentClass<UPointLightComponent>("UPointLightComponent");
    UnrealClassRegistrar::unregisterComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    UnrealClassRegistrar::unregisterComponentClass<UPrimitiveComponent>("UPrimitiveComponent");
    UnrealClassRegistrar::unregisterComponentClass<USceneComponent>("USceneComponent");
    UnrealClassRegistrar::unregisterComponentClass<USkeletalMeshComponent>("USkeletalMeshComponent");
    UnrealClassRegistrar::unregisterComponentClass<USkyAtmosphereComponent>("USkyAtmosphereComponent");
    UnrealClassRegistrar::unregisterComponentClass<USkyLightComponent>("USkyLightComponent");
    UnrealClassRegistrar::unregisterComponentClass<USpotLightComponent>("USpotLightComponent");
    UnrealClassRegistrar::unregisterComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    UnrealClassRegistrar::unregisterComponentClass<URectLightComponent>("URectLightComponent");
    UnrealClassRegistrar::unregisterComponentClass<UVolumetricCloudComponent>("UVolumetricCloudComponent");
    UnrealClassRegistrar::unregisterClass<UClass>("UClass");
    UnrealClassRegistrar::unregisterClass<UGameplayStatics>("UGameplayStatics");
    UnrealClassRegistrar::unregisterClass<UGameUserSettings>("UGameUserSettings");
    UnrealClassRegistrar::unregisterClass<UKismetSystemLibrary>("UKismetSystemLibrary");
    UnrealClassRegistrar::unregisterClass<ULevelSequence>("ULevelSequence");
    UnrealClassRegistrar::unregisterClass<UMaterial>("UMaterial");
    UnrealClassRegistrar::unregisterClass<UMaterialFunction>("UMaterialFunction");
    UnrealClassRegistrar::unregisterClass<UMaterialInstance>("UMaterialInstance");
    UnrealClassRegistrar::unregisterClass<UMaterialInstanceConstant>("UMaterialInstanceConstant");
    UnrealClassRegistrar::unregisterClass<UMaterialInstanceDynamic>("UMaterialInstanceDynamic");
    UnrealClassRegistrar::unregisterClass<UMaterialInterface>("UMaterialInterface");
    UnrealClassRegistrar::unregisterClass<UNavigationSystemV1>("UNavigationSystemV1");
    UnrealClassRegistrar::unregisterClass<UObject>("UObject");
    UnrealClassRegistrar::unregisterClass<UStaticMesh>("UStaticMesh");
    UnrealClassRegistrar::unregisterClass<UTexture>("UTexture");
    UnrealClassRegistrar::unregisterClass<UTexture2D>("UTexture2D");
    UnrealClassRegistrar::unregisterClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");
    UnrealClassRegistrar::unregisterSpecialStruct<FRotator>("FRotator");
    UnrealClassRegistrar::unregisterSpecialStruct<FTransform>("FTransform");
    UnrealClassRegistrar::unregisterSpecialStruct<FVector>("FVector");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
