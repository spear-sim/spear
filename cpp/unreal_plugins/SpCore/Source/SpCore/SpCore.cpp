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
#include <AssetRegistry/IAssetRegistry.h>
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
#include <Engine/LevelStreaming.h>
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
#include "SpCore/UnrealClassRegistry.h"

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

    SP_LOG_CURRENT_FUNCTION(); // useful to see that StartupModule() finishes executing
}

void SpCore::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    shared_memory_region_ = nullptr;

    unregisterClasses();
    SharedMemory::terminate();
    Config::terminate();

    SP_LOG_CURRENT_FUNCTION();  // useful to see that ShutdownModule() finishes executing
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpCore::registerClasses() const
{
    // Unreal classes
    UnrealClassRegistry::registerSubsystemProviderClass<ULocalPlayer>("ULocalPlayer");
    UnrealClassRegistry::registerActorClass<AActor>("AActor");
    UnrealClassRegistry::registerActorClass<AExponentialHeightFog>("AExponentialHeightFog");
    UnrealClassRegistry::registerActorClass<ALevelSequenceActor>("ALevelSequenceActor");
    UnrealClassRegistry::registerActorClass<ANavMeshBoundsVolume>("ANavMeshBoundsVolume");
    UnrealClassRegistry::registerActorClass<ANavModifierVolume>("ANavModifierVolume");
    UnrealClassRegistry::registerActorClass<APlayerCameraManager>("APlayerCameraManager");
    UnrealClassRegistry::registerActorClass<APlayerController>("APlayerController");
    UnrealClassRegistry::registerActorClass<APostProcessVolume>("APostProcessVolume");
    UnrealClassRegistry::registerActorClass<ARecastNavMesh>("ARecastNavMesh");
    UnrealClassRegistry::registerActorClass<ASkyAtmosphere>("ASkyAtmosphere");
    UnrealClassRegistry::registerActorClass<AStaticMeshActor>("AStaticMeshActor");
    UnrealClassRegistry::registerActorClass<AVolumetricCloud>("AVolumetricCloud");
    UnrealClassRegistry::registerComponentClass<UActorComponent>("UActorComponent");
    UnrealClassRegistry::registerComponentClass<UBrushComponent>("UBrushComponent");
    UnrealClassRegistry::registerComponentClass<UCameraComponent>("UCameraComponent");
    UnrealClassRegistry::registerComponentClass<UCapsuleComponent>("UCapsuleComponent");
    UnrealClassRegistry::registerComponentClass<UCharacterMovementComponent>("UCharacterMovementComponent");
    UnrealClassRegistry::registerComponentClass<UDirectionalLightComponent>("UDirectionalLightComponent");
    UnrealClassRegistry::registerComponentClass<UExponentialHeightFogComponent>("UExponentialHeightFogComponent");
    UnrealClassRegistry::registerComponentClass<ULightComponent>("ULightComponent");
    UnrealClassRegistry::registerComponentClass<ULightComponentBase>("ULightComponentBase");
    UnrealClassRegistry::registerComponentClass<ULocalLightComponent>("ULocalLightComponent");
    UnrealClassRegistry::registerComponentClass<UPhysicsConstraintComponent>("UPhysicsConstraintComponent");
    UnrealClassRegistry::registerComponentClass<UPointLightComponent>("UPointLightComponent");
    UnrealClassRegistry::registerComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    UnrealClassRegistry::registerComponentClass<UPrimitiveComponent>("UPrimitiveComponent");
    UnrealClassRegistry::registerComponentClass<USceneComponent>("USceneComponent");
    UnrealClassRegistry::registerComponentClass<USkeletalMeshComponent>("USkeletalMeshComponent");
    UnrealClassRegistry::registerComponentClass<USkyAtmosphereComponent>("USkyAtmosphereComponent");
    UnrealClassRegistry::registerComponentClass<USkyLightComponent>("USkyLightComponent");
    UnrealClassRegistry::registerComponentClass<USpotLightComponent>("USpotLightComponent");
    UnrealClassRegistry::registerComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    UnrealClassRegistry::registerComponentClass<URectLightComponent>("URectLightComponent");
    UnrealClassRegistry::registerComponentClass<UVolumetricCloudComponent>("UVolumetricCloudComponent");
    UnrealClassRegistry::registerInterface<IAssetRegistry>("IAssetRegistry");
    UnrealClassRegistry::registerClass<UClass>("UClass");
    UnrealClassRegistry::registerClass<UGameplayStatics>("UGameplayStatics");
    UnrealClassRegistry::registerClass<UGameUserSettings>("UGameUserSettings");
    UnrealClassRegistry::registerClass<UKismetSystemLibrary>("UKismetSystemLibrary");
    UnrealClassRegistry::registerClass<ULevelSequence>("ULevelSequence");
    UnrealClassRegistry::registerClass<ULevelStreaming>("ULevelStreaming");
    UnrealClassRegistry::registerClass<UMaterial>("UMaterial");
    UnrealClassRegistry::registerClass<UMaterialFunction>("UMaterialFunction");
    UnrealClassRegistry::registerClass<UMaterialInstance>("UMaterialInstance");
    UnrealClassRegistry::registerClass<UMaterialInstanceConstant>("UMaterialInstanceConstant");
    UnrealClassRegistry::registerClass<UMaterialInstanceDynamic>("UMaterialInstanceDynamic");
    UnrealClassRegistry::registerClass<UMaterialInterface>("UMaterialInterface");
    UnrealClassRegistry::registerClass<UNavigationSystemV1>("UNavigationSystemV1");
    UnrealClassRegistry::registerClass<UObject>("UObject");
    UnrealClassRegistry::registerClass<UStaticMesh>("UStaticMesh");
    UnrealClassRegistry::registerClass<UTexture>("UTexture");
    UnrealClassRegistry::registerClass<UTexture2D>("UTexture2D");
    UnrealClassRegistry::registerClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");
    UnrealClassRegistry::registerSpecialStruct<FRotator>("FRotator");
    UnrealClassRegistry::registerSpecialStruct<FTransform>("FTransform");
    UnrealClassRegistry::registerSpecialStruct<FVector>("FVector");
    UnrealClassRegistry::registerSpecialStruct<FVector2D>("FVector2D");
}

void SpCore::unregisterClasses() const
{
    // Unreal classes
    UnrealClassRegistry::unregisterSubsystemProviderClass<ULocalPlayer>("ULocalPlayer");
    UnrealClassRegistry::unregisterActorClass<AActor>("AActor");
    UnrealClassRegistry::unregisterActorClass<AExponentialHeightFog>("AExponentialHeightFog");
    UnrealClassRegistry::unregisterActorClass<ALevelSequenceActor>("ALevelSequenceActor");
    UnrealClassRegistry::unregisterActorClass<ANavMeshBoundsVolume>("ANavMeshBoundsVolume");
    UnrealClassRegistry::unregisterActorClass<ANavModifierVolume>("ANavModifierVolume");
    UnrealClassRegistry::unregisterActorClass<APlayerCameraManager>("APlayerCameraManager");
    UnrealClassRegistry::unregisterActorClass<APlayerController>("APlayerController");
    UnrealClassRegistry::unregisterActorClass<APostProcessVolume>("APostProcessVolume");
    UnrealClassRegistry::unregisterActorClass<ARecastNavMesh>("ARecastNavMesh");
    UnrealClassRegistry::unregisterActorClass<ASkyAtmosphere>("ASkyAtmosphere");
    UnrealClassRegistry::unregisterActorClass<AStaticMeshActor>("AStaticMeshActor");
    UnrealClassRegistry::unregisterActorClass<AVolumetricCloud>("AVolumetricCloud");
    UnrealClassRegistry::unregisterComponentClass<UActorComponent>("UActorComponent");
    UnrealClassRegistry::unregisterComponentClass<UBrushComponent>("UBrushComponent");
    UnrealClassRegistry::unregisterComponentClass<UCameraComponent>("UCameraComponent");
    UnrealClassRegistry::unregisterComponentClass<UCapsuleComponent>("UCapsuleComponent");
    UnrealClassRegistry::unregisterComponentClass<UCharacterMovementComponent>("UCharacterMovementComponent");
    UnrealClassRegistry::unregisterComponentClass<UDirectionalLightComponent>("UDirectionalLightComponent");
    UnrealClassRegistry::unregisterComponentClass<UExponentialHeightFogComponent>("UExponentialHeightFogComponent");
    UnrealClassRegistry::unregisterComponentClass<ULightComponent>("ULightComponent");
    UnrealClassRegistry::unregisterComponentClass<ULightComponentBase>("ULightComponentBase");
    UnrealClassRegistry::unregisterComponentClass<ULocalLightComponent>("ULocalLightComponent");
    UnrealClassRegistry::unregisterComponentClass<UPhysicsConstraintComponent>("UPhysicsConstraintComponent");
    UnrealClassRegistry::unregisterComponentClass<UPointLightComponent>("UPointLightComponent");
    UnrealClassRegistry::unregisterComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    UnrealClassRegistry::unregisterComponentClass<UPrimitiveComponent>("UPrimitiveComponent");
    UnrealClassRegistry::unregisterComponentClass<USceneComponent>("USceneComponent");
    UnrealClassRegistry::unregisterComponentClass<USkeletalMeshComponent>("USkeletalMeshComponent");
    UnrealClassRegistry::unregisterComponentClass<USkyAtmosphereComponent>("USkyAtmosphereComponent");
    UnrealClassRegistry::unregisterComponentClass<USkyLightComponent>("USkyLightComponent");
    UnrealClassRegistry::unregisterComponentClass<USpotLightComponent>("USpotLightComponent");
    UnrealClassRegistry::unregisterComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    UnrealClassRegistry::unregisterComponentClass<URectLightComponent>("URectLightComponent");
    UnrealClassRegistry::unregisterComponentClass<UVolumetricCloudComponent>("UVolumetricCloudComponent");
    UnrealClassRegistry::unregisterInterface<IAssetRegistry>("IAssetRegistry");
    UnrealClassRegistry::unregisterClass<UClass>("UClass");
    UnrealClassRegistry::unregisterClass<UGameplayStatics>("UGameplayStatics");
    UnrealClassRegistry::unregisterClass<UGameUserSettings>("UGameUserSettings");
    UnrealClassRegistry::unregisterClass<UKismetSystemLibrary>("UKismetSystemLibrary");
    UnrealClassRegistry::unregisterClass<ULevelSequence>("ULevelSequence");
    UnrealClassRegistry::unregisterClass<ULevelStreaming>("ULevelStreaming");
    UnrealClassRegistry::unregisterClass<UMaterial>("UMaterial");
    UnrealClassRegistry::unregisterClass<UMaterialFunction>("UMaterialFunction");
    UnrealClassRegistry::unregisterClass<UMaterialInstance>("UMaterialInstance");
    UnrealClassRegistry::unregisterClass<UMaterialInstanceConstant>("UMaterialInstanceConstant");
    UnrealClassRegistry::unregisterClass<UMaterialInstanceDynamic>("UMaterialInstanceDynamic");
    UnrealClassRegistry::unregisterClass<UMaterialInterface>("UMaterialInterface");
    UnrealClassRegistry::unregisterClass<UNavigationSystemV1>("UNavigationSystemV1");
    UnrealClassRegistry::unregisterClass<UObject>("UObject");
    UnrealClassRegistry::unregisterClass<UStaticMesh>("UStaticMesh");
    UnrealClassRegistry::unregisterClass<UTexture>("UTexture");
    UnrealClassRegistry::unregisterClass<UTexture2D>("UTexture2D");
    UnrealClassRegistry::unregisterClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");
    UnrealClassRegistry::unregisterSpecialStruct<FRotator>("FRotator");
    UnrealClassRegistry::unregisterSpecialStruct<FTransform>("FTransform");
    UnrealClassRegistry::unregisterSpecialStruct<FVector>("FVector");
    UnrealClassRegistry::unregisterSpecialStruct<FVector2D>("FVector2D");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
