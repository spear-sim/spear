//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpCore.h"

#include <stdint.h> // uint64_t

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
        uint64_t num_bytes = 1;
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

    SP_REGISTER_SUBSYSTEM_PROVIDER_CLASS(ULocalPlayer);

    SP_REGISTER_ACTOR_CLASS(AActor);
    SP_REGISTER_ACTOR_CLASS(AExponentialHeightFog);
    SP_REGISTER_ACTOR_CLASS(ALevelSequenceActor);
    SP_REGISTER_ACTOR_CLASS(ANavMeshBoundsVolume);
    SP_REGISTER_ACTOR_CLASS(ANavModifierVolume);
    SP_REGISTER_ACTOR_CLASS(APlayerCameraManager);
    SP_REGISTER_ACTOR_CLASS(APlayerController);
    SP_REGISTER_ACTOR_CLASS(APostProcessVolume);
    SP_REGISTER_ACTOR_CLASS(ARecastNavMesh);
    SP_REGISTER_ACTOR_CLASS(ASkyAtmosphere);
    SP_REGISTER_ACTOR_CLASS(AStaticMeshActor);
    SP_REGISTER_ACTOR_CLASS(AVolumetricCloud);

    SP_REGISTER_COMPONENT_CLASS(UActorComponent);
    SP_REGISTER_COMPONENT_CLASS(UBrushComponent);
    SP_REGISTER_COMPONENT_CLASS(UCameraComponent);
    SP_REGISTER_COMPONENT_CLASS(UCapsuleComponent);
    SP_REGISTER_COMPONENT_CLASS(UCharacterMovementComponent);
    SP_REGISTER_COMPONENT_CLASS(UDirectionalLightComponent);
    SP_REGISTER_COMPONENT_CLASS(UExponentialHeightFogComponent);
    SP_REGISTER_COMPONENT_CLASS(ULightComponent);
    SP_REGISTER_COMPONENT_CLASS(ULightComponentBase);
    SP_REGISTER_COMPONENT_CLASS(ULocalLightComponent);
    SP_REGISTER_COMPONENT_CLASS(UPhysicsConstraintComponent);
    SP_REGISTER_COMPONENT_CLASS(UPointLightComponent);
    SP_REGISTER_COMPONENT_CLASS(UPoseableMeshComponent);
    SP_REGISTER_COMPONENT_CLASS(UPrimitiveComponent);
    SP_REGISTER_COMPONENT_CLASS(USceneComponent);
    SP_REGISTER_COMPONENT_CLASS(USkeletalMeshComponent);
    SP_REGISTER_COMPONENT_CLASS(USkyAtmosphereComponent);
    SP_REGISTER_COMPONENT_CLASS(USkyLightComponent);
    SP_REGISTER_COMPONENT_CLASS(USpotLightComponent);
    SP_REGISTER_COMPONENT_CLASS(UStaticMeshComponent);
    SP_REGISTER_COMPONENT_CLASS(URectLightComponent);
    SP_REGISTER_COMPONENT_CLASS(UVolumetricCloudComponent);

    SP_REGISTER_INTERFACE_CLASS(IAssetRegistry);

    SP_REGISTER_CLASS(UClass);
    SP_REGISTER_CLASS(UGameplayStatics);
    SP_REGISTER_CLASS(UGameUserSettings);
    SP_REGISTER_CLASS(UKismetSystemLibrary);
    SP_REGISTER_CLASS(ULevelSequence);
    SP_REGISTER_CLASS(ULevelStreaming);
    SP_REGISTER_CLASS(UMaterial);
    SP_REGISTER_CLASS(UMaterialFunction);
    SP_REGISTER_CLASS(UMaterialInstance);
    SP_REGISTER_CLASS(UMaterialInstanceConstant);
    SP_REGISTER_CLASS(UMaterialInstanceDynamic);
    SP_REGISTER_CLASS(UMaterialInterface);
    SP_REGISTER_CLASS(UNavigationSystemV1);
    SP_REGISTER_CLASS(UObject);
    SP_REGISTER_CLASS(UStaticMesh);
    SP_REGISTER_CLASS(UTexture);
    SP_REGISTER_CLASS(UTexture2D);
    SP_REGISTER_CLASS(UTextureRenderTarget2D);

    SP_REGISTER_SPECIAL_STRUCT(FRotator);
    SP_REGISTER_SPECIAL_STRUCT(FTransform);
    SP_REGISTER_SPECIAL_STRUCT(FVector);
    SP_REGISTER_SPECIAL_STRUCT(FVector2D);
}

void SpCore::unregisterClasses() const
{
    // Unreal classes

    SP_UNREGISTER_SUBSYSTEM_PROVIDER_CLASS(ULocalPlayer);

    SP_UNREGISTER_ACTOR_CLASS(AActor);
    SP_UNREGISTER_ACTOR_CLASS(AExponentialHeightFog);
    SP_UNREGISTER_ACTOR_CLASS(ALevelSequenceActor);
    SP_UNREGISTER_ACTOR_CLASS(ANavMeshBoundsVolume);
    SP_UNREGISTER_ACTOR_CLASS(ANavModifierVolume);
    SP_UNREGISTER_ACTOR_CLASS(APlayerCameraManager);
    SP_UNREGISTER_ACTOR_CLASS(APlayerController);
    SP_UNREGISTER_ACTOR_CLASS(APostProcessVolume);
    SP_UNREGISTER_ACTOR_CLASS(ARecastNavMesh);
    SP_UNREGISTER_ACTOR_CLASS(ASkyAtmosphere);
    SP_UNREGISTER_ACTOR_CLASS(AStaticMeshActor);
    SP_UNREGISTER_ACTOR_CLASS(AVolumetricCloud);

    SP_UNREGISTER_COMPONENT_CLASS(UActorComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UBrushComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UCameraComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UCapsuleComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UCharacterMovementComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UDirectionalLightComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UExponentialHeightFogComponent);
    SP_UNREGISTER_COMPONENT_CLASS(ULightComponent);
    SP_UNREGISTER_COMPONENT_CLASS(ULightComponentBase);
    SP_UNREGISTER_COMPONENT_CLASS(ULocalLightComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UPhysicsConstraintComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UPointLightComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UPoseableMeshComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UPrimitiveComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USceneComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USkeletalMeshComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USkyAtmosphereComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USkyLightComponent);
    SP_UNREGISTER_COMPONENT_CLASS(USpotLightComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UStaticMeshComponent);
    SP_UNREGISTER_COMPONENT_CLASS(URectLightComponent);
    SP_UNREGISTER_COMPONENT_CLASS(UVolumetricCloudComponent);

    SP_UNREGISTER_INTERFACE_CLASS(IAssetRegistry);

    SP_UNREGISTER_CLASS(UClass);
    SP_UNREGISTER_CLASS(UGameplayStatics);
    SP_UNREGISTER_CLASS(UGameUserSettings);
    SP_UNREGISTER_CLASS(UKismetSystemLibrary);
    SP_UNREGISTER_CLASS(ULevelSequence);
    SP_UNREGISTER_CLASS(ULevelStreaming);
    SP_UNREGISTER_CLASS(UMaterial);
    SP_UNREGISTER_CLASS(UMaterialFunction);
    SP_UNREGISTER_CLASS(UMaterialInstance);
    SP_UNREGISTER_CLASS(UMaterialInstanceConstant);
    SP_UNREGISTER_CLASS(UMaterialInstanceDynamic);
    SP_UNREGISTER_CLASS(UMaterialInterface);
    SP_UNREGISTER_CLASS(UNavigationSystemV1);
    SP_UNREGISTER_CLASS(UObject);
    SP_UNREGISTER_CLASS(UStaticMesh);
    SP_UNREGISTER_CLASS(UTexture);
    SP_UNREGISTER_CLASS(UTexture2D);
    SP_UNREGISTER_CLASS(UTextureRenderTarget2D);
    
    SP_UNREGISTER_SPECIAL_STRUCT(FRotator);
    SP_UNREGISTER_SPECIAL_STRUCT(FTransform);
    SP_UNREGISTER_SPECIAL_STRUCT(FVector);
    SP_UNREGISTER_SPECIAL_STRUCT(FVector2D);
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
