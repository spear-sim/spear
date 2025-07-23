//
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
#include <Components/PoseableMeshComponent.h>
#include <Components/PrimitiveComponent.h>
#include <Components/SceneComponent.h>
#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/LocalPlayer.h>
#include <Engine/PostProcessVolume.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/TextureRenderTarget2D.h>
#include <GameFramework/Actor.h>
#include <GameFramework/CharacterMovementComponent.h>
#include <GameFramework/GameUserSettings.h>
#include <GameFramework/PlayerController.h>
#include <Kismet/GameplayStatics.h>
#include <Materials/Material.h>
#include <Materials/MaterialInterface.h>
#include <Math/Rotator.h>
#include <Math/Transform.h>
#include <Math/Vector.h>
#include <NavigationSystem.h>
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
    UnrealClassRegistrar::registerActorClass<APlayerCameraManager>("APlayerCameraManager");
    UnrealClassRegistrar::registerActorClass<APlayerController>("APlayerController");
    UnrealClassRegistrar::registerActorClass<APostProcessVolume>("APostProcessVolume");
    UnrealClassRegistrar::registerActorClass<AStaticMeshActor>("AStaticMeshActor");
    UnrealClassRegistrar::registerComponentClass<UActorComponent>("UActorComponent");
    UnrealClassRegistrar::registerComponentClass<UCameraComponent>("UCameraComponent");
    UnrealClassRegistrar::registerComponentClass<UCharacterMovementComponent>("UCharacterMovementComponent");
    UnrealClassRegistrar::registerComponentClass<UPrimitiveComponent>("UPrimitiveComponent");
    UnrealClassRegistrar::registerComponentClass<USkeletalMeshComponent>("USkeletalMeshComponent");
    UnrealClassRegistrar::registerComponentClass<USceneComponent>("USceneComponent");
    UnrealClassRegistrar::registerComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    UnrealClassRegistrar::registerComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    UnrealClassRegistrar::registerClass<UObject>("UObject");
    UnrealClassRegistrar::registerClass<UClass>("UClass");
    UnrealClassRegistrar::registerClass<UGameplayStatics>("UGameplayStatics");
    UnrealClassRegistrar::registerClass<UGameUserSettings>("UGameUserSettings");
    UnrealClassRegistrar::registerClass<UMaterial>("UMaterial");
    UnrealClassRegistrar::registerClass<UMaterialInterface>("UMaterialInterface");
    UnrealClassRegistrar::registerClass<UNavigationSystemV1>("UNavigationSystemV1");
    UnrealClassRegistrar::registerClass<UStaticMesh>("UStaticMesh");
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
    UnrealClassRegistrar::unregisterActorClass<APlayerCameraManager>("APlayerCameraManager");
    UnrealClassRegistrar::unregisterActorClass<APlayerController>("APlayerController");
    UnrealClassRegistrar::unregisterActorClass<APostProcessVolume>("APostProcessVolume");
    UnrealClassRegistrar::unregisterActorClass<AStaticMeshActor>("AStaticMeshActor");
    UnrealClassRegistrar::unregisterComponentClass<UActorComponent>("UActorComponent");
    UnrealClassRegistrar::unregisterComponentClass<UCameraComponent>("UCameraComponent");
    UnrealClassRegistrar::unregisterComponentClass<UCharacterMovementComponent>("UCharacterMovementComponent");
    UnrealClassRegistrar::unregisterComponentClass<UPrimitiveComponent>("UPrimitiveComponent");
    UnrealClassRegistrar::unregisterComponentClass<USceneComponent>("USceneComponent");
    UnrealClassRegistrar::unregisterComponentClass<USkeletalMeshComponent>("USkeletalMeshComponent");
    UnrealClassRegistrar::unregisterComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    UnrealClassRegistrar::unregisterComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    UnrealClassRegistrar::unregisterClass<UObject>("UObject");
    UnrealClassRegistrar::unregisterClass<UClass>("UClass");
    UnrealClassRegistrar::unregisterClass<UGameplayStatics>("UGameplayStatics");
    UnrealClassRegistrar::unregisterClass<UGameUserSettings>("UGameUserSettings");
    UnrealClassRegistrar::unregisterClass<UMaterial>("UMaterial");
    UnrealClassRegistrar::unregisterClass<UMaterialInterface>("UMaterialInterface");
    UnrealClassRegistrar::unregisterClass<UNavigationSystemV1>("UNavigationSystemV1");
    UnrealClassRegistrar::unregisterClass<UStaticMesh>("UStaticMesh");
    UnrealClassRegistrar::unregisterClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");
    UnrealClassRegistrar::unregisterSpecialStruct<FRotator>("FRotator");
    UnrealClassRegistrar::unregisterSpecialStruct<FTransform>("FTransform");
    UnrealClassRegistrar::unregisterSpecialStruct<FVector>("FVector");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpCore, SpCore);
