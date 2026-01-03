//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include <Components/ActorComponent.h>
#include <Containers/Map.h>
#include <Containers/UnrealString.h>     // FString
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                // SPCORE_API
#include <UObject/NameTypes.h>           // FName
#include <UObject/Object.h>              // UObject
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpStableNameManager.generated.h"

class ULevel;
class UWorld;
struct FPropertyChangedEvent;

//
// If an ASpStableNameComponent is place in a level, then all the actors in that level can be found using
// their names and paths as shown in the Unreal Editor Outliner view, even in standalone shipping builds,
// using UnrealUtils::findActorByName(...) and similar functions.
//

UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpStableNameManager : public AActor
{
    GENERATED_BODY()
public:
    #if WITH_EDITOR // defined in an auto-generated header
        // AActor interface
        void PostActorCreated() override;
    #endif

    bool hasActor(const AActor* actor) const;
    void addActor(const AActor* actor, const std::string& stable_name);
    void removeActor(const AActor* actor);

    std::string getStableName(const AActor* actor) const;
    void setStableName(const AActor* actor, const std::string& stable_name);

    #if WITH_EDITOR // defined in an auto-generated header
        // Interface for updating StableNames
        void requestAddActor(AActor* actor);
        void requestRemoveActor(AActor* actor);
        void requestUpdateActor(AActor* actor);
        void requestUpdateAllActors();
    #endif

    static std::string getStableIdString(const AActor* actor);

    #if WITH_EDITOR // defined in an auto-generated header
        static std::string getStableNameEditorOnly(const AActor* actor);
    #endif

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    TMap<FString, FString> StableNames;
};

//
// If a USpStableNameComponent is attached to an actor, then it can be found using its name and path in the
// Unreal Editor Outliner view, even in standalone shipping builds, using UnrealUtils::findActorByName(...)
// and similar functions.
//

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Activation, AssetUserData, Collision, Cooking, LOD, Navigation, Physics, Rendering, Tags), meta=(BlueprintSpawnableComponent))
class SPCORE_API USpStableNameComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    #if WITH_EDITOR // defined in an auto-generated header
        // UActorComponent interface
        void OnComponentCreated() override;
    #endif

    std::string getStableName();
    void setStableName(const std::string& stable_name);

    #if WITH_EDITOR // defined in an auto-generated header
        // Interface for updating StableName
        void requestUpdate();
    #endif

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    FString StableName;
};

//
// The USpStableNameEventHandler default object listens to UI events in the editor and keeps
// ASpStableNameManager and USpStableNameComponent objects up-to-date.
//

UCLASS()
class USpStableNameEventHandler : public UObject
{
    GENERATED_BODY()
public: 
    USpStableNameEventHandler();
    ~USpStableNameEventHandler() override;

#if WITH_EDITOR // defined in an auto-generated header
    private:
        void postEngineInitHandler();
        void enginePreExitHandler();

        void actorLabelChangedHandler(AActor* actor);

        void objectPropertyChangedHandler(UObject* uobject, FPropertyChangedEvent& event);

        void levelAddedToWorldHandler(ULevel* level, UWorld* world);
        void levelRemovedFromWorldHandler(ULevel* level, UWorld* world);

        void levelActorAddedHandler(AActor* actor);
        void levelActorFolderChangedHandler(const AActor* actor, FName name);
        void levelActorDeletedHandler(AActor* actor);

        FDelegateHandle post_engine_init_handle_;
        FDelegateHandle engine_pre_exit_handle_;

        FDelegateHandle actor_label_changed_handle_;

        FDelegateHandle object_property_changed_handle_;

        FDelegateHandle level_added_to_world_handle_;    
        FDelegateHandle level_removed_from_world_handle_;

        FDelegateHandle level_actor_added_handle_;
        FDelegateHandle level_actor_folder_changed_handle_;
        FDelegateHandle level_actor_deleted_handle_;
#endif
};
