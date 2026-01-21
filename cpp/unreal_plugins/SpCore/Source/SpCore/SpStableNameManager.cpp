//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpStableNameManager.h"

#include <algorithm> // std::ranges::reverse
#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // FWorldDelegates
#include <GameFramework/Actor.h>
#include <Misc/CoreDelegates.h>
#include <Templates/Casts.h>
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // EObjectFlags
#include <UObject/Package.h>
#include <UObject/SoftObjectPath.h>
#include <UObject/UnrealType.h>          // FProperty, FPropertyChangedEvent
#include <UObject/UObjectGlobals.h>      // FCoreUObjectDelegates

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

//
// ASpStableNameManager
//

#if WITH_EDITOR // defined in an auto-generated header
    void ASpStableNameManager::PostActorCreated()
    {
        AActor::PostActorCreated();
        requestUpdateAllActors();
    }
#endif

bool ASpStableNameManager::hasActor(const AActor* actor) const
{
    FString id = Unreal::toFString(getStableIdString(actor));
    return StableNames.Contains(id);
}

void ASpStableNameManager::addActor(const AActor* actor, const std::string& stable_name)
{
    FString id = Unreal::toFString(getStableIdString(actor));
    SP_ASSERT(!StableNames.Contains(id));
    StableNames.Add(id, Unreal::toFString(stable_name));
}

void ASpStableNameManager::removeActor(const AActor* actor)
{
    FString id = Unreal::toFString(getStableIdString(actor));
    SP_ASSERT(StableNames.Contains(id));
    StableNames.Remove(id);
}

std::string ASpStableNameManager::getStableName(const AActor* actor) const
{
    FString id = Unreal::toFString(getStableIdString(actor));
    SP_ASSERT(StableNames.Contains(id));
    return Unreal::toStdString(StableNames[id]);
}

void ASpStableNameManager::setStableName(const AActor* actor, const std::string& stable_name)
{
    FString id = Unreal::toFString(getStableIdString(actor));
    SP_ASSERT(StableNames.Contains(id));
    StableNames[id] = Unreal::toFString(stable_name);
}

#if WITH_EDITOR // defined in an auto-generated header
    void ASpStableNameManager::requestAddActor(AActor* actor)
    {
        SP_ASSERT(actor);

        if (actor->HasAnyFlags(EObjectFlags::RF_Transient) ||
            actor->GetWorld() != GetWorld() ||
            actor->IsA(ASpStableNameManager::StaticClass()) ||
            !actor->IsListedInSceneOutliner()) {
                return;
            }

        FString id = Unreal::toFString(getStableIdString(actor));
        FString stable_name = Unreal::toFString(getStableNameEditorOnly(actor));
        if (StableNames.Contains(id)) {
            StableNames[id] = stable_name;
        } else {
            StableNames.Add(id, stable_name);
        }
    }

    void ASpStableNameManager::requestRemoveActor(AActor* actor)
    {
        SP_ASSERT(actor);

        if (actor->HasAnyFlags(EObjectFlags::RF_Transient) ||
            actor->GetWorld() != GetWorld() ||
            actor->IsA(ASpStableNameManager::StaticClass()) ||
            !actor->IsListedInSceneOutliner()) {
                return;
            }

        // it is not guaranteed that the actor is contained in StableNames, e.g., if the user adds multiple ASpStableNameManager actors
        FString id = Unreal::toFString(getStableIdString(actor));
        if (StableNames.Contains(id)) {
            StableNames.Remove(id);
        }
    }

    void ASpStableNameManager::requestUpdateActor(AActor* actor)
    {
        if (actor->HasAnyFlags(EObjectFlags::RF_Transient) ||
            actor->GetWorld() != GetWorld() ||
            actor->IsA(ASpStableNameManager::StaticClass()) ||
            !actor->IsListedInSceneOutliner()) {
                return;
            }

        FString id = Unreal::toFString(getStableIdString(actor));
        FString stable_name = Unreal::toFString(getStableNameEditorOnly(actor));
        if (StableNames.Contains(id)) {
            StableNames[id] = stable_name;
        } else {
            StableNames.Add(id, stable_name);
        }
    }

    void ASpStableNameManager::requestUpdateAllActors()
    {
        UWorld* world = GetWorld();
        if (!world) {
            return;
        }

        StableNames.Empty();
        std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
        for (auto actor : actors) {
            if (actor != this) {
                requestAddActor(actor);
            }
        }
    }
#endif

std::string ASpStableNameManager::getStableIdString(const AActor* actor)
{
    SP_ASSERT(actor);
    SP_ASSERT(actor->GetOutermost());

    FSoftObjectPath soft_object_path = FSoftObjectPath(actor);

    FString asset_path_string = soft_object_path.GetAssetPathString();
    FString sub_path_string = soft_object_path.GetSubPathString();
    FString non_pie_asset_path_string;
    if (actor->GetOutermost()->GetPIEInstanceID() == -1) {
        non_pie_asset_path_string = asset_path_string;
    } else {
        FString pie_prefix = UWorld::BuildPIEPackagePrefix(actor->GetOutermost()->GetPIEInstanceID());
        non_pie_asset_path_string = UWorld::StripPIEPrefixFromPackageName(asset_path_string, pie_prefix);
    }

    return Unreal::toStdString(non_pie_asset_path_string + ":" + sub_path_string);
}

#if WITH_EDITOR // defined in an auto-generated header
    std::string ASpStableNameManager::getStableNameEditorOnly(const AActor* actor)
    {
        SP_ASSERT(actor);

        FName folder_path = actor->GetFolderPath();
        std::string folder_path_string = "";
        if (!folder_path.IsNone()) {
            folder_path_string = Unreal::toStdString(folder_path) + "/";
        }

        std::string label_string = Unreal::toStdString(actor->GetActorLabel());

        return folder_path_string + label_string;
    }
#endif

//
// USpStableNameComponent
//

#if WITH_EDITOR // defined in an auto-generated header
    void USpStableNameComponent::OnComponentCreated()
    {
        UActorComponent::OnComponentCreated();
        requestUpdate();
    }
#endif

std::string USpStableNameComponent::getStableName()
{
    return Unreal::toStdString(StableName);
}

void USpStableNameComponent::setStableName(const std::string& stable_name)
{
    StableName = Unreal::toFString(stable_name);
}

#if WITH_EDITOR // defined in an auto-generated header
    void USpStableNameComponent::requestUpdate()
    {
        AActor* actor = GetOwner();
        if (!actor) {
            return;
        }

        // This method will not update the stable name of any actor spawned at runtime. Any such actor
        // needs to update its stable name via Unreal::setStableName(...).
        if (!actor->HasAnyFlags(EObjectFlags::RF_Transient)) {
            StableName = Unreal::toFString(ASpStableNameManager::getStableNameEditorOnly(actor));
        }
    }
#endif

//
// USpStableNameEventHandler
//

USpStableNameEventHandler::USpStableNameEventHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        post_engine_init_handle_ = FCoreDelegates::OnPostEngineInit.AddUObject(this, &USpStableNameEventHandler::postEngineInitHandler);
        engine_pre_exit_handle_  = FCoreDelegates::OnEnginePreExit.AddUObject(this, &USpStableNameEventHandler::enginePreExitHandler);
    #endif
}

USpStableNameEventHandler::~USpStableNameEventHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        FCoreDelegates::OnEnginePreExit.Remove(engine_pre_exit_handle_);
        FCoreDelegates::OnPostEngineInit.Remove(post_engine_init_handle_);
        engine_pre_exit_handle_.Reset();
        post_engine_init_handle_.Reset();
    #endif
}

#if WITH_EDITOR // defined in an auto-generated header
    void USpStableNameEventHandler::postEngineInitHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(GEngine);
        
        actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &USpStableNameEventHandler::actorLabelChangedHandler);

        level_added_to_world_handle_     = FWorldDelegates::LevelAddedToWorld.AddUObject(this, &USpStableNameEventHandler::levelAddedToWorldHandler);
        level_removed_from_world_handle_ = FWorldDelegates::LevelRemovedFromWorld.AddUObject(this, &USpStableNameEventHandler::levelRemovedFromWorldHandler);

        level_actor_added_handle_          = GEngine->OnLevelActorAdded().AddUObject(this, &USpStableNameEventHandler::levelActorAddedHandler);
        level_actor_folder_changed_handle_ = GEngine->OnLevelActorFolderChanged().AddUObject(this, &USpStableNameEventHandler::levelActorFolderChangedHandler);
        level_actor_deleted_handle_        = GEngine->OnLevelActorDeleted().AddUObject(this, &USpStableNameEventHandler::levelActorDeletedHandler);
    }

    void USpStableNameEventHandler::enginePreExitHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(GEngine);

        GEngine->OnLevelActorAdded().Remove(level_actor_added_handle_);
        GEngine->OnLevelActorFolderChanged().Remove(level_actor_folder_changed_handle_);
        GEngine->OnLevelActorDeleted().Remove(level_actor_deleted_handle_);
        level_actor_added_handle_.Reset();
        level_actor_folder_changed_handle_.Reset();
        level_actor_deleted_handle_.Reset();

        FWorldDelegates::LevelAddedToWorld.Remove(level_added_to_world_handle_);
        FWorldDelegates::LevelRemovedFromWorld.Remove(level_removed_from_world_handle_);
        level_added_to_world_handle_.Reset();
        level_removed_from_world_handle_.Reset();

        FCoreDelegates::OnActorLabelChanged.Remove(actor_label_changed_handle_);
        actor_label_changed_handle_.Reset();
    }

    void USpStableNameEventHandler::actorLabelChangedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        UnrealUtils::requestUpdateStableName(actor);
    }

    void USpStableNameEventHandler::levelAddedToWorldHandler(ULevel* level, UWorld* world)
    {
        SP_ASSERT(level);
        SP_ASSERT(world);
        UnrealUtils::requestUpdateAllStableNameActors(world);
    }

    void USpStableNameEventHandler::levelRemovedFromWorldHandler(ULevel* level, UWorld* world)
    {
        SP_ASSERT(level);
        SP_ASSERT(world);
        UnrealUtils::requestUpdateAllStableNameActors(world);
    }

    void USpStableNameEventHandler::levelActorAddedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        UnrealUtils::requestAddStableNameActor(actor);
    }

    void USpStableNameEventHandler::levelActorFolderChangedHandler(const AActor* actor, FName name)
    {
        SP_ASSERT(actor);
        UnrealUtils::requestUpdateStableName(const_cast<AActor*>(actor));
    }

    void USpStableNameEventHandler::levelActorDeletedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        UnrealUtils::requestRemoveStableNameActor(actor);
    }
#endif
