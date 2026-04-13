//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpStableNameManager.h"

#include <string>
#include <vector>

#include <Containers/UnrealString.h>     // FString
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Misc/CoreDelegates.h>
#include <UObject/NameTypes.h>           // FName
#include <UObject/Package.h>
#include <UObject/SoftObjectPath.h>

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

//
// ASpStableNameManager
//

void ASpStableNameManager::BeginPlay()
{
    AActor::BeginPlay();

    UWorld* world = GetWorld();
    if (!world) {
        return;
    }

    SP_ASSERT(!s_worlds_with_stable_name_manager_.contains(world));
    Std::insert(s_worlds_with_stable_name_manager_, world);
}

void ASpStableNameManager::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    AActor::EndPlay(end_play_reason);

    UWorld* world = GetWorld();
    if (!world) {
        return;
    }

    SP_ASSERT(s_worlds_with_stable_name_manager_.contains(world));
    Std::remove(s_worlds_with_stable_name_manager_, world);
}

#if WITH_EDITOR // defined in an auto-generated header
    void ASpStableNameManager::PostActorCreated()
    {
        AActor::PostActorCreated();
        requestAddOrUpdateAllActors();
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
    void ASpStableNameManager::requestAddOrUpdateAllActors()
    {
        UWorld* world = GetWorld();
        if (!world) {
            return;
        }

        if (world->IsEditorWorld() && !world->IsPreviewWorld() && !world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
            StableNames.Empty();
            std::vector<AActor*> actors = UnrealUtils::findActors(GetWorld());
            for (auto actor : actors) {
                if (actor != this) {
                    requestAddOrUpdateActor(actor);
                }
            }
        }
    }

    void ASpStableNameManager::requestAddOrUpdateActor(AActor* actor)
    {
        SP_ASSERT(actor);

        UWorld* world = GetWorld();
        if (!world) {
            return;
        }

        SP_ASSERT(world == actor->GetWorld());

        if (world->IsEditorWorld() && !world->IsPreviewWorld() && !world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
            FString id = Unreal::toFString(getStableIdString(actor));
            FString stable_name = Unreal::toFString(UnrealUtils::resolveStableName(actor));
            if (StableNames.Contains(id)) {
                StableNames[id] = stable_name;
            } else {
                StableNames.Add(id, stable_name);
            }
        }
    }

    void ASpStableNameManager::requestAddActor(AActor* actor)
    {
        UWorld* world = GetWorld();
        if (!world) {
            return;
        }

        SP_ASSERT(world == actor->GetWorld());

        if (world->IsEditorWorld() && !world->IsPreviewWorld() && !world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
            FString id = Unreal::toFString(getStableIdString(actor));
            FString stable_name = Unreal::toFString(UnrealUtils::resolveStableName(actor));
            SP_ASSERT(!StableNames.Contains(id));
            StableNames.Add(id, stable_name);
        }
    }

    void ASpStableNameManager::requestUpdateActor(AActor* actor)
    {
        UWorld* world = GetWorld();
        if (!world) {
            return;
        }

        SP_ASSERT(world == actor->GetWorld());

        if (world->IsEditorWorld() && !world->IsPreviewWorld() && !world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
            FString id = Unreal::toFString(getStableIdString(actor));
            FString stable_name = Unreal::toFString(UnrealUtils::resolveStableName(actor));
            SP_ASSERT(StableNames.Contains(id));
            StableNames[id] = stable_name;
        }
    }

    void ASpStableNameManager::requestRemoveActor(AActor* actor)
    {
        SP_ASSERT(actor);

        UWorld* world = GetWorld();
        if (!world) {
            return;
        }

        SP_ASSERT(world == actor->GetWorld());

        if (world->IsEditorWorld() && !world->IsPreviewWorld() && !world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
            FString id = Unreal::toFString(getStableIdString(actor));
            SP_ASSERT(StableNames.Contains(id));
            StableNames.Remove(id);
        }
    }

#endif

std::string ASpStableNameManager::getStableIdString(const AActor* actor)
{
    SP_ASSERT(actor);

    UWorld* world = actor->GetWorld();
    SP_ASSERT(world);
    SP_ASSERT(!world->IsPreviewWorld());
    SP_ASSERT(GEngine->GetWorldContextFromWorld(world));

    FSoftObjectPath soft_object_path = FSoftObjectPath(actor);

    FString asset_path_string = soft_object_path.GetAssetPathString();
    FString sub_path_string = soft_object_path.GetSubPathString();
    FString non_pie_asset_path_string;

    if (world->IsEditorWorld() && world->IsGameWorld()) {
        FString pie_prefix = UWorld::BuildPIEPackagePrefix(actor->GetOutermost()->GetPIEInstanceID());
        non_pie_asset_path_string = UWorld::StripPIEPrefixFromPackageName(asset_path_string, pie_prefix);
    } else {
        non_pie_asset_path_string = asset_path_string;
    }

    return Unreal::toStdString(non_pie_asset_path_string + ":" + sub_path_string);
}

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

        UWorld* world = GetWorld();
        if (!world) {
            return;
        }

        if (world->IsEditorWorld() && !world->IsPreviewWorld() && !world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
            StableName = Unreal::toFString(UnrealUtils::resolveStableName(actor));
        }
    }
#endif

//
// USpStableNameEventHandler
//

USpStableNameEventHandler::USpStableNameEventHandler()
{
    #if WITH_EDITOR // defined in an auto-generated header
        post_engine_init_handle_ = FCoreDelegates::OnPostEngineInit.AddUObject(this, &USpStableNameEventHandler::postEngineInitHandler);
        engine_pre_exit_handle_  = FCoreDelegates::OnEnginePreExit.AddUObject(this, &USpStableNameEventHandler::enginePreExitHandler);
    #endif
}

USpStableNameEventHandler::~USpStableNameEventHandler()
{
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
        // level can be null
        SP_ASSERT(world);
        UnrealUtils::requestAddOrUpdateAllStableNameActors(world);
    }

    void USpStableNameEventHandler::levelRemovedFromWorldHandler(ULevel* level, UWorld* world)
    {
        // level can be null
        SP_ASSERT(world);
        UnrealUtils::requestAddOrUpdateAllStableNameActors(world);
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
