//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpGameMode.h"

#include <string>

#include <Containers/UnrealString.h> // FString
#include <Engine/Engine.h>           // GEngine
#include <Engine/LevelStreamingDynamic.h>
#include <Engine/World.h>            // FActorSpawnParameters
#include <GameFramework/GameModeBase.h>
#include <GameFramework/Pawn.h>
#include <GameFramework/PlayerController.h>
#include <HAL/Platform.h>            // int32, uint64
#include <Kismet/GameplayStatics.h>
#include <Math/Color.h>
#include <Math/ColorList.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpDebugCameraController.h"
#include "SpUnrealTypes/SpPlayerController.h"
#include "SpUnrealTypes/SpSpectatorPawn.h"

class IPakFile;

ASpGameMode::ASpGameMode()
{
    SP_LOG_CURRENT_FUNCTION();

    DefaultPawnClass = ASpSpectatorPawn::StaticClass();
    PlayerControllerClass = ASpPlayerController::StaticClass();
}

ASpGameMode::~ASpGameMode()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpGameMode::PostLogin(APlayerController* new_player)
{
    SP_LOG_CURRENT_FUNCTION();

    AGameModeBase::PostLogin(new_player);

    SP_ASSERT(GetWorld());

    // Set the stable name for the DefaultPawnClass instance so we can find it later. We only do this if the
    // pawn is non-null (it is possible for pawn to be null if we press "Simulate" in the editor), and if the
    // pawn has a USpStableNameComponent.
    APawn* pawn = new_player->GetPawn();
    if (pawn && Unreal::hasStableName(pawn)) {
        Unreal::setStableName(pawn, "__SP_DEFAULT_PAWN__");
    }

    // Spawn custom debug camera.
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Instigator = new_player->GetInstigator();
    sp_debug_camera_controller_ = GetWorld()->SpawnActor<ASpDebugCameraController>(actor_spawn_parameters);
}

void ASpGameMode::SpAddOnScreenDebugMessage(uint64 Key, float TimeToDisplay, const FString& DisplayColor, const FString& DebugMessage) const
{
    SP_ASSERT(GEngine);

    // Note that GEngine->AddOnScreenDebugMessage(...) is only available when the game is running, either in
    // standalone mode or in play-in-editor mode. But in practice this is not an issue, because UFUNTION(Exec)
    // methods only execute when the game is running anyway.
    std::string display_color_str = Unreal::toStdString(DisplayColor.ToLower());
    std::string debug_message_str = SP_LOG_GET_PREFIX() + Unreal::toStdString(DebugMessage);
    GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, GColorList.GetFColorByName(Unreal::toTCharPtr(display_color_str)), Unreal::toFString(debug_message_str));
}

void ASpGameMode::SpMountPak(const FString& PakFile) const
{
    if (FCoreDelegates::MountPak.IsBound()) {
        int32 pak_order = 0;
        IPakFile* pak_file = FCoreDelegates::MountPak.Execute(PakFile, pak_order);
        SP_ASSERT(pak_file);
    } else {
        SP_LOG("WARNING: FCoreDelegates::MountPak delegate is not bound.");
    }
}

void ASpGameMode::SpUnmountPak(const FString& PakFile) const
{
    if (FCoreDelegates::OnUnmountPak.IsBound()) {
        bool success = FCoreDelegates::OnUnmountPak.Execute(PakFile);
        SP_ASSERT(success);
    } else {
        SP_LOG("WARNING: FCoreDelegates::OnUnmountPak delegate is not bound.");        
    }
}

void ASpGameMode::SpLoadStreamLevel(const FString& LevelName) const
{
    SP_ASSERT(GetWorld());
    bool make_visible_after_load = true;
    bool should_block_on_load = true;
    UGameplayStatics::LoadStreamLevel(GetWorld(), Unreal::toFName(Unreal::toStdString(LevelName)), make_visible_after_load, should_block_on_load, FLatentActionInfo());
}

void ASpGameMode::SpUnloadStreamLevel(const FString& LevelName) const
{
    SP_ASSERT(GetWorld());
    bool should_block_on_unload = true;
    UGameplayStatics::UnloadStreamLevel(GetWorld(), Unreal::toFName(Unreal::toStdString(LevelName)), FLatentActionInfo(), should_block_on_unload);
}

void ASpGameMode::SpLoadLevelInstance(const FString& LevelName)
{
    SP_ASSERT(GetWorld());
    std::string level_name = Unreal::toStdString(LevelName);
    SP_LOG("Loading: ", level_name);

    bool success = false;
    ULevelStreamingDynamic* level_instance = ULevelStreamingDynamic::LoadLevelInstance(GetWorld(), LevelName, FVector::ZeroVector, FRotator::ZeroRotator, success);

    if (success) {
        SP_ASSERT(level_instance);
        Std::insert(level_instances_, level_name, level_instance);
    } else {
        SP_ASSERT(!level_instance);
        SP_LOG("WARNING: Load unsuccessful: ", level_name);
    }
}

void ASpGameMode::SpUnloadLevelInstance(const FString& LevelName)
{
    SP_ASSERT(GetWorld());
    std::string level_name = Unreal::toStdString(LevelName);
    SP_LOG("Unloading: ", level_name);

    if (!Std::containsKey(level_instances_, level_name)) {
        SP_LOG("WARNING: Unload unsuccessful: ", level_name);
        return;
    }

    bool success = false;
    ULevelStreamingDynamic* level_instance = level_instances_.at(level_name);
    SP_ASSERT(level_instance);
    level_instance->SetIsRequestingUnloadAndRemoval(true);
    Std::remove(level_instances_, level_name);
}

void ASpGameMode::SpOpenLevel(const FString& LevelName) const
{
    UGameplayStatics::OpenLevel(GetWorld(), Unreal::toFName(Unreal::toStdString(LevelName)));
}

void ASpGameMode::SpToggleDebugCamera()
{
    SP_ASSERT(GetWorld());

    APlayerController* player_controller = GetWorld()->GetFirstPlayerController();
    SP_ASSERT(player_controller);

    // currently enabled so disable
    if (sp_debug_camera_controller_->OriginalPlayer && sp_debug_camera_controller_->OriginalControllerRef) {
        SP_ASSERT(!player_controller->Player);
        sp_debug_camera_controller_->OriginalPlayer->SwitchController(sp_debug_camera_controller_->OriginalControllerRef);
        sp_debug_camera_controller_->OnDeactivate(sp_debug_camera_controller_->OriginalControllerRef);
        SP_ASSERT(player_controller->Player);

    // currently disabled (and the default debug camera is disabled) so enable
    } else if (player_controller->Player) {
        sp_debug_camera_controller_->OnActivate(player_controller);
        player_controller->Player->SwitchController(sp_debug_camera_controller_);
         SP_ASSERT(!player_controller->Player);
    }
}
