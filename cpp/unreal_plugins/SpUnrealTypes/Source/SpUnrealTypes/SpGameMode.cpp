//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpGameMode.h"

#include <Containers/UnrealString.h> // FString
#include <Engine/Engine.h>           // GEngine
#include <GameFramework/GameModeBase.h>
#include <GameFramework/Pawn.h>
#include <GameFramework/PlayerController.h>
#include <HAL/Platform.h>            // int32, uint64
#include <Kismet/GameplayStatics.h>

#include <Math/Color.h>
#include <Math/ColorList.h>
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpPlayerController.h"
#include "SpUnrealTypes/SpSpectatorPawn.h"
#include "SpUnrealTypes/SpDebugCameraController.h"

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
    GEngine->AddOnScreenDebugMessage(Key, TimeToDisplay, GColorList.GetFColorByName(*Unreal::toFString(display_color_str)), Unreal::toFString(debug_message_str));
}

void ASpGameMode::SpMountPak(const FString& PakFile, int32 PakOrder) const
{
    IPakFile* pak_file = FCoreDelegates::MountPak.Execute(PakFile, PakOrder);
    SP_ASSERT(pak_file);
}

void ASpGameMode::SpOpenLevel(const FString& LevelName) const
{
    UGameplayStatics::OpenLevel(GetWorld(), Unreal::toFName(Unreal::toStdString(LevelName)));
}

void ASpGameMode::SpToggleDebugCamera()
{
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
