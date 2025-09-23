//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpGameMode.h"

#include <Containers/UnrealString.h> // FString
#include <Engine/Engine.h>           // GEngine
#include <GameFramework/GameModeBase.h>
#include <GameFramework/Pawn.h>
#include <GameFramework/PlayerController.h>
#include <HAL/Platform.h>            // uint64
#include <Math/Color.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpPlayerController.h"
#include "SpUnrealTypes/SpSpectatorPawn.h"

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
    AGameModeBase::PostLogin(new_player);

    // Set the stable name for the DefaultPawnClass instance so we can find it later. We only do this if the
    // pawn is non-null (it is possible for pawn to be null if we press "Simulate" in the editor), and if the
    // pawn has a USpStableNameComponent.
    APawn* pawn = new_player->GetPawn();
    if (pawn && Unreal::hasStableName(pawn)) {
        Unreal::setStableName(pawn, "__SP_DEFAULT_PAWN__");
    }
}

void ASpGameMode::SpAddOnScreenDebugMessage(float display_time, FString message)
{
    SP_ASSERT(GEngine);

    // Note that GEngine->AddOnScreenDebugMessage(...) is only available when the game is running, either in
    // standalone mode or in play-in-editor mode. But in practice this is not an issue, because UFUNTION(Exec)
    // methods only execute when the game is running anyway.
    uint64 key              = -1;
    FColor color            = FColor::Yellow;
    std::string message_str = SP_LOG_GET_PREFIX() + Unreal::toStdString(message);
    GEngine->AddOnScreenDebugMessage(key, display_time, color, Unreal::toFString(message_str));
}
