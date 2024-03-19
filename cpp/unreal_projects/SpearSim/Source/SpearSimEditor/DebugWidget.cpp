//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSimEditor/DebugWidget.h"

#include <vector>

#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <Misc/CoreDelegates.h>
#include <Misc/Paths.h>

#include "SpCore/Log.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "UrdfRobot/UrdfRobotPawn.h"
#include "Vehicle/VehiclePawn.h"

ADebugWidget::ADebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();
}

ADebugWidget::~ADebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();

    DebugString = Unreal::toFString("");
    UrdfFile = Unreal::toFString("");
}

#if WITH_EDITOR
    void ADebugWidget::PostLoad()
    {
        AActor::PostLoad();

        actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &ADebugWidget::actorLabelChangedHandler);

        SP_ASSERT(GEngine);
        level_actor_folder_changed_handle_ = GEngine->OnLevelActorFolderChanged().AddUObject(this, &ADebugWidget::levelActorFolderChangedHandler);
    }

    void ADebugWidget::BeginDestroy()
    {
        AActor::BeginDestroy();

        // Need to check IsValid() because PostLoad() is not called for default objects, but BeginDestroy() is.

        if (level_actor_folder_changed_handle_.IsValid()) {
            SP_ASSERT(GEngine);
            GEngine->OnLevelActorFolderChanged().Remove(level_actor_folder_changed_handle_);
            level_actor_folder_changed_handle_.Reset();
        }

        if (actor_label_changed_handle_.IsValid()) {
            FCoreDelegates::OnActorLabelChanged.Remove(actor_label_changed_handle_);
            actor_label_changed_handle_.Reset();
        }
    }
#endif

void ADebugWidget::LoadConfig()
{
    AActor::LoadConfig();
}

void ADebugWidget::SaveConfig()
{
    AActor::SaveConfig();
}

void ADebugWidget::PrintDebugString()
{
    SP_LOG("DebugString: ", Unreal::toStdString(DebugString));
}

void ADebugWidget::SpawnVehiclePawn()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    std::vector<AVehiclePawn*> vehicle_pawns_list = Unreal::findActorsByType<AVehiclePawn>(world);
    std::string name_suffix = Std::toString(vehicle_pawns_list.size() + 1);
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName("vehicle_pawn_" + name_suffix);
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AVehiclePawn* vehicle_pawn = world->SpawnActor<AVehiclePawn>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(vehicle_pawn);
}

void ADebugWidget::SpawnUrdfRobotPawn()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    std::vector<AUrdfRobotPawn*> urdf_robot_pawns_list = Unreal::findActorsByType<AUrdfRobotPawn>(world);
    std::string name_suffix = Std::toString(urdf_robot_pawns_list.size() + 1);
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName("urdf_robot_pawn_" + name_suffix);
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AUrdfRobotPawn* urdf_robot_pawn = world->SpawnActor<AUrdfRobotPawn>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(urdf_robot_pawn);

    urdf_robot_pawn->UrdfFile = UrdfFile;
    urdf_robot_pawn->Initialize();
}

#if WITH_EDITOR
    void ADebugWidget::actorLabelChangedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableActorName(actor);
    }

    void ADebugWidget::levelActorFolderChangedHandler(const AActor* actor, FName name)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableActorName(actor);
    }
#endif
