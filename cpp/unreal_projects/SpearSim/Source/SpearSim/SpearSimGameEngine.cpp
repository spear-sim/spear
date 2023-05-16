//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSim/SpearSimGameEngine.h"

#include <string>

#include <CoreMinimal.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/GameEngine.h>
#include <Engine/World.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavModifierVolume.h>

#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

USpearSimGameEngine::USpearSimGameEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpearSimGameEngine::~USpearSimGameEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool USpearSimGameEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::string cmd_str = Unreal::toStdString(cmd);

    SP_LOG(cmd_str);

    if (cmd_str == "spear getNavigationData") {

        std::vector<ANavModifierVolume*> nav_modifier_volumes = Unreal::findActorsByType<ANavModifierVolume>(world);

        for (int i = 0; i < nav_modifier_volumes.size(); i++) {
            ANavModifierVolume* nav_modifier_volume = nav_modifier_volumes.at(i);
            ASSERT(nav_modifier_volume);

            FVector position = nav_modifier_volume->GetActorLocation();
            FRotator rotation = nav_modifier_volume->GetActorRotation();
            FVector scale = nav_modifier_volume->GetActorScale3D();

            SP_LOG("nav_modifier_volume->GetName():                 ", Unreal::toStdString(nav_modifier_volume->GetName()));
            SP_LOG("nav_modifier_volume->GetAreaClass()->GetName(): ", Unreal::toStdString(nav_modifier_volume->GetAreaClass()->GetName()));
            SP_LOG("position.X:                                     ", position.X);
            SP_LOG("position.Y:                                     ", position.Y);
            SP_LOG("position.Z:                                     ", position.Z);
            SP_LOG("rotation.Pitch:                                 ", rotation.Pitch);
            SP_LOG("rotation.Yaw:                                   ", rotation.Yaw);
            SP_LOG("rotation.Roll:                                  ", rotation.Roll);
            SP_LOG("scale.X:                                        ", scale.X);
            SP_LOG("scale.Y:                                        ", scale.Y);
            SP_LOG("scale.Z:                                        ", scale.Z);
            SP_LOG("nav_modifier_volume->Brush->Bounds.BoxExtent.X: ", nav_modifier_volume->Brush->Bounds.BoxExtent.X);
            SP_LOG("nav_modifier_volume->Brush->Bounds.BoxExtent.Y: ", nav_modifier_volume->Brush->Bounds.BoxExtent.Y);
            SP_LOG("nav_modifier_volume->Brush->Bounds.BoxExtent.Z: ", nav_modifier_volume->Brush->Bounds.BoxExtent.Z);
            SP_LOG();
        }

        return true;

    }
    else if (cmd_str == "spear setNavigationData") {

        std::vector<ANavModifierVolume*> nav_modifier_volumes = Unreal::findActorsByType<ANavModifierVolume>(world);

        UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
        ASSERT(nav_sys);

        ANavMeshBoundsVolume* nav_mesh_bounds_volume = Unreal::findActorByType<ANavMeshBoundsVolume>(world);
        ASSERT(nav_mesh_bounds_volume);

        nav_mesh_bounds_volume->SetActorScale3D(FVector(3.5f, 3.5f, 3.5f));
        nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds_volume);

        for (int i = 0; i < nav_modifier_volumes.size(); i++) {
            ANavModifierVolume* nav_modifier_volume = nav_modifier_volumes.at(i);
            ASSERT(nav_modifier_volume);

            nav_modifier_volume->SetActorScale3D(FVector(0.5f, 0.5f, 0.5f));
            nav_modifier_volume->RebuildNavigationData();
        }

        nav_sys->Build();

        return true;

    }
    else if (cmd_str == "spear resetNavigationData") {

        std::vector<ANavModifierVolume*> nav_modifier_volumes = Unreal::findActorsByType<ANavModifierVolume>(world);

        UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world);
        ASSERT(nav_sys);

        ANavMeshBoundsVolume* nav_mesh_bounds_volume = Unreal::findActorByType<ANavMeshBoundsVolume>(world);
        ASSERT(nav_mesh_bounds_volume);

        nav_mesh_bounds_volume->SetActorScale3D(FVector(7.0f, 7.0f, 2.0f));
        nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds_volume);

        for (int i = 0; i < nav_modifier_volumes.size(); i++) {
            ANavModifierVolume* nav_modifier_volume = nav_modifier_volumes.at(i);
            ASSERT(nav_modifier_volume);

            nav_modifier_volume->SetActorScale3D(FVector(1.0f, 1.0f, 1.0f));
            nav_modifier_volume->RebuildNavigationData();
        }

        nav_sys->Build();

        return true;

    }
    else if (cmd_str == "spear sampleNavigationData") {

        ARecastNavMesh* nav_mesh = Unreal::findActorByType<ARecastNavMesh>(world);
        ASSERT(nav_mesh);

        int num_spheres = 5000;
        static int j = 0;
        for (int i = 0; i < num_spheres; i++) {
            FVector spawn_location = nav_mesh->GetRandomPoint().Location;
            FRotator spawn_rotation = FRotator::ZeroRotator;
            FActorSpawnParameters actor_spawn_params;
            actor_spawn_params.Name = Unreal::toFName("DebugSphere_" + boost::lexical_cast<std::string>(i + j));
            actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            AStaticMeshActor* sphere_actor = world->SpawnActor<AStaticMeshActor>(spawn_location, spawn_rotation, actor_spawn_params);
            ASSERT(sphere_actor);

            sphere_actor->SetMobility(EComponentMobility::Type::Movable);
            sphere_actor->SetActorScale3D(FVector(0.2f, 0.2f, 0.2f));

            UStaticMesh* sphere_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString("/Engine/BasicShapes/Sphere.Sphere"));
            ASSERT(sphere_mesh);
            UMaterial* sphere_material = LoadObject<UMaterial>(nullptr, *Unreal::toFString("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
            ASSERT(sphere_material);

            UStaticMeshComponent* static_mesh_component = sphere_actor->GetStaticMeshComponent();
            ASSERT(static_mesh_component);

            static_mesh_component->SetStaticMesh(sphere_mesh);
            static_mesh_component->SetMaterial(0, sphere_material);
            static_mesh_component->SetCanEverAffectNavigation(false);
        }
        j = j + num_spheres;

        return true;

    }
    else if (cmd_str == "spear callPython") {
        GEngine->Exec(world, TEXT("py _hello.py"));
        return true;
    }

    return UGameEngine::Exec(world, cmd, output_device);
}
