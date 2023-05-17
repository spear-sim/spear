//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSimEditor/SpearSimEditorUnrealEdEngine.h"

#include <string>

#include <Builders/CubeBuilder.h>
#include <CoreMinimal.h>
#include <Engine/StaticMeshActor.h>
#include <Editor/UnrealEdEngine.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavModifierVolume.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"

USpearSimEditorUnrealEdEngine::USpearSimEditorUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpearSimEditorUnrealEdEngine::~USpearSimEditorUnrealEdEngine()
{
    SP_LOG_CURRENT_FUNCTION();
}

bool USpearSimEditorUnrealEdEngine::Exec(UWorld* world, const TCHAR* cmd, FOutputDevice& output_device)
{
    std::string cmd_str = Unreal::toStdString(cmd);

    SP_LOG(cmd_str);
    
    if (cmd_str == "spear getNavigationData") {

        std::vector<ANavModifierVolume*> nav_modifier_volumes = Unreal::findActorsByType<ANavModifierVolume>(world);

        for (int i = 0; i < nav_modifier_volumes.size(); i++) {
            ANavModifierVolume* nav_modifier_volume = nav_modifier_volumes.at(i);
            ASSERT(nav_modifier_volume);

            UCubeBuilder* cube_builder = dynamic_cast<UCubeBuilder*>(nav_modifier_volume->BrushBuilder.Get());
            ASSERT(cube_builder);

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
            SP_LOG("cube_builder->X:                                ", cube_builder->X);
            SP_LOG("cube_builder->Y:                                ", cube_builder->Y);
            SP_LOG("cube_builder->Z:                                ", cube_builder->Z);
            SP_LOG();
        }

        return true;

    } else if (cmd_str == "spear setNavigationData") {

        std::vector<ANavModifierVolume*> nav_modifier_volumes = Unreal::findActorsByType<ANavModifierVolume>(world);

        for (int i = 0; i < nav_modifier_volumes.size(); i++) {
            ANavModifierVolume* nav_modifier_volume = nav_modifier_volumes.at(i);
            ASSERT(nav_modifier_volume);

            UCubeBuilder* cube_builder = dynamic_cast<UCubeBuilder*>(nav_modifier_volume->BrushBuilder.Get());
            ASSERT(cube_builder);

            cube_builder->X = 11.0f;
            cube_builder->Y = 11.0f;
            cube_builder->Z = 11.0f;
            cube_builder->Build(world, nav_modifier_volume);
            nav_modifier_volume->SetActorLocation(FVector(i*40.0f, i*40.0f, 0.0f));
            nav_modifier_volume->SetActorRotation(FRotator(i*20.0f, i*20.0f, i*20.0f));
            nav_modifier_volume->SetActorScale3D(FVector(i+1.0f, i+1.0f, i+1.0f));
            nav_modifier_volume->PostEditImport();
        }

        return true;

    } else if (cmd_str == "spear resetNavigationData") {

        std::vector<ANavModifierVolume*> nav_modifier_volumes = Unreal::findActorsByType<ANavModifierVolume>(world);

        std::vector brush_sizes = {50.0f, 100.0f, 200.0f};
        for (int i = 0; i < nav_modifier_volumes.size(); i++) {
            ANavModifierVolume* nav_modifier_volume = nav_modifier_volumes.at(i);
            ASSERT(nav_modifier_volume);

            UCubeBuilder* cube_builder = dynamic_cast<UCubeBuilder*>(nav_modifier_volume->BrushBuilder.Get());
            ASSERT(cube_builder);

            cube_builder->X = brush_sizes.at(i);
            cube_builder->Y = brush_sizes.at(i);
            cube_builder->Z = brush_sizes.at(i);
            cube_builder->Build(world, nav_modifier_volume);
            nav_modifier_volume->SetActorLocation(FVector(0.0f, 0.0f, 0.0f));
            nav_modifier_volume->SetActorRotation(FRotator(0.0f, 0.0f, 0.0f));
            nav_modifier_volume->SetActorScale3D(FVector(1.0f, 1.0f, 1.0f));
            nav_modifier_volume->PostEditImport();
        }

        return true;

    } else if (cmd_str == "spear sampleNavigationData") {

        ARecastNavMesh* nav_mesh = Unreal::findActorByType<ARecastNavMesh>(world);
        ASSERT(nav_mesh);

        int num_spheres = 5000;
        static int j = 0;
        for (int i = 0; i < num_spheres; i++) {
            FVector spawn_location = nav_mesh->GetRandomPoint().Location;
            FRotator spawn_rotation = FRotator::ZeroRotator;
            FActorSpawnParameters actor_spawn_params;
            actor_spawn_params.Name = Unreal::toFName("DebugSphere_" + boost::lexical_cast<std::string>(i+j));
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

    } else if (cmd_str == "spear callPython") {
        GEngine->Exec(world, TEXT("py _hello.py"));
        return true;
    }

    return UUnrealEdEngine::Exec(world, cmd, output_device);
}
