//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright Epic Games, Inc. All Rights Reserved.
//

#include "SpearSimEditor/SpearSimEditorUnrealEdEngine.h"

#include <map>
#include <string>
#include <vector>

#include <PhysicsEngine/PhysicsConstraintComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Editor/UnrealEdEngine.h>
#include <GameFramework/Actor.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
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

    std::vector<std::string> cmd_list = Std::tokenize(cmd_str, " ");

    if (cmd_list.at(0) == "print") {
        SP_LOG(cmd_list.at(1));
        return true;
    } 
    else if (cmd_list.at(0) == "printAllStaticMeshComponents") {
        
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});
        TArray<UStaticMeshComponent*> sm_comps;
        for (auto& element: all_actors_name_ref_map) {
            element.second->GetComponents<UStaticMeshComponent*>(sm_comps);
            SP_LOG(element.first, "......");
            for (auto& comp : sm_comps) {
                SP_LOG("    ", Unreal::toStdString(comp->GetName()));
            }
        }
        return true;
    }
    else if (cmd_list.at(0) == "printAllPhysicsConstraintComponents") {

        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});
        TArray<UPhysicsConstraintComponent*> sm_comps;
        for (auto& element : all_actors_name_ref_map) {
            element.second->GetComponents<UPhysicsConstraintComponent*>(sm_comps);
            SP_LOG(element.first, "......");
            for (auto& comp : sm_comps) {
                SP_LOG("    ", Unreal::toStdString(comp->GetName()));
            }
        }
        return true;
    }
    else if (cmd_list.at(0) == "getActorLocation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});
        
        if (all_actors_name_ref_map.count(cmd_list.at(1))) {

            FVector location = all_actors_name_ref_map.at(cmd_list.at(1))->GetActorLocation(); // returns the location of the root component of this AActor

            SP_LOG("actor ", cmd_list.at(1), " location is ", location.X, " ,", location.Y, ", ", location.Z);
            
            TArray<UStaticMeshComponent*> sm_comps;
            all_actors_name_ref_map.at(cmd_list.at(1))->GetComponents<UStaticMeshComponent*>(sm_comps);
            for (auto& comp : sm_comps) {
                location = comp->GetComponentLocation();
                SP_LOG("    ", Unreal::toStdString(comp->GetName()), " location is ", location.X, " ,", location.Y, ", ", location.Z);
            }
        }

        return true;
    }
    else if (cmd_list.at(0) == "getActorRotation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});

        if (all_actors_name_ref_map.count(cmd_list.at(1))) {

            FRotator rotation = all_actors_name_ref_map.at(cmd_list.at(1))->GetActorRotation(); // returns the rotation of the root component of this AActor

            SP_LOG("actor ", cmd_list.at(1), " rotation is ", rotation.Pitch, " ,", rotation.Yaw, ", ", rotation.Roll);

            TArray<UStaticMeshComponent*> sm_comps;
            all_actors_name_ref_map.at(cmd_list.at(1))->GetComponents<UStaticMeshComponent*>(sm_comps);
            for (auto& comp : sm_comps) {
                rotation = comp->GetComponentRotation();
                SP_LOG("    ", Unreal::toStdString(comp->GetName()), " rotation is ", rotation.Pitch, " ,", rotation.Yaw, ", ", rotation.Roll);
            }
        }

        return true;
    }
    else if (cmd_list.at(0) == "setActorLocation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});

        if (all_actors_name_ref_map.count(cmd_list.at(1))) {
            FVector location = { std::stof(cmd_list.at(2)), std::stof(cmd_list.at(3)), std::stof(cmd_list.at(4)) };
            bool sweep = false;
            FHitResult* hit_result = nullptr;
            all_actors_name_ref_map.at(cmd_list.at(1))->SetActorLocation(location, sweep, hit_result, ETeleportType::TeleportPhysics);
        }

        return true;
    }
    else if (cmd_list.at(0) == "setActorRotation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});

        if (all_actors_name_ref_map.count(cmd_list.at(1))) {
            FRotator rotation = { std::stof(cmd_list.at(2)), std::stof(cmd_list.at(3)), std::stof(cmd_list.at(4)) };
            bool sweep = false;
            FHitResult* hit_result = nullptr;
            all_actors_name_ref_map.at(cmd_list.at(1))->SetActorRotation(rotation, ETeleportType::TeleportPhysics);
        }

        return true;
    }
    else if (cmd_list.at(0) == "setComponentLocation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});

        if (all_actors_name_ref_map.count(cmd_list.at(1))) {
            
            TArray<UStaticMeshComponent*> sm_comps;
            all_actors_name_ref_map.at(cmd_list.at(1))->GetComponents<UStaticMeshComponent*>(sm_comps);

            std::map<std::string, UStaticMeshComponent*> all_comps_name_ref_map;
            for (auto& comp : sm_comps) {
                all_comps_name_ref_map[Unreal::toStdString(comp->GetName())] = comp;
            }

            if (all_comps_name_ref_map.at(cmd_list.at(2))) {
                FVector location = { std::stof(cmd_list.at(3)), std::stof(cmd_list.at(4)), std::stof(cmd_list.at(5)) };
                bool sweep = false;
                FHitResult* hit_result = nullptr;
                all_comps_name_ref_map.at(cmd_list.at(2))->SetWorldLocation(location, sweep, hit_result, ETeleportType::TeleportPhysics);
            }
        }

        return true;
    }
    else if (cmd_list.at(0) == "setComponentRotation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});

        if (all_actors_name_ref_map.count(cmd_list.at(1))) {

            TArray<UStaticMeshComponent*> sm_comps;
            all_actors_name_ref_map.at(cmd_list.at(1))->GetComponents<UStaticMeshComponent*>(sm_comps);

            std::map<std::string, UStaticMeshComponent*> all_comps_name_ref_map;
            for (auto& comp : sm_comps) {
                all_comps_name_ref_map[Unreal::toStdString(comp->GetName())] = comp;
            }

            if (all_comps_name_ref_map.at(cmd_list.at(2))) {
                FRotator rotation = { std::stof(cmd_list.at(3)), std::stof(cmd_list.at(4)), std::stof(cmd_list.at(5)) };
                bool sweep = false;
                FHitResult* hit_result = nullptr;
                all_comps_name_ref_map.at(cmd_list.at(2))->SetWorldRotation(rotation, sweep, hit_result, ETeleportType::TeleportPhysics);
            }
        }

        return true;
    }
    else if (cmd_list.at(0) == "setPhysicsConstraintLocation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});

        if (all_actors_name_ref_map.count(cmd_list.at(1))) {

            TArray<UPhysicsConstraintComponent*> pc_comps;
            all_actors_name_ref_map.at(cmd_list.at(1))->GetComponents<UPhysicsConstraintComponent*>(pc_comps);

            std::map<std::string, UPhysicsConstraintComponent*> all_comps_name_ref_map;
            for (auto& comp : pc_comps) {
                all_comps_name_ref_map[Unreal::toStdString(comp->GetName())] = comp;
            }

            if (all_comps_name_ref_map.at(cmd_list.at(2))) {
                FVector location = { std::stof(cmd_list.at(3)), std::stof(cmd_list.at(4)), std::stof(cmd_list.at(5)) };
                bool sweep = false;
                FHitResult* hit_result = nullptr;
                all_comps_name_ref_map.at(cmd_list.at(2))->SetWorldLocation(location, sweep, hit_result, ETeleportType::TeleportPhysics);
            }
        }

        return true;
    }
    else if (cmd_list.at(0) == "setComponentRelativeLocation") {
        std::map<std::string, AActor*> all_actors_name_ref_map = Unreal::findActorsByTagAllAsMap(world, {});

        if (all_actors_name_ref_map.count(cmd_list.at(1))) {

            TArray<UStaticMeshComponent*> sm_comps;
            all_actors_name_ref_map.at(cmd_list.at(1))->GetComponents<UStaticMeshComponent*>(sm_comps);

            std::map<std::string, UStaticMeshComponent*> all_comps_name_ref_map;
            for (auto& comp : sm_comps) {
                all_comps_name_ref_map[Unreal::toStdString(comp->GetName())] = comp;
            }

            if (all_comps_name_ref_map.at(cmd_list.at(2))) {
                FVector location = { std::stof(cmd_list.at(3)), std::stof(cmd_list.at(4)), std::stof(cmd_list.at(5)) };
                bool sweep = false;
                FHitResult* hit_result = nullptr;
                all_comps_name_ref_map.at(cmd_list.at(2))->SetRelativeLocation(location, sweep, hit_result, ETeleportType::TeleportPhysics);
            }
        }

        return true;
    }

    return UUnrealEdEngine::Exec(world, cmd, output_device);
}
