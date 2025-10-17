//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>

#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/LevelStreamingDynamic.h>
#include <LevelSequenceDirector.h>
#include <LevelSequencePlayer.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpLevelSequenceDirector.generated.h"

UCLASS()
class USpLevelSequenceDirector : public ULevelSequenceDirector
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")    
    void OnCreatedImpl()
    {
        SP_LOG_CURRENT_FUNCTION();
        initialize();
    }

    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category="SPEAR")
    TArray<FString> LevelInstancePaths;

private:
    UFUNCTION()
    void StopHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        terminate();
    }

    UFUNCTION()
    void FinishedHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        terminate();
    }

    void initialize()
    {
        SP_ASSERT(GetWorld());
        SP_ASSERT(Player);

        Player->OnStop.AddDynamic(this, &USpLevelSequenceDirector::StopHandler);
        Player->OnFinished.AddDynamic(this, &USpLevelSequenceDirector::FinishedHandler);

        for (auto& level_instance_path : LevelInstancePaths) {
            std::string level_instance_path_str = Unreal::toStdString(level_instance_path);
            SP_LOG("Loading: ", level_instance_path_str);

            bool success = false;
            ULevelStreamingDynamic* level_instance = ULevelStreamingDynamic::LoadLevelInstance(GetWorld(), level_instance_path, FVector::ZeroVector, FRotator::ZeroRotator, success);
            if (success) {
                SP_ASSERT(level_instance);
                Std::insert(level_instances_, level_instance_path_str, level_instance);
            } else {
                SP_ASSERT(!level_instance);
                SP_LOG("WARNING: Load unsuccessful: ", level_instance_path_str);
            }
        }
    }

    void terminate()
    {
        SP_ASSERT(Player);

        for (auto& level_instance_path : LevelInstancePaths) {
            std::string level_instance_path_str = Unreal::toStdString(level_instance_path);
            SP_LOG("Unloading: ", level_instance_path_str);

            bool success = false;
            ULevelStreamingDynamic* level_instance = level_instances_.at(level_instance_path_str);
            SP_ASSERT(level_instance);
            level_instance->SetIsRequestingUnloadAndRemoval(true);
            Std::remove(level_instances_, level_instance_path_str);
        }

        Player->OnStop.RemoveDynamic(this, &USpLevelSequenceDirector::StopHandler);
        Player->OnFinished.RemoveDynamic(this, &USpLevelSequenceDirector::FinishedHandler);
    }

    std::map<std::string, ULevelStreamingDynamic*> level_instances_;
};
