//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>

#include <Containers/Array.h>
#include <Containers/UnrealString.h>        // FString
#include <Engine/LevelStreamingDynamic.h>
#include <LevelSequenceDirector.h>
#include <LevelSequencePlayer.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>           // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY
#include <UObject/WeakObjectPtrTemplates.h> // TWeakObjectPtr

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
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
    void StopHandler() // not intended to be BlueprintCallable
    {
        SP_LOG_CURRENT_FUNCTION();
        terminate();
    }

    UFUNCTION()
    void FinishedHandler() // not intended to be BlueprintCallable
    {
        SP_LOG_CURRENT_FUNCTION();
        terminate();
    }

    void initialize()
    {
        if (Player && GetWorld()) {
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
    }

    void terminate()
    {
        if (Player && GetWorld()) {
            // Iterate over level_instances_ rather than LevelInstancePaths, because level instances that failed to
            // load in initialize() are not stored in level_instances_.
            for (auto& [level_instance_path_str, level_instance_weak_ptr] : level_instances_) {
                SP_LOG("Unloading: ", level_instance_path_str);
                ULevelStreamingDynamic* level_instance = Unreal::toPtr(level_instance_weak_ptr);
                SP_ASSERT(level_instance);
                level_instance->SetIsRequestingUnloadAndRemoval(true);
            }
            level_instances_.clear();

            Player->OnStop.RemoveDynamic(this, &USpLevelSequenceDirector::StopHandler);
            Player->OnFinished.RemoveDynamic(this, &USpLevelSequenceDirector::FinishedHandler);
        }
    }

    std::map<std::string, TWeakObjectPtr<ULevelStreamingDynamic>> level_instances_;
};
