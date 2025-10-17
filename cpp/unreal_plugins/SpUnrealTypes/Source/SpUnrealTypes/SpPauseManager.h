//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/Actor.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpPauseManager.generated.h"

UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpPauseManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpPauseManager();
    ~ASpPauseManager() override;

    // AActor interface
    void Tick(float delta_time) override;

private:
    // Calling UGameplayStatics::SetGamePaused() doesn't synchronize with the default play/pause button in
    // the editor. So we provide a custom button and a custom read-only property to update and visualize the
    // engine-level (as opposed to the editor-level) pause state of the game. All of this functionality is
    // implemented by interacting directly with UGameplayStatics, and is therefore guaranteed to be correctly
    // synchronized with the engine-level state.
    UFUNCTION(CallInEditor, Category="SPEAR")
    void ToggleGamePaused();

    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    bool bIsGamePaused = false;
};
