//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/Actor.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpPauseWidget.generated.h"

UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpPauseWidget : public AActor
{
    GENERATED_BODY()
public: 
    ASpPauseWidget();
    ~ASpPauseWidget();

    #if WITH_EDITOR // defined in an auto-generated header
        // AActor interface
        void Tick(float delta_time) override;
    #endif

private:

    // Calling UGameplayStatics::SetGamePaused() doesn't synchronize with the default play/pause button in
    // the editor. So we provide a custom button and a custom read-only property to update and visualize the
    // engine-level (as opposed to the editor-level) pause state of the game. All of this functionality is
    // implemented by interacting directly with UGameplayStatics, and is therefore guaranteed to be correctly
    // synchronized with the engine-level state.

    #if WITH_EDITOR // defined in an auto-generated header
        UFUNCTION(CallInEditor, Category="SPEAR")
        void ToggleGamePaused();
    #endif

    #if WITH_EDITORONLY_DATA // defined in an auto-generated header
        UPROPERTY(VisibleAnywhere, Category="SPEAR");
        bool bIsGamePaused = false;
    #endif
};
