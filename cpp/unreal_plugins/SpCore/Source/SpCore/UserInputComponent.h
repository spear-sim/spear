//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h> // ELevelTick
#include <UObject/ObjectMacros.h>   // GENERATED_BODY, UCLASS

#include "UserInputComponent.generated.h"

class UInputComponent;
struct FActorComponentTickFunction;

// We need meta = (BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API UUserInputComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    UUserInputComponent();
    ~UUserInputComponent();

    // USceneComponent interface
    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override;

    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="Enable Handling of User Input");
    bool bEnableHandleUserInput = false;

    // Must be called in BeginPlay() or later, because GetWorld() needs to be valid.
    void subscribeToUserInputs(const std::vector<std::string>& user_inputs);

    // Set by the code using this class to specify what happens when user input (e.g., keyboard or mouse input) is received.
    void setHandleUserInputFunc(const std::function<void(const std::string&, float)>& handle_user_input_func);

private:
    struct UserInputDesc
    {
        std::string key_;
        std::string axis_;
        float scale_ = 1.0f;
        float threshold_ = 1.0f;
    };

    std::vector<UserInputDesc> user_input_descs_;
    std::function<void(const std::string&, float)> handle_user_input_func_;

    UInputComponent* input_component_ = nullptr;
};
