//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <string>
#include <map>
#include <vector>

#include <Components/SceneComponent.h>
#include <Engine/EngineBaseTypes.h>    // ELevelTick
#include <GameFramework/PlayerInput.h> // FInputAxisKeyMapping
#include <HAL/Platform.h>              // SPUNREALTYPES_API
#include <UObject/NameTypes.h>         // FName
#include <UObject/ObjectMacros.h>      // GENERATED_BODY, UCLASS

#include "SpUserInputComponent.generated.h"

class UInputComponent;
struct FActorComponentTickFunction;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPUNREALTYPES_API USpUserInputComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpUserInputComponent();
    ~USpUserInputComponent() override;

    // USceneComponent interface
    void TickComponent(float delta_time, ELevelTick level_tick, FActorComponentTickFunction* this_tick_function) override;

    // Must be called between BeginPlay() and EndPlay(), because GetWorld() needs to be valid.
    void subscribeToUserInputs(const std::vector<std::string>& user_input_names);
    void unsubscribeFromUserInputs(const std::vector<std::string>& user_input_names);

    // Set by the code using this class to specify what happens when user input (e.g., keyboard or gamepad input) is received.
    void setHandleUserInputFunc(const std::function<void(const std::string&, float)>& handle_user_input_func);

    // USpUserInputComponents need to be enabled explicitly to avoid a single key press unexpectedly doing multiple things.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR");
    bool bHandleUserInput = false;

private:
    std::string getUniqueAxisNameFromUserInputName(const std::string& user_input_name) const;

    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    TArray<FString> SubscribedToUserInputs;

    struct UserInputDesc
    {
        float scale_ = 1.0f;
        float threshold_ = 1.0f;
        FName axis_;
        FInputAxisKeyMapping input_axis_key_mapping_;
    };

    std::map<std::string, UserInputDesc> user_input_descs_;
    std::function<void(const std::string&, float)> handle_user_input_func_;

    UInputComponent* input_component_ = nullptr;
    UPlayerInput* player_input_ = nullptr;
};
