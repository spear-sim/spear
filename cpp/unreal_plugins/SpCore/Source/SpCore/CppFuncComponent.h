//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <string>
#include <map>
#include <vector>

#include <Components/SceneComponent.h>
#include <UObject/ObjectMacros.h>      // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/CppFuncRegistrar.h"

#include "CppFuncComponent.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API UCppFuncComponent : public USceneComponent
{
    GENERATED_BODY()
public:

    using TReturn = std::map<std::string, std::vector<uint8_t>>;
    using TArgs = std::map<std::string, std::span<const uint8_t>>;

    UCppFuncComponent();
    ~UCppFuncComponent();

    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Func Names");
    TArray<FString> FuncNames;

    void registerFunc(const std::string& func_name, const std::function<TReturn(const TArgs&)>& func);
    void unregisterFunc(const std::string& func_name);
    TReturn call(const std::string& func_name, const TArgs& args);

private:
    CppFuncRegistrar<TReturn, const TArgs&> funcs_;
};
