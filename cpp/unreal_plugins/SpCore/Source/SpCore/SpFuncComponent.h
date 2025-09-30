//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional>  // std::function
#include <map>
#include <string>

#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Containers/Array.h> 
#include <HAL/Platform.h>            // SPCORE_API
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/FuncRegistry.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpTypes.h"

#include "SpFuncComponent.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API USpFuncComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    // typically called by the owning actor or component to register/unregister an SpFunc
    void initialize();
    void terminate();
    void registerSharedMemoryView(const std::string& shared_memory_name, const SpArraySharedMemoryView& shared_memory_view);
    void unregisterSharedMemoryView(const std::string& shared_memory_name);
    void registerFunc(const std::string& func_name, const std::function<SpFuncDataBundle(SpFuncDataBundle&)>& func);
    void unregisterFunc(const std::string& func_name);

    // typically called by code that wants to call an SpFunc
    std::map<std::string, SpArraySharedMemoryView> getSharedMemoryViews() const;
    SpFuncDataBundle callFunc(const std::string& func_name, SpFuncDataBundle& args) const;

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    TArray<FString> FuncNames;

    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    TArray<FString> SharedMemoryViewNames;

    FuncRegistry<SpFuncDataBundle, SpFuncDataBundle&> funcs_;
    std::map<std::string, SpArraySharedMemoryView> shared_memory_views_;
};
