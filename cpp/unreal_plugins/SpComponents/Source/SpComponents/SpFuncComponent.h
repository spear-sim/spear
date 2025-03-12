//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional>  // std::function
#include <map>
#include <string>

#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Containers/Array.h> 
#include <HAL/Platform.h>            // SPCOMPONENTS_API
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/FuncRegistrar.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncDataBundle.h"

#include "SpFuncComponent.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpFuncComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpFuncComponent();
    ~USpFuncComponent();

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

    FuncRegistrar<SpFuncDataBundle, SpFuncDataBundle&> funcs_;
    std::map<std::string, SpArraySharedMemoryView> shared_memory_views_;
};
