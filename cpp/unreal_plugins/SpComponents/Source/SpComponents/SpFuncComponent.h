//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional>  // std::function
#include <map>
#include <string>
#include <type_traits> // std::underlying_type_t
#include <vector>

#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Containers/Array.h> 
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/FuncRegistrar.h"
#include "SpCore/SpFuncArray.h"

#include "SpFuncComponent.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpFuncComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpFuncComponent();
    ~USpFuncComponent();

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Func Names");
    TArray<FString> FuncNames;

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Shared Memory View Names");
    TArray<FString> SharedMemoryViewNames;

    // typically called by the owning actor or component to register/unregister an SpFunc
    void registerSharedMemoryView(const std::string& shared_memory_name, const SpFuncSharedMemoryView& shared_memory_view);
    void unregisterSharedMemoryView(const std::string& shared_memory_name);
    void registerFunc(const std::string& func_name, const std::function<SpFuncDataBundle(SpFuncDataBundle&)>& func);
    void unregisterFunc(const std::string& func_name);

    // typically called by code that wants to call an SpFunc; note that getSharedMemoryViews() returns by
    // const reference because getSharedMemoryViews() is called every time we call an SpFunc, and returning
    // by value might be expensive if lots of shared memory views have been registered
    const std::map<std::string, SpFuncSharedMemoryView>& getSharedMemoryViews() const;
    SpFuncDataBundle callFunc(const std::string& func_name, SpFuncDataBundle& args) const;

private:
    FuncRegistrar<SpFuncDataBundle, SpFuncDataBundle&> funcs_;
    std::map<std::string, SpFuncSharedMemoryView> shared_memory_views_;
};
