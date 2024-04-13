//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <functional> // std::function
#include <map>
#include <span>
#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Containers/Array.h> 
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/CppFuncData.h"
#include "SpCore/CppFuncRegistrar.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/YamlCpp.h"

#include "CppFuncComponent.generated.h"

struct CppFuncComponentArgs
{
    std::map<std::string, CppFuncArg> args_;
    std::map<std::string, std::string> unreal_obj_strings_;
    YAML::Node config_;
};

struct CppFuncComponentReturnValues
{
    std::map<std::string, CppFuncReturnValue> return_values_;
    std::map<std::string, std::string> unreal_obj_strings_;
    YAML::Node info_;
};

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API UCppFuncComponent : public USceneComponent
{
    GENERATED_BODY()
public:

    UCppFuncComponent();
    ~UCppFuncComponent();

    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Func Names");
    TArray<FString> FuncNames;

    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Shared Memory View Names");
    TArray<FString> SharedMemoryViewNames;

    // typically called by the owning component to register/unregister functions and shared memory
    void registerSharedMemoryView(const std::string& name, const SharedMemoryView& shared_memory_view);
    void unregisterSharedMemoryView(const std::string& name);
    void registerFunc(const std::string& name, const std::function<CppFuncComponentReturnValues(const CppFuncComponentArgs&)>& func);
    void unregisterFunc(const std::string& name);

    // typically called by code that wants to call previously registered functions and shared memory
    const std::map<std::string, SharedMemoryView>& getSharedMemoryViews() const;
    CppFuncComponentReturnValues callFunc(const std::string& name, const CppFuncComponentArgs& args);

private:
    CppFuncRegistrar<CppFuncComponentReturnValues, const CppFuncComponentArgs&> funcs_;
    std::map<std::string, SharedMemoryView> shared_memory_views_;
};
