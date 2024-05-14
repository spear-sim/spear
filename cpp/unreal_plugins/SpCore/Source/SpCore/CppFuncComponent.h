//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <functional>  // std::function
#include <map>
#include <string>
#include <type_traits> // std::underlying_type_t
#include <vector>

#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Containers/Array.h> 
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UPROPERTY

#include "SpCore/CppFunc.h"
#include "SpCore/CppFuncRegistrar.h"

#include "CppFuncComponent.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API UCppFuncComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    UCppFuncComponent();
    ~UCppFuncComponent();

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Func Names");
    TArray<FString> FuncNames;

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Shared Memory View Names");
    TArray<FString> SharedMemoryViewNames;

    // typically called by the owning actor or component to register/unregister a CppFunc
    void registerSharedMemoryView(const std::string& shared_memory_name, const CppFuncSharedMemoryView& shared_memory_view);
    void unregisterSharedMemoryView(const std::string& shared_memory_name);
    void registerFunc(const std::string& func_name, const std::function<CppFuncPackage(CppFuncPackage&)>& func);
    void unregisterFunc(const std::string& func_name);

    // typically called by code that wants to call a CppFunc
    const std::map<std::string, CppFuncSharedMemoryView>& getSharedMemoryViews() const;
    CppFuncPackage callFunc(const std::string& func_name, CppFuncPackage& args) const;

private:
    CppFuncRegistrar<CppFuncPackage, CppFuncPackage&> funcs_;
    std::map<std::string, CppFuncSharedMemoryView> shared_memory_views_;
};
