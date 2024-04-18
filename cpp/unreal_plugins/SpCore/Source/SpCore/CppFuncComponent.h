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
#include "SpCore/SharedMemoryRegion.h"

#include "CppFuncComponent.generated.h"

// This is the input type and return type for all CppFuncs. We choose to make this a struct so it will be
// easier to add fields if necessary, without needing to update the definition of every CppFunc.
struct CppFuncComponentItems
{
    std::map<std::string, CppFuncItem> items_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};

// Needs to match SpEngine/CppFuncService.h
enum class CppFuncSharedMemoryUsageFlags : uint8_t
{
    DoNotUse    = 0,
    Arg         = 1 << 0,
    ReturnValue = 1 << 1
};

struct CppFuncSharedMemoryView
{
    SharedMemoryView view_;
    CppFuncSharedMemoryUsageFlags usage_flags_;
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

    // typically called by the owning component to register/unregister CppFuncs
    void registerSharedMemoryView(const std::string& shared_memory_name, const SharedMemoryView& shared_memory_view, CppFuncSharedMemoryUsageFlags usage_flags);
    void unregisterSharedMemoryView(const std::string& shared_memory_name);
    void registerFunc(const std::string& func_name, const std::function<CppFuncComponentItems(CppFuncComponentItems&)>& func);
    void unregisterFunc(const std::string& func_name);

    // typically called by code that wants to call CppFuncs
    const std::map<std::string, CppFuncSharedMemoryView>& getSharedMemoryViews() const;
    CppFuncComponentItems callFunc(const std::string& func_name, CppFuncComponentItems& args);

private:
    CppFuncRegistrar<CppFuncComponentItems, CppFuncComponentItems&> funcs_;
    std::map<std::string, CppFuncSharedMemoryView> shared_memory_views_;
};

//
// Needed to perform | and & operations on CppFuncSharedMemoryUsageFlags
//

static CppFuncSharedMemoryUsageFlags operator|(CppFuncSharedMemoryUsageFlags lhs, CppFuncSharedMemoryUsageFlags rhs)
{
    return static_cast<CppFuncSharedMemoryUsageFlags>(
        static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(lhs) |
        static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(rhs));
}
static CppFuncSharedMemoryUsageFlags operator&(CppFuncSharedMemoryUsageFlags lhs, CppFuncSharedMemoryUsageFlags rhs)
{
    return static_cast<CppFuncSharedMemoryUsageFlags>(
        static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(lhs) &
        static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(rhs));
}
static bool operator||(CppFuncSharedMemoryUsageFlags lhs, bool rhs) // needed for SP_ASSERT
{
    return static_cast<std::underlying_type_t<CppFuncSharedMemoryUsageFlags>>(lhs) || rhs;
}
