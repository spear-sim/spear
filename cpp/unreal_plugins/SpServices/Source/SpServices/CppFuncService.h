//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int8_t, uint8_t

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine

#include "SpCore/Assert.h"
#include "SpCore/CppFunc.h"
#include "SpCore/CppFuncComponent.h"
#include "SpCore/Rpclib.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/Std.h"

#include "SpServices/CppFuncServiceTypes.h"
#include "SpServices/EntryPointBinder.h"

// TODO: remove these headers when ACppFuncServiceDebugActor is removed as the hard-coded target for function calls
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

// TODO: remove this header when ACppFuncServiceDebugActor is removed as the hard-coded target for function calls
#include "CppFuncService.generated.h"

class UObject;
class UWorld;

// TODO: remove this class as the hard-coded target for function calls
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpCppFuncServiceDebugActor : public AActor
{
    GENERATED_BODY()
public: 
    ASpCppFuncServiceDebugActor()
    {
        SP_LOG_CURRENT_FUNCTION();

        cpp_func_component_ = Unreal::createComponentInsideOwnerConstructor<UCppFuncComponent>(this, "cpp_func");
        SP_ASSERT(cpp_func_component_);

        cpp_func_component_->registerFunc("hello_world", [this](CppFuncPackage& args) -> CppFuncPackage {
            CppFuncData<uint8_t> hello("hello");
            CppFuncData<double> my_data("my_data");

            hello.setData("Hello World!");
            my_data.setData({ 1.0, 2.0, 3.0 });

            // Return CppFuncData objects.
            CppFuncPackage return_values;
            return_values.items_ = CppFuncUtils::moveDataToItems({ hello.getPtr(), my_data.getPtr() });
            return return_values;
        });
    };

    ~ASpCppFuncServiceDebugActor()
    {
        SP_LOG_CURRENT_FUNCTION();
    };

private:
    UCppFuncComponent* cpp_func_component_ = nullptr;
};

class CppFuncService {
public:
    CppFuncService() = delete;
    CppFuncService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &CppFuncService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &CppFuncService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("cpp_func_service", "call_func", [this](const uint64_t& uobject, const std::string& func_name, const CppFuncServicePackage& args) -> CppFuncServicePackage {
            SP_ASSERT(world_);

            // get CppFuncComponent and shared memory views
            UCppFuncComponent* cpp_func_component = getCppFuncComponent(reinterpret_cast<UObject*>(uobject));
            SP_ASSERT(cpp_func_component);
            const std::map<std::string, CppFuncSharedMemoryView>& inner_shared_memory_views = cpp_func_component->getSharedMemoryViews();

            // prepare args
            CppFuncPackage inner_args = CppFuncServiceUtils::moveToPackage(args);
            CppFuncUtils::resolveArgs(inner_args.items_, inner_shared_memory_views);

            // call CppFunc
            CppFuncPackage inner_return_values = cpp_func_component->callFunc(func_name, inner_args);

            // prepare return values
            CppFuncUtils::resolveReturnValues(inner_return_values.items_, inner_shared_memory_views);
            CppFuncServicePackage return_values = CppFuncServiceUtils::moveToServicePackage(inner_return_values);

            return return_values;
        });

        unreal_entry_point_binder->bindFuncUnreal("cpp_func_service", "get_shared_memory_views", [this](const uint64_t& uobject) -> std::map<std::string, CppFuncServiceSharedMemoryView> {
            SP_ASSERT(world_);

            // get CppFuncComponent and shared memory views
            UCppFuncComponent* cpp_func_component                                           = getCppFuncComponent(reinterpret_cast<UObject*>(uobject));
            const std::map<std::string, CppFuncSharedMemoryView>& inner_shared_memory_views = cpp_func_component->getSharedMemoryViews();
            std::map<std::string, CppFuncServiceSharedMemoryView> shared_memory_views = CppFuncServiceUtils::toServiceSharedMemoryViews(inner_shared_memory_views);

            return shared_memory_views;
        });
    }

    ~CppFuncService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

    static UCppFuncComponent* getCppFuncComponent(const UObject* uobject);

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    UWorld* world_ = nullptr;
};
