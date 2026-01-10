//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpFuncComponent.h"

#include <functional> // std::function
#include <map>
#include <string>
#include <vector>

#include <Containers/Array.h>

#include "SpCore/FuncRegistry.h"
#include "SpCore/Log.h"
#include "SpCore/SpArray.h"
#include "SpCore/Unreal.h"

void USpFuncComponent::OnComponentCreated()
{
    SP_LOG_CURRENT_FUNCTION();

    UActorComponent::OnComponentCreated();
    SpFuncComponentPtr = Unreal::toFString(Std::toStringFromPtr(this));
}

void USpFuncComponent::PostLoad()
{
    SP_LOG_CURRENT_FUNCTION();

    UActorComponent::PostLoad();
    SpFuncComponentPtr = Unreal::toFString(Std::toStringFromPtr(this));
}

void USpFuncComponent::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    UActorComponent::BeginPlay();
    FuncNames.Empty();
    SharedMemoryViewNames.Empty();
}

void USpFuncComponent::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    UActorComponent::EndPlay(end_play_reason);
}

void USpFuncComponent::registerSharedMemoryView(const SpArraySharedMemoryView& shared_memory_view)
{
    SP_ASSERT(shared_memory_view.name_ != "");
    SP_ASSERT(shared_memory_view.name_ != "smem:invalid");
    Std::insert(shared_memory_views_, shared_memory_view.name_, shared_memory_view);
    SharedMemoryViewNames.Add(Unreal::toFString(shared_memory_view.name_));
}

void USpFuncComponent::unregisterSharedMemoryView(const SpArraySharedMemoryView& shared_memory_view)
{
    SP_ASSERT(shared_memory_view.name_ != "");
    SP_ASSERT(shared_memory_view.name_ != "smem:invalid");
    Std::remove(shared_memory_views_, shared_memory_view.name_);
    SharedMemoryViewNames.Remove(Unreal::toFString(shared_memory_view.name_));
}

void USpFuncComponent::registerFunc(const std::string& func_name, const std::function<SpFuncDataBundle(SpFuncDataBundle&)>& func)
{
    funcs_.registerFunc(func_name, func);
    FuncNames.Add(Unreal::toFString(func_name));
}

void USpFuncComponent::unregisterFunc(const std::string& func_name)
{
    funcs_.unregisterFunc(func_name);
    FuncNames.Remove(Unreal::toFString(func_name));
}

std::vector<std::string> USpFuncComponent::getFuncNames() const
{
    return funcs_.getFuncNames();
}

std::map<std::string, SpArraySharedMemoryView> USpFuncComponent::getSharedMemoryViews() const
{
    return shared_memory_views_;
}

SpFuncDataBundle USpFuncComponent::callFunc(const std::string& func_name, SpFuncDataBundle& args) const
{
    return funcs_.call(func_name, args);
}
