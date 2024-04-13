//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/CppFuncComponent.h"

#include <functional> // std::function
#include <string>

#include "SpCore/CppFuncRegistrar.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/Unreal.h"

UCppFuncComponent::UCppFuncComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

UCppFuncComponent::~UCppFuncComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

void UCppFuncComponent::registerFunc(const std::string& name, const std::function<CppFuncComponentReturnValues(const CppFuncComponentArgs&)>& func)
{
    funcs_.registerFunc(name, func);
    FuncNames.Add(Unreal::toFString(name));
}

void UCppFuncComponent::unregisterFunc(const std::string& name)
{
    FuncNames.Remove(Unreal::toFString(name));
    funcs_.unregisterFunc(name);
}

void UCppFuncComponent::registerSharedMemoryView(const std::string& name, const SharedMemoryView& shared_memory_view)
{
    Std::insert(shared_memory_views_, name, shared_memory_view);
    SharedMemoryViewNames.Add(Unreal::toFString(name));
}

void UCppFuncComponent::unregisterSharedMemoryView(const std::string& name)
{
    SharedMemoryViewNames.Remove(Unreal::toFString(name));
    Std::remove(shared_memory_views_, name);
}

const std::map<std::string, SharedMemoryView>& UCppFuncComponent::getSharedMemoryViews() const
{
    return shared_memory_views_;
}

CppFuncComponentReturnValues UCppFuncComponent::callFunc(const std::string& name, const CppFuncComponentArgs& args)
{
    return funcs_.call(name, args);
}
