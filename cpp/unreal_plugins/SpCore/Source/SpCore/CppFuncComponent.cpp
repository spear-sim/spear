//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/CppFuncComponent.h"

#include <functional> // std::function
#include <map>
#include <string>

#include <Containers/UnrealString.h>
#include <Containers/Array.h>

#include "SpCore/CppFunc.h"
#include "SpCore/CppFuncRegistrar.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

UCppFuncComponent::UCppFuncComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

UCppFuncComponent::~UCppFuncComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

void UCppFuncComponent::registerFunc(const std::string& func_name, const std::function<CppFuncPackage(CppFuncPackage&)>& func)
{
    funcs_.registerFunc(func_name, func);
    FuncNames.Add(Unreal::toFString(func_name));
}

void UCppFuncComponent::unregisterFunc(const std::string& func_name)
{
    FuncNames.Remove(Unreal::toFString(func_name));
    funcs_.unregisterFunc(func_name);
}

void UCppFuncComponent::registerSharedMemoryView(const std::string& shared_memory_name, const CppFuncSharedMemoryView& shared_memory_view)
{
    SP_ASSERT(shared_memory_name != "");
    Std::insert(shared_memory_views_, shared_memory_name, shared_memory_view);
    SharedMemoryViewNames.Add(Unreal::toFString(shared_memory_name));
}

void UCppFuncComponent::unregisterSharedMemoryView(const std::string& shared_memory_name)
{
    SP_ASSERT(shared_memory_name != "");
    SharedMemoryViewNames.Remove(Unreal::toFString(shared_memory_name));
    Std::remove(shared_memory_views_, shared_memory_name);
}

const std::map<std::string, CppFuncSharedMemoryView>& UCppFuncComponent::getSharedMemoryViews() const
{
    return shared_memory_views_;
}

CppFuncPackage UCppFuncComponent::callFunc(const std::string& func_name, CppFuncPackage& args) const
{
    return funcs_.call(func_name, args);
}
