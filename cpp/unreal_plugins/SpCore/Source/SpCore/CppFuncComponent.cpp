//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/CppFuncComponent.h"

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

void UCppFuncComponent::registerFunc(const std::string& func_name, const std::function<TReturn(const TArgs&)>& func)
{
    funcs_.registerFunc(func_name, func);
    FuncNames.Add(Unreal::toFString(func_name));
}

void UCppFuncComponent::unregisterFunc(const std::string& func_name)
{
    FuncNames.Remove(Unreal::toFString(func_name));
    funcs_.unregisterFunc(func_name);
}

typename UCppFuncComponent::TReturn UCppFuncComponent::call(const std::string& func_name, const typename UCppFuncComponent::TArgs& args)
{
    return funcs_.call(func_name, args);
}
