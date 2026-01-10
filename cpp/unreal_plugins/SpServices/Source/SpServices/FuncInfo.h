//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::same_as
#include <type_traits> // std::invoke_result_t, std::is_invocable_v

#include "SpCore/Boost.h"

template <typename TFunc, typename... TArgs>
concept CFuncIsCallableWithArgs = std::is_invocable_v<TFunc, TArgs...>;

template <typename TFunc, typename TReturn, typename... TArgs>
concept CFuncReturnsAndIsCallableWithArgs = std::same_as<TReturn, std::invoke_result_t<TFunc, TArgs...>>;

template <typename TReturn, typename... TArgs>
class FuncInfo {};

template <typename TReturn, typename TArgsTuple>
class FuncInfoHelper;

template <typename TReturn, typename... TArgs>
class FuncInfoHelper<TReturn, std::tuple<TArgs...>>
{
public:
    using TFuncInfo = FuncInfo<TReturn, TArgs...>;
};

class FuncInfoUtils
{
public:
    FuncInfoUtils() = delete;
    ~FuncInfoUtils() = delete;

    template <typename TFunc>
    static auto getFuncInfo()
    {
        using TReturn = boost::callable_traits::return_type_t<TFunc>;
        using TArgsTuple = boost::callable_traits::args_t<TFunc>;
        using TFuncInfo = typename FuncInfoHelper<TReturn, TArgsTuple>::TFuncInfo;

        return TFuncInfo();
    }
};
