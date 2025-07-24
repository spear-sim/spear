//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <array>
#include <concepts>    // std::same_as
#include <type_traits> // std::invoke_result_t, std::is_invocable_v

#include "SpCore/Boost.h"

template <typename TFunc, typename... TArgs>
concept CFuncIsCallableWithArgs = std::is_invocable_v<TFunc, TArgs...>;

template <typename TFunc, typename TReturn, typename... TArgs>
concept CFuncReturnsAndIsCallableWithArgs = std::same_as<TReturn, std::invoke_result_t<TFunc, TArgs...>>;

template <typename TReturn, typename... TArgs>
class FuncInfo {};

class FuncInfoUtils
{
public:
    FuncInfoUtils() = delete;
    ~FuncInfoUtils() = delete;

    template <typename TFunc>
    static auto getFuncInfo()
    {
        using TArgsTuple = boost::callable_traits::args_t<TFunc>;
        using TReturn = boost::callable_traits::return_type_t<TFunc>; // can't use TReturn = std::invoke_result_t<TFunc, TArgs...> because we haven't deduced TArgs... yet

        return getFuncInfo<TReturn>(std::array<TArgsTuple, 0>());
    }

private:
    template <typename TReturn, typename... TArgs>
    static auto getFuncInfo(std::array<std::tuple<TArgs...>, 0> array)
    {
        return FuncInfo<TReturn, TArgs...>();
    }
};
