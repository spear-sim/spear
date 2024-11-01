//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::same_as
#include <type_traits> // std::invoke_result_t, std::is_invocable_v

template <typename TFunc>
concept CFuncIsCallableWithNoArgs = std::is_invocable_v<TFunc>;

template <typename TFunc, typename... TArgs>
concept CFuncIsCallableWithArgs = std::is_invocable_v<TFunc, TArgs...>;

template <typename TFunc, typename TReturn, typename... TArgs>
concept CFuncReturnsAndIsCallableWithArgs = std::same_as<TReturn, std::invoke_result_t<TFunc, TArgs...>>;

template <typename TClass>
struct FuncInfo : public FuncInfo<decltype(&TClass::operator())> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(TClass::*)(TArgs...)> : public FuncInfo<TReturn(*)(TArgs...)> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(TClass::*)(TArgs...) const> : public FuncInfo<TReturn(*)(TArgs...)> {};

template <class T>
struct FuncInfo<T&> : public FuncInfo<T> {};

template <typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(*)(TArgs...)> {};
