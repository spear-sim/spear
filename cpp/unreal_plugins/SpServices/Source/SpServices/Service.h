//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <map>
#include <ranges>  // std::views::transform
#include <string>
#include <utility> // std::make_pair
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // UWorld::InitializationValues
#include <Misc/CoreDelegates.h>
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS, UENUM, UPROPERTY, USTRUCT

#include "SpCore/Std.h"

class Service {
public:
    Service();
    ~Service();

    UWorld* getWorld();

    virtual void postWorldInitialization(UWorld* world, const UWorld::InitializationValues initialization_values);
    virtual void worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources);
    virtual void worldBeginPlay() {}

protected:
    template <typename TValue>
    static uint64_t toUInt64(const TValue* src)
    {
        return reinterpret_cast<uint64_t>(src);
    }

    template <typename TValue>
    static std::vector<uint64_t> toUInt64(const std::vector<TValue>& src)
    {
        return Std::reinterpretAsVectorOf<uint64_t>(src);
    }

    template <typename TKey, typename TValue>
    static std::map<TKey, uint64_t> toUInt64(const std::map<TKey, TValue>& src)
    {
        return Std::toMap<TKey, uint64_t>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toUInt64(value)); }));
    }

    template <typename TPtr>
    static TPtr* toPtr(uint64_t src)
    {
        return reinterpret_cast<TPtr*>(src);
    }

    template <typename TPtr>
    static std::vector<TPtr*> toPtr(const std::vector<uint64_t>& src)
    {
        return Std::reinterpretAsVectorOf<TPtr*>(src);
    }

    template <typename TKey, typename TPtr>
    static std::map<TKey, TPtr*> toPtr(const std::map<TKey, uint64_t>& src)
    {
        return Std::toMap<TKey, TPtr*>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toPtr<TPtr>(value)); }));
    }

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);
    void worldBeginPlayHandler();

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    FDelegateHandle world_begin_play_handle_;

    UWorld* world_ = nullptr;
};
