//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Containers/Array.h>
#include <Containers/UnrealString.h>
#include <CoreMinimal.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <UObject/NameTypes.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Std.h"

class COREUTILS_API Unreal
{
public:

    //
    // String functions
    //

    static std::string toStdString(const FString& str)
    {
        // Note that the * operator for FString returns a pointer to the underlying string
        return std::string(TCHAR_TO_UTF8(*str));
    }

    static std::string toStdString(const FName& str)
    {
        // Note that str.ToString() converts FName to FString
        return toStdString(str.ToString());
    }

    static std::string toStdString(const TCHAR* str)
    {
        return std::string(TCHAR_TO_UTF8(str));
    }

    static FString toFString(const std::string& str)
    {
        return FString(UTF8_TO_TCHAR(str.c_str()));
    }

    static FName toFName(const std::string& str)
    {
        return FName(str.c_str());
    }

    //
    // Container functions
    //

    template <typename TDest, typename TSrc>
    static std::vector<TDest> reinterpretAs(const TArray<TSrc>& src)
    {
        std::vector<TDest> dest;
        if (src.Num() > 0) {
            size_t src_bytes = src.Num() * sizeof(TSrc);
            SP_ASSERT(src_bytes % sizeof(TDest) == 0);
            size_t dest_elements = src_bytes / sizeof(TDest);
            dest.resize(dest_elements);
            std::memcpy(dest.data(), src.GetData(), src_bytes);
        }
        return dest;
    }

    //
    // Find actor by name or tag, non-templated, return pointer
    //

    static AActor* findActorByName(UWorld* world, const std::string& name, bool assert_if_not_found = true)
    {
        return findActorByName<AActor>(world, name, assert_if_not_found);
    }

    static AActor* findActorByTag(UWorld* world, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        return findActorByTag<AActor>(world, tag, assert_if_not_found, assert_if_multiple_found);
    }

    static AActor* findActorByTagAny(
        UWorld* world,
        const std::vector<std::string>& tags,
        bool assert_if_not_found = true,
        bool assert_if_multiple_found = true)
    {
        return findActorByTagAny<AActor>(world, tags, assert_if_not_found, assert_if_multiple_found);
    }

    static AActor* findActorByTagAll(
        UWorld* world,
        const std::vector<std::string>& tags,
        bool assert_if_not_found = true,
        bool assert_if_multiple_found = true)
    {
        return findActorByTagAll<AActor>(world, tags, assert_if_not_found, assert_if_multiple_found);
    }

    //
    // Find actors by name or tag, non-templated, return std::vector
    //

    static std::vector<AActor*> findActorsByName(UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        return findActorsByName<AActor>(world, names, return_null_if_not_found);
    }

    static std::vector<AActor*> findActorsByTag(UWorld* world, const std::string& tag)
    {
        return findActorsByTag<AActor>(world, tag);
    }

    static std::vector<AActor*> findActorsByTagAny(UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagAny<AActor>(world, tags);
    }

    static std::vector<AActor*> findActorsByTagAll(UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagAll<AActor>(world, tags);
    }

    //
    // Find actors by name or tag, non-templated, return std::map
    //

    static std::map<std::string, AActor*> findActorsByNameAsMap(UWorld* world, const std::vector<std::string>& names)
    {
        return findActorsByNameAsMap<AActor>(world, names);
    }

    static std::map<std::string, AActor*> findActorsByTagAsMap(UWorld* world, const std::string& tag)
    {
        return findActorsByTagAsMap<AActor>(world, tag);
    }

    static std::map<std::string, AActor*> findActorsByTagAnyAsMap(UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagAnyAsMap<AActor>(world, tags);
    }

    static std::map<std::string, AActor*> findActorsByTagAllAsMap(UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagAllAsMap<AActor>(world, tags);
    }

    //
    // Find actor by name or tag or type, templated, return pointer
    //

    template <typename TActor>
    static TActor* findActorByName(UWorld* world, const std::string& name, bool assert_if_not_found = true)
    {
        TActor* default_val                     = nullptr;
        bool return_null_if_not_found           = true;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = true;
        return getItem(
            findActorsByName<TActor>(world, {name}, return_null_if_not_found), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <typename TActor>
    static TActor* findActorByTag(UWorld* world, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByTag<TActor>(world, tag), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <typename TActor>
    static TActor* findActorByTagAny(
        UWorld* world,
        const std::vector<std::string>& tags,
        bool assert_if_not_found      = true,
        bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByTagAny<TActor>(world, tags), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <typename TActor>
    static TActor* findActorByTagAll(
        UWorld* world,
        const std::vector<std::string>& tags,
        bool assert_if_not_found      = true,
        bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByTagAll<TActor>(world, tags), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    template <typename TActor>
    static TActor* findActorByType(UWorld* world, bool assert_if_not_found = true, bool assert_if_multiple_found = true)
    {
        TActor* default_val                     = nullptr;
        bool assert_if_size_is_zero             = assert_if_not_found;
        bool assert_if_size_is_greater_than_one = assert_if_multiple_found;
        return getItem(findActorsByType<TActor>(world), default_val, assert_if_size_is_zero, assert_if_size_is_greater_than_one);
    }

    //
    // Find actors by name or tag or type, templated, return std::vector
    //

    template <typename TActor>
    static std::vector<TActor*> findActorsByName(UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::vector<TActor*> actors;
        std::map<std::string, TActor*> actor_map = findActorsByNameAsMap<TActor>(world, names);
        for (auto& name : names) {
            TActor* a = actor_map.at(name);
            if (a || return_null_if_not_found) {
                actors.push_back(a);
            }
        }
        return actors;
    }

    template <typename TActor>
    static std::vector<TActor*> findActorsByTag(UWorld* world, const std::string& tag)
    {
        return findActorsByTagAny<TActor>(world, {tag});
    }

    template <typename TActor>
    static std::vector<TActor*> findActorsByTagAny(UWorld* world, const std::vector<std::string>& tags)
    {
        SP_ASSERT(world);
        std::vector<TActor*> actors;
        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TActor* actor = *itr;
            for (auto& tag : tags) {
                if (actor->ActorHasTag(toFName(tag))) {
                    actors.push_back(actor);
                    break;
                }
            }
        }
        return actors;
    }

    template <typename TActor>
    static std::vector<TActor*> findActorsByTagAll(UWorld* world, const std::vector<std::string>& tags)
    {
        SP_ASSERT(world);
        std::vector<TActor*> actors;
        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TActor* actor = *itr;
            bool all = true;
            for (auto& tag : tags) {
                if (!actor->ActorHasTag(toFName(tag))) {
                    all = false;
                    break;
                }
            }
            if (all) {
                actors.push_back(actor);
            }
        }
        return actors;
    }

    template <typename TActor>
    static std::vector<TActor*> findActorsByType(UWorld* world)
    {
        SP_ASSERT(world);
        std::vector<TActor*> actors;
        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            actors.push_back(*itr);
        }
        return actors;
    }

    //
    // Find actors by name or tag or type, templated, return std::map
    //

    template <typename TActor>
    static std::map<std::string, TActor*> findActorsByNameAsMap(UWorld* world, const std::vector<std::string>& names)
    {
        SP_ASSERT(world);
        std::map<std::string, TActor*> actor_map;
        for (auto& name : names) {
            actor_map[name] = nullptr;
        }
        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TActor* a = *itr;
            std::string name = toStdString(a->GetName());
            if (Std::containsKey(actor_map, name)) {
                SP_ASSERT(!actor_map.at(name)); // There shouldn't be two actors with the same name
                actor_map[name] = a;
            }
        }
        return actor_map;
    }

    template <typename TActor>
    static std::map<std::string, TActor*> findActorsByTagAsMap(UWorld* world, const std::string& tag)
    {
        return getActorsAsMap(findActorsByTag<TActor>(world, tag));
    }
    
    template <typename TActor>
    static std::map<std::string, TActor*> findActorsByTagAnyAsMap(UWorld* world, const std::vector<std::string>& tags)
    {
        return getActorsAsMap(findActorsByTagAny<TActor>(world, tags));
    }
    template <typename TActor>
    static std::map<std::string, TActor*> findActorsByTagAllAsMap(UWorld* world, const std::vector<std::string>& tags)
    {
        return getActorsAsMap(findActorsByTagAll<TActor>(world, tags));
    }
    
    template <typename TActor>
    static std::map<std::string, TActor*> findActorsByTypeAsMap(UWorld* world)
    {
        return getActorsAsMap(findActorsByType<TActor>(world));
    }

    //
    // Helper functions for finding actors
    //

    template <typename TActor>
    static std::map<std::string, TActor*> getActorsAsMap(const std::vector<TActor*>& actors)
    {
        std::map<std::string, TActor*> actor_map;
        for (auto& a : actors) {
            SP_ASSERT(a);
            std::string name = toStdString(a->GetName());
            SP_ASSERT(!Std::containsKey(actor_map, name)); // There shouldn't be two actors with the same name
            actor_map[name] = a;
        }
        return actor_map;
    }

    template <typename T>
    static const T& getItem(const std::vector<T>& vec, const T& default_val, bool assert_if_size_is_zero, bool assert_if_size_is_greater_than_one)
    {
        if (vec.size() == 0) {
            if (assert_if_size_is_zero) {
                SP_ASSERT(false);
            }
            return default_val;
        } else if (vec.size() == 1) {
            return vec.at(0);
        } else {
            if (assert_if_size_is_greater_than_one) {
                SP_ASSERT(false);
            }
            return vec.at(0);
        }
    }
};
