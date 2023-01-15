//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Containers/UnrealString.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include "UObject/NameTypes.h"

#include "CoreUtils/Std.h"

class COREUTILS_API Unreal
{
public:

    //
    // String manipulation
    //

    static std::string toString(const FString& str)
    {
        // Note that the * operator for FString returns a pointer to the underlying string
        return std::string(TCHAR_TO_UTF8(*str));
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
    // Find actors by name or tag (non-templated)
    //

    static AActor* findActorByName(UWorld* world, const std::string& name)
    {
        return findActorByName<AActor>(world, name);
    }

    static std::map<std::string, AActor*> findActorsByNameAsMap(UWorld* world, const std::vector<std::string>& names)
    {
        return findActorsByNameAsMap<AActor>(world, names);
    }

    static std::vector<AActor*> findActorsByNameAsVector(UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        return findActorsByNameAsVector<AActor>(world, names, return_null_if_not_found);
    }

    static std::vector<AActor*> findActorsByTagAny(UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagAny<AActor>(world, tags);
    }

    static std::vector<AActor*> findActorsByTagAll(UWorld* world, const std::vector<std::string>& tags)
    {
        return findActorsByTagAll<AActor>(world, tags);
    }

    static std::vector<AActor*> findActorsByTag(UWorld* world, const std::string& tag)
    {
        return findActorsByTagAny<AActor>(world, {tag});
    }


    //
    // Find actors by name, tag, or type (templated by type)
    //

    template <typename TActor>
    static std::map<std::string, TActor*> findActorsByNameAsMap(UWorld* world, const std::vector<std::string>& names)
    {
        std::map<std::string, TActor*> actor_map;

        for (auto& name : names) {
            actor_map[name] = nullptr;
        }

        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TActor* a = (*itr);
            std::string name = toString(a->GetName());
            if (Std::containsKey(actor_map, name)) {
                ASSERT(!actor_map.at(name)); // There shouldn't be two actors with the same name
                actor_map[name] = a;
            }
        }

        return actor_map;
    }

    template <typename TActor>
    static std::vector<TActor*> findActorsByNameAsVector(UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true)
    {
        std::vector<TActor*> actors;

        std::map<std::string, TActor*> actor_map = findActorsByNameAsMap<TActor>(world, names);

        for (auto& name : names) {
            TActor* a = actor_map.at(name);
            if (return_null_if_not_found || a) {
                actors.push_back(a);
            }
        }

        return actors;
    }

    template <typename TActor>
    static TActor* findActorByName(UWorld* world, const std::string& name)
    {
        return findActorsByNameAsVector<TActor>(world, {name}).at(0);
    }

    template <typename TActor>
    static std::vector<TActor*> findActorsByTagAny(UWorld* world, const std::vector<std::string>& tags)
    {
        std::vector<TActor*> actors;

        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TActor* actor = (*itr);
            for (auto& tag : tags) {
                if (actor->ActorHasTag(tag.c_str())) {
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
        std::vector<TActor*> actors;

        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            TActor* actor = (*itr);
            bool all = true;
            for (auto& tag : tags) {
                if (!actor->ActorHasTag(tag.c_str())) {
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
    static std::vector<TActor*> findActorsByTag(UWorld* world, const std::string& tag)
    {
        return findActorsByTagAny<TActor>(world, {tag});
    }

    template <typename TActor>
    static std::vector<TActor*> findActorsByType(UWorld* world)
    {
        std::vector<TActor*> actors;
        for (TActorIterator<TActor> itr(world); itr; ++itr) {
            actors.push_back(*itr);
        }
        return actors;
    }

    template <typename TActor>
    static TActor* findActorByType(UWorld* world)
    {
        std::vector<TActor*> actors = findActorsByType<TActor>(world);
        if (actors.size() == 0) {
            return nullptr;
        } else if (actors.size() == 1) {
            return actors.at(0);
        } else {
            ASSERT(false);
            return nullptr;
        }
    }
};
