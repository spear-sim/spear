//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <map>
#include <string>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class AActor;
class UWorld;

class SPCORE_API UnrealClassRegistrar
{
public:
    UnrealClassRegistrar() = delete;
    ~UnrealClassRegistrar() = delete;

    static void initialize();
    static void terminate();

    //
    // Register actor
    //

    template <CActor TActor>
    static void registerActorClass(const std::string& class_name)
    {
        s_find_actors_by_name_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) -> std::vector<AActor*> {
                return Unreal::findActorsByName<TActor, AActor>(world, names, return_null_if_not_found);
            });

        s_find_actors_by_tag_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::string& tag) -> std::vector<AActor*> {
                return Unreal::findActorsByTag<TActor, AActor>(world, tag);
            });

        s_find_actors_by_tag_any_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::vector<AActor*> {
                return Unreal::findActorsByTagAny<TActor, AActor>(world, tags);
            });

        s_find_actors_by_tag_all_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::vector<AActor*> {
                return Unreal::findActorsByTagAll<TActor, AActor>(world, tags);
            });

        s_find_actors_by_type_registrar_.registerClass(
            class_name, [](const UWorld* world) -> std::vector<AActor*> {
                return Unreal::findActorsByType<TActor, AActor>(world);
            });

        s_find_actors_by_name_as_map_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& names) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByNameAsMap<TActor, AActor>(world, names);
            });

        s_find_actors_by_tag_as_map_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::string& tag) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAsMap<TActor, AActor>(world, tag);
            });
        
        s_find_actors_by_tag_any_as_map_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAnyAsMap<TActor, AActor>(world, tags);
            });
        
        s_find_actors_by_tag_all_as_map_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAllAsMap<TActor, AActor>(world, tags);
            });
        
        s_find_actors_by_type_as_map_registrar_.registerClass(
            class_name, [](const UWorld* world) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTypeAsMap<TActor, AActor>(world);
            });
        
        s_find_actor_by_name_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::string& name, bool assert_if_not_found) -> AActor* {
                return Unreal::findActorByName<TActor, AActor>(world, name, assert_if_not_found);
            });
        
        s_find_actor_by_tag_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByTag<TActor, AActor>(world, tag, assert_if_not_found, assert_if_multiple_found);
            });
        
        s_find_actor_by_tag_any_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByTagAny<TActor, AActor>(world, tags, assert_if_not_found, assert_if_multiple_found);
            });
        
        s_find_actor_by_tag_all_registrar_.registerClass(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByTagAll<TActor, AActor>(world, tags, assert_if_not_found, assert_if_multiple_found);
            });
        
        s_find_actor_by_type_registrar_.registerClass(
            class_name, [](const UWorld* world, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByType<TActor, AActor>(world, assert_if_not_found, assert_if_multiple_found);
            });
    }

    //
    // Register component
    //

    template <CComponent TComponent>
    static void registerComponentClass(const std::string& class_name)
    {
        s_get_components_by_name_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByName<TComponent, UActorComponent>(actor, names, return_null_if_not_found);
            });

        s_get_components_by_tag_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::string& tag) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTag<TComponent, UActorComponent>(actor, tag);
            });

        s_get_components_by_tag_any_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTagAny<TComponent, UActorComponent>(actor, tags);
            });

        s_get_components_by_tag_all_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTagAll<TComponent, UActorComponent>(actor, tags);
            });

        s_get_components_by_type_registrar_.registerClass(
            class_name, [](const AActor* actor) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByType<TComponent, UActorComponent>(actor);
            });

        s_get_components_by_name_as_map_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& names) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByNameAsMap<TComponent, UActorComponent>(actor, names);
            });

        s_get_components_by_tag_as_map_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::string& tag) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAsMap<TComponent, UActorComponent>(actor, tag);
            });
        
        s_get_components_by_tag_any_as_map_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAnyAsMap<TComponent, UActorComponent>(actor, tags);
            });
        
        s_get_components_by_tag_all_as_map_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAllAsMap<TComponent, UActorComponent>(actor, tags);
            });
        
        s_get_components_by_type_as_map_registrar_.registerClass(
            class_name, [](const AActor* actor) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTypeAsMap<TComponent, UActorComponent>(actor);
            });
        
        s_get_component_by_name_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::string& name, bool assert_if_not_found) -> UActorComponent* {
                return Unreal::getComponentByName<TComponent, UActorComponent>(actor, name, assert_if_not_found);
            });
        
        s_get_component_by_tag_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByTag<TComponent, UActorComponent>(actor, tag, assert_if_not_found, assert_if_multiple_found);
            });
        
        s_get_component_by_tag_any_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByTagAny<TComponent, UActorComponent>(actor, tags, assert_if_not_found, assert_if_multiple_found);
            });
        
        s_get_component_by_tag_all_registrar_.registerClass(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByTagAll<TComponent, UActorComponent>(actor, tags, assert_if_not_found, assert_if_multiple_found);
            });
        
        s_get_component_by_type_registrar_.registerClass(
            class_name, [](const AActor* actor, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByType<TComponent, UActorComponent>(actor, assert_if_not_found, assert_if_multiple_found);
            });
    }

    //
    // Find actors using a class name instead of template parameters
    //

    static std::vector<AActor*> findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag) {
        return s_find_actors_by_tag_registrar_.create(class_name, world, tag);
    }

    static std::vector<AActor*> findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
        return s_find_actors_by_tag_any_registrar_.create(class_name, world, tags);
    }

    static std::vector<AActor*> findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
        return s_find_actors_by_tag_all_registrar_.create(class_name, world, tags);
    }

    static std::vector<AActor*> findActorsByType(const std::string& class_name, const UWorld* world) {
        return s_find_actors_by_type_registrar_.create(class_name, world);
    }

    static std::map<std::string, AActor*> findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names) {
        return s_find_actors_by_name_as_map_registrar_.create(class_name, world, names);
    }

    static std::map<std::string, AActor*> findActorsByTagAsMap(const std::string& class_name, const UWorld* world, const std::string& tag) {
        return s_find_actors_by_tag_as_map_registrar_.create(class_name, world, tag);
    }

    static std::map<std::string, AActor*> findActorsByTagAnyAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
        return s_find_actors_by_tag_any_as_map_registrar_.create(class_name, world, tags);
    }
    
    static std::map<std::string, AActor*> findActorsByTagAllAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
        return s_find_actors_by_tag_all_as_map_registrar_.create(class_name, world, tags);
    }
    
    static std::map<std::string, AActor*> findActorsByTypeAsMap(const std::string& class_name, const UWorld* world) {
        return s_find_actors_by_type_as_map_registrar_.create(class_name, world);
    }
    
    static AActor* findActorByName(const std::string& class_name, const UWorld* world, const std::string& name, bool assert_if_not_found = true) {
        return s_find_actor_by_name_registrar_.create(class_name, world, name, assert_if_not_found);
    }
    
    static AActor* findActorByTag(const std::string& class_name, const UWorld* world, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_find_actor_by_tag_registrar_.create(class_name, world, tag, assert_if_not_found, assert_if_multiple_found);
    }
    
    static AActor* findActorByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_find_actor_by_tag_any_registrar_.create(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
    }
    
    static AActor* findActorByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_find_actor_by_tag_all_registrar_.create(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
    }
    
    static AActor* findActorByType(const std::string& class_name, const UWorld* world, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_find_actor_by_type_registrar_.create(class_name, world, assert_if_not_found, assert_if_multiple_found);
    }

    //
    // Find components using a class name instead of template parameters
    //

    static std::vector<UActorComponent*> getComponentsByTag(const std::string& class_name, const AActor* actor, const std::string& tag) {
        return s_get_components_by_tag_registrar_.create(class_name, actor, tag);
    }

    static std::vector<UActorComponent*> getComponentsByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
        return s_get_components_by_tag_any_registrar_.create(class_name, actor, tags);
    }

    static std::vector<UActorComponent*> getComponentsByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
        return s_get_components_by_tag_all_registrar_.create(class_name, actor, tags);
    }

    static std::vector<UActorComponent*> getComponentsByType(const std::string& class_name, const AActor* actor) {
        return s_get_components_by_type_registrar_.create(class_name, actor);
    }

    static std::map<std::string, UActorComponent*> getComponentsByNameAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names) {
        return s_get_components_by_name_as_map_registrar_.create(class_name, actor, names);
    }

    static std::map<std::string, UActorComponent*> getComponentsByTagAsMap(const std::string& class_name, const AActor* actor, const std::string& tag) {
        return s_get_components_by_tag_as_map_registrar_.create(class_name, actor, tag);
    }

    static std::map<std::string, UActorComponent*> getComponentsByTagAnyAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
        return s_get_components_by_tag_any_as_map_registrar_.create(class_name, actor, tags);
    }
    
    static std::map<std::string, UActorComponent*> getComponentsByTagAllAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
        return s_get_components_by_tag_all_as_map_registrar_.create(class_name, actor, tags);
    }
    
    static std::map<std::string, UActorComponent*> getComponentsByTypeAsMap(const std::string& class_name, const AActor* actor) {
        return s_get_components_by_type_as_map_registrar_.create(class_name, actor);
    }
    
    static UActorComponent* getComponentByName(const std::string& class_name, const AActor* actor, const std::string& name, bool assert_if_not_found = true) {
        return s_get_component_by_name_registrar_.create(class_name, actor, name, assert_if_not_found);
    }
    
    static UActorComponent* getComponentByTag(const std::string& class_name, const AActor* actor, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_get_component_by_tag_registrar_.create(class_name, actor, tag, assert_if_not_found, assert_if_multiple_found);
    }
    
    static UActorComponent* getComponentByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_get_component_by_tag_any_registrar_.create(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found);
    }
    
    static UActorComponent* getComponentByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_get_component_by_tag_all_registrar_.create(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found);
    }
    
    static UActorComponent* getComponentByType(const std::string& class_name, const AActor* actor, bool assert_if_not_found = true, bool assert_if_multiple_found = true) {
        return s_get_component_by_type_registrar_.create(class_name, actor, assert_if_not_found, assert_if_multiple_found);
    }

private:

    //
    // A ClassRegistrar<TReturn, TArgs...> is a templated type that allows a caller to create objects by name
    // instead of by type. TReturn is the common type that is always returned by the ClassRegistrar (e.g., a
    // base class pointer type). TArgs... are common argument types that will always be passed to a user-defined
    // "create" function that can be specialized by type (e.g., a user-defined create function might call the
    // new operator on a derived type). The user can also specify a type-specialized "destroy" function that
    // is responsible for cleaning up whatever was previously created (e.g., by calling the delete operator on
    // a derived pointer). Consider the following registar.
    // 
    //     ClassRegistrar<void*, int> my_registar;
    //
    // All create functions that are registered with my_registrar must take as input an int and return as output
    // a void*. We can register a specific type by calling registerClass(...) as follows.
    //
    //     my_registrar.registerClass(
    //         "float",
    //         [](int num_elements) -> void* { return new float[num_elements]; },          // create function
    //         [](void* created) -> void { delete[] reinterpret_cast<float*>(created); }); // destroy function
    //
    // Here, we are registering the name "float" with a create function that allocates an array of floats, and
    // a destroy function that deletes the array. In our destroy function, we need to cast the void* pointer to
    // its correct type because calling deleting a void* is undefined behavior. At this point, we are able to
    // call our create and delete functions using only their name.
    //
    //     void* my_ptr = my_registrar.create("float", 10); // create an array of 10 floats
    //     my_registrar.destroy("float", my_ptr);           // correctly destroy array
    //

    template <typename TReturn, typename... TArgs>
    class ClassRegistrar
    {
    public:
        void registerClass(const std::string& class_name, const std::function<TReturn(TArgs...)>& create_func, const std::function<void(TReturn&)> destroy_func)
        {
            registerClass(class_name, create_func);
            registerClass(class_name, destroy_func);
        }

        void registerClass(const std::string& class_name, const std::function<TReturn(TArgs...)>& create_func)
        {
            Std::insert(create_funcs_, class_name, create_func);
        }

        void registerClass(const std::string& class_name, const std::function<void(TReturn&)> destroy_func)
        {
            Std::insert(destroy_funcs_, class_name, destroy_func);
        }

        TReturn create(const std::string& class_name, TArgs... args)
        {
            SP_ASSERT(Std::containsKey(create_funcs_, class_name));
            SP_ASSERT(create_funcs_.at(class_name));
            return create_funcs_.at(class_name)(args...);
        }

        void destroy(const std::string& class_name, TReturn& created)
        {
            SP_ASSERT(Std::containsKey(destroy_funcs_, class_name));
            SP_ASSERT(destroy_funcs_.at(class_name));
            destroy_funcs_.at(class_name)(created);
        }

    private:
        std::map<std::string, std::function<TReturn(TArgs...)>> create_funcs_;
        std::map<std::string, std::function<void(TReturn&)>> destroy_funcs_;
    };

    inline static ClassRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&, bool>     s_find_actors_by_name_registrar_;
    inline static ClassRegistrar<std::vector<AActor*>, const UWorld*, const std::string&>                        s_find_actors_by_tag_registrar_;
    inline static ClassRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>           s_find_actors_by_tag_any_registrar_;
    inline static ClassRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>           s_find_actors_by_tag_all_registrar_;
    inline static ClassRegistrar<std::vector<AActor*>, const UWorld*>                                            s_find_actors_by_type_registrar_;
    inline static ClassRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&> s_find_actors_by_name_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::string&>              s_find_actors_by_tag_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&> s_find_actors_by_tag_any_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&> s_find_actors_by_tag_all_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, AActor*>, const UWorld*>                                  s_find_actors_by_type_as_map_registrar_;
    inline static ClassRegistrar<AActor*, const UWorld*, const std::string&, bool>                               s_find_actor_by_name_registrar_;
    inline static ClassRegistrar<AActor*, const UWorld*, const std::string&, bool, bool>                         s_find_actor_by_tag_registrar_;
    inline static ClassRegistrar<AActor*, const UWorld*, const std::vector<std::string>&, bool, bool>            s_find_actor_by_tag_any_registrar_;
    inline static ClassRegistrar<AActor*, const UWorld*, const std::vector<std::string>&, bool, bool>            s_find_actor_by_tag_all_registrar_;
    inline static ClassRegistrar<AActor*, const UWorld*, bool, bool>                                             s_find_actor_by_type_registrar_;

    inline static ClassRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>     s_get_components_by_name_registrar_;
    inline static ClassRegistrar<std::vector<UActorComponent*>, const AActor*, const std::string&>                        s_get_components_by_tag_registrar_;
    inline static ClassRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&>           s_get_components_by_tag_any_registrar_;
    inline static ClassRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&>           s_get_components_by_tag_all_registrar_;
    inline static ClassRegistrar<std::vector<UActorComponent*>, const AActor*>                                            s_get_components_by_type_registrar_;
    inline static ClassRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&> s_get_components_by_name_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::string&>              s_get_components_by_tag_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&> s_get_components_by_tag_any_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&> s_get_components_by_tag_all_as_map_registrar_;
    inline static ClassRegistrar<std::map<std::string, UActorComponent*>, const AActor*>                                  s_get_components_by_type_as_map_registrar_;
    inline static ClassRegistrar<UActorComponent*, const AActor*, const std::string&, bool>                               s_get_component_by_name_registrar_;
    inline static ClassRegistrar<UActorComponent*, const AActor*, const std::string&, bool, bool>                         s_get_component_by_tag_registrar_;
    inline static ClassRegistrar<UActorComponent*, const AActor*, const std::vector<std::string>&, bool, bool>            s_get_component_by_tag_any_registrar_;
    inline static ClassRegistrar<UActorComponent*, const AActor*, const std::vector<std::string>&, bool, bool>            s_get_component_by_tag_all_registrar_;
    inline static ClassRegistrar<UActorComponent*, const AActor*, bool, bool>                                             s_get_component_by_type_registrar_;
};
