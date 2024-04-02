//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <map>
#include <string>
#include <vector>

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
    // Find actors using a class name instead of template parameters
    //

    static std::vector<AActor*> findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag);
    static std::vector<AActor*> findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::vector<AActor*> findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::vector<AActor*> findActorsByType(const std::string& class_name, const UWorld* world);
    static std::map<std::string, AActor*> findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names);
    static std::map<std::string, AActor*> findActorsByTagAsMap(const std::string& class_name, const UWorld* world, const std::string& tag);
    static std::map<std::string, AActor*> findActorsByTagAnyAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::map<std::string, AActor*> findActorsByTagAllAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::map<std::string, AActor*> findActorsByTypeAsMap(const std::string& class_name, const UWorld* world);
    static AActor* findActorByName(const std::string& class_name, const UWorld* world, const std::string& name, bool assert_if_not_found = true);
    static AActor* findActorByTag(const std::string& class_name, const UWorld* world, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static AActor* findActorByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static AActor* findActorByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static AActor* findActorByType(const std::string& class_name, const UWorld* world, bool assert_if_not_found = true, bool assert_if_multiple_found = true);

    //
    // Get components using a class name instead of template parameters
    //

    static std::vector<UActorComponent*> getComponentsByTag(const std::string& class_name, const AActor* actor, const std::string& tag);
    static std::vector<UActorComponent*> getComponentsByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags);
    static std::vector<UActorComponent*> getComponentsByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags);
    static std::vector<UActorComponent*> getComponentsByType(const std::string& class_name, const AActor* actor);
    static std::map<std::string, UActorComponent*> getComponentsByNameAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names);
    static std::map<std::string, UActorComponent*> getComponentsByTagAsMap(const std::string& class_name, const AActor* actor, const std::string& tag);
    static std::map<std::string, UActorComponent*> getComponentsByTagAnyAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags);
    static std::map<std::string, UActorComponent*> getComponentsByTagAllAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags);
    static std::map<std::string, UActorComponent*> getComponentsByTypeAsMap(const std::string& class_name, const AActor* actor);
    static UActorComponent* getComponentByName(const std::string& class_name, const AActor* actor, const std::string& name, bool assert_if_not_found = true);
    static UActorComponent* getComponentByTag(const std::string& class_name, const AActor* actor, const std::string& tag, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static UActorComponent* getComponentByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static UActorComponent* getComponentByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static UActorComponent* getComponentByType(const std::string& class_name, const AActor* actor, bool assert_if_not_found = true, bool assert_if_multiple_found = true);

    //
    // Register and unregister actor class
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

    static void unregisterActorClass(const std::string& class_name);

    //
    // Register and unregister component class
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

    static void unregisterComponentClass(const std::string& class_name);

private:

    //
    // A ClassRegistrar<TReturn, TArgs...> is a templated type that allows a caller to create objects by name
    // instead of by type. The user is responsible for registering create and destroy functions that are
    // specialized for each type that will be created. For example, a typical create function might call the
    // new operator on a derived type and return a base class pointer, and the corresponding destroy function
    // would call delete on the base class pointer.
    // 
    // All user-specified create functions that are registered with a registrar must have the same signature,
    // taking as input TArgs... and returning as output TReturn. Likewise, all user-specified destroy functions
    // registered with a registrar must take as input TReturn& and return void.
    //
    // This following example demonstrates a typical use case.
    // 
    //     ClassRegistrar<void*, int> my_registar;
    //
    // In this case, all create functions that are registered with my_registrar must take as input an int and
    // return as output void*. We can register a specific type to my_registrar by calling registerClass(...)
    // as follows.
    //
    //     my_registrar.registerClass(
    //         "float",                                                                    // type name
    //         [](int num_elements) -> void* { return new float[num_elements]; },          // create function
    //         [](void* created) -> void { delete[] reinterpret_cast<float*>(created); }); // destroy function
    //
    // Here, we are registering the name "float" with a create function that allocates an array of floats, and
    // a corresponding destroy function that deletes the array. In our destroy function, we need to explicitly
    // cast the void* pointer to float* because deleting a void* results in undefined behavior. After we have
    // registered our type, we can call our create and destroy functions using only the type's registered name.
    //
    //     void* my_ptr = my_registrar.create("float", 10); // create an array of 10 floats
    //     my_registrar.destroy("float", my_ptr);           // destroy the array
    //

    template <typename TReturn, typename... TArgs>
    class ClassRegistrar
    {
    public:
        void registerClass(const std::string& class_name, const std::function<TReturn(TArgs...)>& create_func, const std::function<void(TReturn&)> destroy_func)
        {
            SP_ASSERT(create_func);
            SP_ASSERT(destroy_func);
            Std::insert(create_funcs_, class_name, create_func);
            Std::insert(destroy_funcs_, class_name, destroy_func);
        }

        void registerClass(const std::string& class_name, const std::function<TReturn(TArgs...)>& create_func)
        {
            SP_ASSERT(create_func);
            Std::insert(create_funcs_, class_name, create_func);
        }

        void registerClass(const std::string& class_name, const std::function<void(TReturn&)> destroy_func)
        {
            SP_ASSERT(destroy_func);
            Std::insert(destroy_funcs_, class_name, destroy_func);
        }

        void unregisterClass(const std::string& class_name)
        {
            SP_ASSERT(Std::containsKey(create_funcs_, class_name) || Std::containsKey(destroy_funcs_, class_name));
            if (Std::containsKey(create_funcs_, class_name)) {
                Std::remove(create_funcs_, class_name);
            }
            if (Std::containsKey(destroy_funcs_, class_name)) {
                Std::remove(create_funcs_, class_name);
            }
        }

        TReturn create(const std::string& class_name, TArgs... args)
        {
            SP_ASSERT(Std::containsKey(create_funcs_, class_name));
            return create_funcs_.at(class_name)(args...);
        }

        void destroy(const std::string& class_name, TReturn& created)
        {
            SP_ASSERT(Std::containsKey(destroy_funcs_, class_name));
            destroy_funcs_.at(class_name)(created);
        }

    private:
        std::map<std::string, std::function<TReturn(TArgs...)>> create_funcs_;
        std::map<std::string, std::function<void(TReturn&)>> destroy_funcs_;
    };

    //
    // Registrars for finding actors using a class name instead of template parameters
    //

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

    //
    // Registrars for getting components using a class name instead of template parameters
    //

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
