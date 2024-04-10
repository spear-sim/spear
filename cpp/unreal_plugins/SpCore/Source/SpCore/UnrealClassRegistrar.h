//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function
#include <map>
#include <string>
#include <vector>

#include <Engine/World.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <UObject/NameTypes.h>      // FName
#include <UObject/ObjectMacros.h>   // ELoadFlags, EObjectFlags
#include <UObject/UObjectGlobals.h> // LoadObject, NewObject

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/CppFuncRegistrar.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class AActor;
class FLinkerInstancingContext;
class UActorComponent;
class UObject;
class UPackage;
class UPackageMap;
class USceneComponent;
class UStruct;
struct FActorSpawnParameters;
struct FObjectInstancingGraph;

class SPCORE_API UnrealClassRegistrar
{
public:
    UnrealClassRegistrar() = delete;
    ~UnrealClassRegistrar() = delete;

    static void initialize();
    static void terminate();

    //
    // Spawn actor using a class name instead of template parameters
    //

    static AActor* spawnActor(const std::string& class_name, UWorld* world, const FVector& location, const FRotator& rotation, const FActorSpawnParameters& spawn_parameters);

    //
    // Create component using a class name instead of template parameters
    //

    static UActorComponent* createComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& name);
    static USceneComponent* createSceneComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& name);
    static USceneComponent* createSceneComponentOutsideOwnerConstructor(const std::string& class_name, UObject* owner, USceneComponent* parent, const std::string& name);
    static USceneComponent* createSceneComponentOutsideOwnerConstructor(const std::string& class_name, USceneComponent* owner, const std::string& name);

    //
    // Create object using a class name instead of template parameters
    //

    static UObject* newObject(
        const std::string& class_name,
        UObject* outer,
        FName name = NAME_None,
        EObjectFlags flags = EObjectFlags::RF_NoFlags,
        UObject* uobject_template = nullptr,
        bool copy_transients_from_class_defaults = false,
        FObjectInstancingGraph* in_instance_graph = nullptr,
        UPackage* external_package = nullptr);

    static UObject* loadObject(
        const std::string& class_name,
        UObject* outer,
        const TCHAR* name,
        const TCHAR* filename = nullptr,
        uint32 load_flags = ELoadFlags::LOAD_None,
        UPackageMap* sandbox = nullptr,
        const FLinkerInstancingContext* instancing_context = nullptr);

    //
    // Get static class using a class name instead of template parameters
    //

    static UClass* getStaticClass(const std::string& class_name);

    //
    // Find actors using a class name instead of template parameters
    //

    static std::vector<AActor*> findActorsByName(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true);
    static std::vector<AActor*> findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag);
    static std::vector<AActor*> findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::vector<AActor*> findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::vector<AActor*> findActorsByType(const std::string& class_name, const UWorld* world);
    static std::map<std::string, AActor*> findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found = true);
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

    static std::vector<UActorComponent*> getComponentsByName(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found = true);
    static std::vector<UActorComponent*> getComponentsByTag(const std::string& class_name, const AActor* actor, const std::string& tag);
    static std::vector<UActorComponent*> getComponentsByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags);
    static std::vector<UActorComponent*> getComponentsByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags);
    static std::vector<UActorComponent*> getComponentsByType(const std::string& class_name, const AActor* actor);
    static std::map<std::string, UActorComponent*> getComponentsByNameAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found = true);
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
    // Get children components using a class name and an AActor* instead of template parameters
    //

    static std::vector<USceneComponent*> getChildrenComponentsByName(const std::string& class_name, const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTag(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAny(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAll(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByType(const std::string& class_name, const AActor* parent, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByNameAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAsMap(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAnyAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAllAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTypeAsMap(const std::string& class_name, const AActor* parent, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByName(const std::string& class_name, const AActor* parent, const std::string& name, bool include_all_descendants = true, bool assert_if_not_found = true);
    static USceneComponent* getChildComponentByTag(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static USceneComponent* getChildComponentByTagAny(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static USceneComponent* getChildComponentByTagAll(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static USceneComponent* getChildComponentByType(const std::string& class_name, const AActor* parent, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);

    //
    // Get children components using a class name and an USceneComponent* instead of template parameters
    //

    static std::vector<USceneComponent*> getChildrenComponentsByName(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByNameAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAsMap(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAnyAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAllAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTypeAsMap(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByName(const std::string& class_name, const USceneComponent* parent, const std::string& name, bool include_all_descendants = true, bool assert_if_not_found = true);
    static USceneComponent* getChildComponentByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static USceneComponent* getChildComponentByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static USceneComponent* getChildComponentByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);
    static USceneComponent* getChildComponentByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants = true, bool assert_if_not_found = true, bool assert_if_multiple_found = true);

    //
    // Register and unregister actor class
    //

    template <CActor TActor>
    static void registerActorClass(const std::string& class_name)
    {
        registerClassCommon<TActor>(class_name);

        //
        // Spawn actor
        //

        s_spawn_actor_registrar_.registerFunc(
            class_name, [](UWorld* world, const FVector& location, const FRotator& rotation, const FActorSpawnParameters& spawn_parameters) -> AActor* {
                SP_ASSERT(world); return world->SpawnActor<TActor>(location, rotation, spawn_parameters); });

        //
        // Find actors by name or tag or type and return an std::vector
        //

        s_find_actors_by_name_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) -> std::vector<AActor*> {
                return Unreal::findActorsByName<TActor, AActor>(world, names, return_null_if_not_found); });

        s_find_actors_by_tag_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::string& tag) -> std::vector<AActor*> {
                return Unreal::findActorsByTag<TActor, AActor>(world, tag); });

        s_find_actors_by_tag_any_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::vector<AActor*> {
                return Unreal::findActorsByTagAny<TActor, AActor>(world, tags); });

        s_find_actors_by_tag_all_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::vector<AActor*> {
                return Unreal::findActorsByTagAll<TActor, AActor>(world, tags); });

        s_find_actors_by_type_registrar_.registerFunc(
            class_name, [](const UWorld* world) -> std::vector<AActor*> {
                return Unreal::findActorsByType<TActor, AActor>(world); });

        //
        // Find actors by name or tag or type and return an std::map
        //

        s_find_actors_by_name_as_map_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByNameAsMap<TActor, AActor>(world, names, return_null_if_not_found); });

        s_find_actors_by_tag_as_map_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::string& tag) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAsMap<TActor, AActor>(world, tag); });
        
        s_find_actors_by_tag_any_as_map_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAnyAsMap<TActor, AActor>(world, tags); });
        
        s_find_actors_by_tag_all_as_map_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAllAsMap<TActor, AActor>(world, tags); });
        
        s_find_actors_by_type_as_map_registrar_.registerFunc(
            class_name, [](const UWorld* world) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTypeAsMap<TActor, AActor>(world); });
        
        //
        // Find actor by name or tag or type and return a pointer
        //

        s_find_actor_by_name_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::string& name, bool assert_if_not_found) -> AActor* {
                return Unreal::findActorByName<TActor, AActor>(world, name, assert_if_not_found); });
        
        s_find_actor_by_tag_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByTag<TActor, AActor>(world, tag, assert_if_not_found, assert_if_multiple_found); });
        
        s_find_actor_by_tag_any_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByTagAny<TActor, AActor>(world, tags, assert_if_not_found, assert_if_multiple_found); });
        
        s_find_actor_by_tag_all_registrar_.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByTagAll<TActor, AActor>(world, tags, assert_if_not_found, assert_if_multiple_found); });
        
        s_find_actor_by_type_registrar_.registerFunc(
            class_name, [](const UWorld* world, bool assert_if_not_found, bool assert_if_multiple_found) -> AActor* {
                return Unreal::findActorByType<TActor, AActor>(world, assert_if_not_found, assert_if_multiple_found); });
    }

    template <CComponent TComponent>
    static void registerComponentClass(const std::string& class_name)
    {
        registerComponentClassCommon<TComponent>(class_name);

        //
        // Create component
        //

        s_create_component_outside_owner_constructor_registrar_.registerFunc(
            class_name, [](AActor* owner, const std::string& name) -> UActorComponent* {
                return Unreal::createComponentOutsideOwnerConstructor<TComponent, UActorComponent>(owner, name); });
    }

    template <CSceneComponent TSceneComponent>
    static void registerComponentClass(const std::string& class_name)
    {
        registerComponentClassCommon<TSceneComponent>(class_name);

        //
        // Create component
        //

        s_create_scene_component_outside_owner_constructor_from_actor_registrar_.registerFunc(
            class_name, [](AActor* owner, const std::string& name) -> USceneComponent* {
                return Unreal::createComponentOutsideOwnerConstructor<TSceneComponent, USceneComponent>(owner, name); });

        s_create_scene_component_outside_owner_constructor_from_object_registrar_.registerFunc(
            class_name, [](UObject* owner, USceneComponent* parent, const std::string& name) -> USceneComponent* {
                return Unreal::createComponentOutsideOwnerConstructor<TSceneComponent, USceneComponent>(owner, parent, name); });

        s_create_scene_component_outside_owner_constructor_from_scene_component_registrar_.registerFunc(
            class_name, [](USceneComponent* owner, const std::string& name) -> USceneComponent* {
                return Unreal::createComponentOutsideOwnerConstructor<TSceneComponent, USceneComponent>(owner, name); });

        //
        // Get children components by name or tag or type from an AActor* and return an std::vector
        //

        s_get_children_components_by_name_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByName<AActor, TSceneComponent, USceneComponent>(parent, names, include_all_descendants, return_null_if_not_found); });

        s_get_children_components_by_tag_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::string& tag, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTag<AActor, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants); });

        s_get_children_components_by_tag_any_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAny<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        s_get_children_components_by_tag_all_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAll<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        s_get_children_components_by_type_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByType<TSceneComponent, USceneComponent>(parent, include_all_descendants); });

        //
        // Get children components by name or tag or type from an AActor* and return an std::map
        //

        s_get_children_components_by_name_as_map_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByNameAsMap<AActor, TSceneComponent, USceneComponent>(parent, names, include_all_descendants, return_null_if_not_found);
            });

        s_get_children_components_by_tag_as_map_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::string& tag, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAsMap<AActor, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants);
            });

        s_get_children_components_by_tag_any_as_map_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAnyAsMap<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        s_get_children_components_by_tag_all_as_map_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAllAsMap<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        s_get_children_components_by_type_as_map_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTypeAsMap<AActor, TSceneComponent, USceneComponent>(parent, include_all_descendants);
            });

        //
        // Get child component by name or tag or type from an AActor* and return a pointer
        //

        s_get_child_component_by_name_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::string& name, bool include_all_descendants, bool assert_if_not_found) -> USceneComponent* {
                return Unreal::getChildComponentByName<AActor, TSceneComponent, USceneComponent>(parent, name, include_all_descendants, assert_if_not_found);
            });

        s_get_child_component_by_tag_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::string& tag, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByTag<AActor, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });

        s_get_child_component_by_tag_any_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByTagAny<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });

        s_get_child_component_by_tag_all_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByTagAll<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });

        s_get_child_component_by_type_from_actor_registrar_.registerFunc(
            class_name, [](const AActor* parent, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByType<AActor, TSceneComponent, USceneComponent>(parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });

        //
        // Get children components by name or tag or type from a USceneComponent* and return an std::vector
        //

        s_get_children_components_by_name_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByName<USceneComponent, TSceneComponent, USceneComponent>(parent, names, include_all_descendants, return_null_if_not_found); });

        s_get_children_components_by_tag_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& tag, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTag<USceneComponent, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants); });

        s_get_children_components_by_tag_any_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAny<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        s_get_children_components_by_tag_all_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAll<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        s_get_children_components_by_type_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByType<TSceneComponent, USceneComponent>(parent, include_all_descendants); });

        //
        // Get children components by name or tag or type from a USceneComponent* and return an std::map
        //

        s_get_children_components_by_name_as_map_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByNameAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, names, include_all_descendants, return_null_if_not_found);
            });

        s_get_children_components_by_tag_as_map_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& tag, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants);
            });

        s_get_children_components_by_tag_any_as_map_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAnyAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        s_get_children_components_by_tag_all_as_map_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAllAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        s_get_children_components_by_type_as_map_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTypeAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, include_all_descendants);
            });

        //
        // Get child component by name or tag or type from a USceneComponent* and return a pointer
        //

        s_get_child_component_by_name_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& name, bool include_all_descendants, bool assert_if_not_found) -> USceneComponent* {
                return Unreal::getChildComponentByName<USceneComponent, TSceneComponent, USceneComponent>(parent, name, include_all_descendants, assert_if_not_found);
            });

        s_get_child_component_by_tag_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& tag, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByTag<USceneComponent, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });

        s_get_child_component_by_tag_any_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByTagAny<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });

        s_get_child_component_by_tag_all_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByTagAll<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });

        s_get_child_component_by_type_from_scene_component_registrar_.registerFunc(
            class_name, [](const USceneComponent* parent, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) -> USceneComponent* {
                return Unreal::getChildComponentByType<USceneComponent, TSceneComponent, USceneComponent>(parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
            });
    }

    //
    // Register and unregister component class
    //

    template <CComponent TComponent>
    static void registerComponentClassCommon(const std::string& class_name)
    {
        registerClassCommon<TComponent>(class_name);

        //
        // Get components by name or tag or type and return an std::vector
        //

        s_get_components_by_name_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByName<TComponent, UActorComponent>(actor, names, return_null_if_not_found); });

        s_get_components_by_tag_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::string& tag) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTag<TComponent, UActorComponent>(actor, tag); });

        s_get_components_by_tag_any_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTagAny<TComponent, UActorComponent>(actor, tags); });

        s_get_components_by_tag_all_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTagAll<TComponent, UActorComponent>(actor, tags); });

        s_get_components_by_type_registrar_.registerFunc(
            class_name, [](const AActor* actor) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByType<TComponent, UActorComponent>(actor); });

        //
        // Get components by name or tag or type and return an std::map
        //

        s_get_components_by_name_as_map_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByNameAsMap<TComponent, UActorComponent>(actor, names, return_null_if_not_found); });

        s_get_components_by_tag_as_map_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::string& tag) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAsMap<TComponent, UActorComponent>(actor, tag); });
        
        s_get_components_by_tag_any_as_map_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAnyAsMap<TComponent, UActorComponent>(actor, tags); });
        
        s_get_components_by_tag_all_as_map_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAllAsMap<TComponent, UActorComponent>(actor, tags); });
        
        s_get_components_by_type_as_map_registrar_.registerFunc(
            class_name, [](const AActor* actor) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTypeAsMap<TComponent, UActorComponent>(actor); });

        //
        // Get component by name or tag or type and return a pointer
        //

        s_get_component_by_name_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::string& name, bool assert_if_not_found) -> UActorComponent* {
                return Unreal::getComponentByName<TComponent, UActorComponent>(actor, name, assert_if_not_found); });
        
        s_get_component_by_tag_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByTag<TComponent, UActorComponent>(actor, tag, assert_if_not_found, assert_if_multiple_found); });
        
        s_get_component_by_tag_any_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByTagAny<TComponent, UActorComponent>(actor, tags, assert_if_not_found, assert_if_multiple_found); });
        
        s_get_component_by_tag_all_registrar_.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByTagAll<TComponent, UActorComponent>(actor, tags, assert_if_not_found, assert_if_multiple_found); });
        
        s_get_component_by_type_registrar_.registerFunc(
            class_name, [](const AActor* actor, bool assert_if_not_found, bool assert_if_multiple_found) -> UActorComponent* {
                return Unreal::getComponentByType<TComponent, UActorComponent>(actor, assert_if_not_found, assert_if_multiple_found); });
    }

    template <CClass TClass>
    static void registerClass(const std::string& class_name)
    {
        registerClassCommon<TClass>(class_name);

        //
        // Create object
        //

        s_new_object_registrar_.registerFunc(
            class_name, [](UObject* outer, FName name, EObjectFlags flags, UObject* uobject_template, bool copy_transients_from_class_defaults, FObjectInstancingGraph* in_instance_graph, UPackage* external_package) -> UObject* {
                return NewObject<UObject>(outer, TClass::StaticClass(), name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph, external_package);
            });

        s_load_object_registrar_.registerFunc(
            class_name, [](UObject* outer, const TCHAR* name, const TCHAR* filename, uint32 load_flags, UPackageMap* sandbox, const FLinkerInstancingContext* instancing_context) -> UObject* {
                return LoadObject<TClass>(outer, name, filename, load_flags, sandbox, instancing_context);
            });
    }


    template <CClass TClass>
    static void registerClassCommon(const std::string& class_name)
    {
        //
        // Get static class
        //

        s_get_static_class_registrar_.registerFunc(
            class_name, []() -> UClass* {
                return TClass::StaticClass();
            });
    }

    template <CActor TActor>
    static void unregisterActorClass(const std::string& class_name)
    {
        unregisterClassCommon<TActor>(class_name);

        s_spawn_actor_registrar_.unregisterFunc(class_name);

        s_find_actors_by_name_registrar_.unregisterFunc(class_name);
        s_find_actors_by_tag_registrar_.unregisterFunc(class_name);
        s_find_actors_by_tag_any_registrar_.unregisterFunc(class_name);
        s_find_actors_by_tag_all_registrar_.unregisterFunc(class_name);
        s_find_actors_by_type_registrar_.unregisterFunc(class_name);
        s_find_actors_by_name_as_map_registrar_.unregisterFunc(class_name);
        s_find_actors_by_tag_as_map_registrar_.unregisterFunc(class_name);
        s_find_actors_by_tag_any_as_map_registrar_.unregisterFunc(class_name);
        s_find_actors_by_tag_all_as_map_registrar_.unregisterFunc(class_name);
        s_find_actors_by_type_as_map_registrar_.unregisterFunc(class_name);
        s_find_actor_by_name_registrar_.unregisterFunc(class_name);
        s_find_actor_by_tag_registrar_.unregisterFunc(class_name);
        s_find_actor_by_tag_any_registrar_.unregisterFunc(class_name);
        s_find_actor_by_tag_all_registrar_.unregisterFunc(class_name);
        s_find_actor_by_type_registrar_.unregisterFunc(class_name);
    }

    template <CComponent TComponent>
    static void unregisterComponentClass(const std::string& class_name)
    {
        unregisterComponentClassCommon<TComponent>(class_name);

        s_create_component_outside_owner_constructor_registrar_.unregisterFunc(class_name);
    }

    template <CSceneComponent TSceneComponent>
    static void unregisterComponentClass(const std::string& class_name)
    {
        unregisterComponentClassCommon<TSceneComponent>(class_name);

        s_create_scene_component_outside_owner_constructor_from_actor_registrar_.unregisterFunc(class_name);
        s_create_scene_component_outside_owner_constructor_from_object_registrar_.unregisterFunc(class_name);
        s_create_scene_component_outside_owner_constructor_from_scene_component_registrar_.unregisterFunc(class_name);

        s_get_children_components_by_name_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_any_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_all_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_type_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_name_as_map_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_as_map_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_any_as_map_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_all_as_map_from_actor_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_type_as_map_from_actor_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_name_from_actor_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_tag_from_actor_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_tag_any_from_actor_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_tag_all_from_actor_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_type_from_actor_registrar_.unregisterFunc(class_name);

        s_get_children_components_by_name_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_any_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_all_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_type_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_name_as_map_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_as_map_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_any_as_map_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_tag_all_as_map_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_children_components_by_type_as_map_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_name_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_tag_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_tag_any_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_tag_all_from_scene_component_registrar_.unregisterFunc(class_name);
        s_get_child_component_by_type_from_scene_component_registrar_.unregisterFunc(class_name);
    }


    template <CComponent TComponent>
    static void unregisterComponentClassCommon(const std::string& class_name)
    {
        unregisterClassCommon<TComponent>(class_name);

        s_get_components_by_name_registrar_.unregisterFunc(class_name);
        s_get_components_by_tag_registrar_.unregisterFunc(class_name);
        s_get_components_by_tag_any_registrar_.unregisterFunc(class_name);
        s_get_components_by_tag_all_registrar_.unregisterFunc(class_name);
        s_get_components_by_type_registrar_.unregisterFunc(class_name);
        s_get_components_by_name_as_map_registrar_.unregisterFunc(class_name);
        s_get_components_by_tag_as_map_registrar_.unregisterFunc(class_name);
        s_get_components_by_tag_any_as_map_registrar_.unregisterFunc(class_name);
        s_get_components_by_tag_all_as_map_registrar_.unregisterFunc(class_name);
        s_get_components_by_type_as_map_registrar_.unregisterFunc(class_name);
        s_get_component_by_name_registrar_.unregisterFunc(class_name);
        s_get_component_by_tag_registrar_.unregisterFunc(class_name);
        s_get_component_by_tag_any_registrar_.unregisterFunc(class_name);
        s_get_component_by_tag_all_registrar_.unregisterFunc(class_name);
        s_get_component_by_type_registrar_.unregisterFunc(class_name);
    }

    template <CClass TClass>
    static void unregisterClass(const std::string& class_name)
    {
        unregisterClassCommon<TClass>(class_name);

        s_new_object_registrar_.unregisterFunc(class_name);
        s_load_object_registrar_.unregisterFunc(class_name);
    }

    template <CClass TClass>
    static void unregisterClassCommon(const std::string& class_name)
    {
        s_get_static_class_registrar_.unregisterFunc(class_name);
    }

    //
    // The functions below are required to support the getStaticStruct<T>() method in cases where an otherwise
    // well-formed struct type doesn't define a StaticStruct() method, as is the case with FRotator and FVector.
    // See the following file for more examples of similar special struct types:
    //     Engine/Source/CoreUObject/Public/UObject/NoExportTypes.h
    //
    // For the getStaticStruct<T>() method to behave as expected for a type T, the type must meet the following
    // conditions. First, the type needs to be registered by calling registerSpecialStruct<T>(...) before calling
    // getStaticStruct<T>(). If the type is registered by UStruct*, then there are no other requirements. If the
    // type is registered by name, then getStaticStruct<T>() will call Unreal::findSpecialStructByName(...)
    // internally to find the appropriate UStruct*. For a type to be visible to Unreal::findSpecialStructByName(...),
    // ASpCoreActor must define a UPROPERTY of type T that is named according to a particular naming convention.
    // When a type no longer needs to be visible to getStaticStruct<T>(), it should be unregistered by calling
    // UnrealClassRegistrar::unregisterSpecialStruct<T>().
    // 
    // Registering special structs by name is more convenient because it doesn't require the caller to obtain a
    // valid UStruct*, and can therefore be done any time during a program's execution. On the other hand,
    // registering special structs by UStruct* is is more flexible because it enables the caller to register
    // special structs that are not registered by default in this class.
    //

    template <typename TSpecialStruct> requires (!CStruct<TSpecialStruct>)
    static void registerSpecialStruct(const std::string& struct_name)
    {
        std::string type_id_string = getTypeIdString<TSpecialStruct>();
        SP_ASSERT(!Std::containsKey(s_special_struct_names_, type_id_string));
        SP_ASSERT(!Std::containsKey(s_special_structs_, type_id_string));
        Std::insert(s_special_struct_names_, type_id_string, struct_name);
    }

    template <typename TSpecialStruct> requires (!CStruct<TSpecialStruct>)
    static void registerSpecialStruct(UStruct* ustruct)
    {
        SP_ASSERT(ustruct);
        std::string type_id_string = getTypeIdString<TSpecialStruct>();
        SP_ASSERT(!Std::containsKey(s_special_struct_names_, type_id_string));
        SP_ASSERT(!Std::containsKey(s_special_structs_, type_id_string));
        Std::insert(s_special_structs_, type_id_string, ustruct);
    }

    template <typename TSpecialStruct> requires (!CStruct<TSpecialStruct>)
    static void unregisterSpecialStruct()
    {
        std::string type_id_string = getTypeIdString<TSpecialStruct>();
        SP_ASSERT(Std::containsKey(s_special_struct_names_, type_id_string) + Std::containsKey(s_special_structs_, type_id_string) == 1);
        if (Std::containsKey(s_special_struct_names_, type_id_string)) {
            Std::remove(s_special_struct_names_, type_id_string);
        }
        if (Std::containsKey(s_special_structs_, type_id_string)) {
            Std::remove(s_special_structs_, type_id_string);
        }
    }

    template <CStruct TStruct>
    static UStruct* getStaticStruct()
    {
        return TStruct::StaticStruct();
    }

    template <typename TSpecialStruct> requires (!CStruct<TSpecialStruct>)
    static UStruct* getStaticStruct()
    {
        std::string type_id_string = getTypeIdString<TSpecialStruct>();
        SP_ASSERT(Std::containsKey(s_special_struct_names_, type_id_string) + Std::containsKey(s_special_structs_, type_id_string) == 1);
        if (Std::containsKey(s_special_struct_names_, type_id_string)) {
            std::string name = s_special_struct_names_.at(type_id_string);
            return Unreal::findSpecialStructByName(name);
        }
        if (Std::containsKey(s_special_structs_, type_id_string)) {
            return s_special_structs_.at(type_id_string);
        }
        return nullptr;
    }

private:

    template <typename T>
    static const char* getTypeIdString()
    {
        // RTTI is not allowed in modules that define Unreal types, so we can't use typeid(T). We also can't use
        // use boost::typeindex::type_id<T>, which is intended to emulate RTTI without actually enabling it, because
        // this conflicts with some Unreal modules that explicitly enable RTTI. So we use BOOST_CURRENT_FUNCTION
        // as a lightweight alternative because it will give us a unique string for each type, and that is the only
        // type information we need here.
        return BOOST_CURRENT_FUNCTION;
    }

    //
    // These maps are necessary to support getStaticStruct<T>() for special struct types that don't have a
    // StaticStruct() method., e.g., FRotator and FVector.
    //

    inline static std::map<std::string, std::string> s_special_struct_names_; // map from platform-dependent type name to user-facing name
    inline static std::map<std::string, UStruct*> s_special_structs_;         // map from platform-dependent type name to UStruct*

    //
    // Registrars for spawning actors using a class name instead of template parameters
    //

    inline static CppFuncRegistrar<AActor*, UWorld*, const FVector&, const FRotator&, const FActorSpawnParameters&> s_spawn_actor_registrar_;

    //
    // Registrars for creating components using a class name instead of template parameters
    //

    inline static CppFuncRegistrar<UActorComponent*, AActor*, const std::string&>                    s_create_component_outside_owner_constructor_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, AActor*, const std::string&>                    s_create_scene_component_outside_owner_constructor_from_actor_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, UObject*, USceneComponent*, const std::string&> s_create_scene_component_outside_owner_constructor_from_object_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, USceneComponent*, const std::string&>           s_create_scene_component_outside_owner_constructor_from_scene_component_registrar_;

    //
    // Registrars for getting static classes using a class name instead of template parameters
    //

    inline static CppFuncRegistrar<UClass*> s_get_static_class_registrar_;

    //
    // Registrars for creating objects using a class name instead of template parameters
    //

    inline static CppFuncRegistrar<UObject*, UObject*, FName, EObjectFlags, UObject*, bool, FObjectInstancingGraph*, UPackage*> s_new_object_registrar_;
    inline static CppFuncRegistrar<UObject*, UObject*, const TCHAR*, const TCHAR*, uint32, UPackageMap*, const FLinkerInstancingContext*> s_load_object_registrar_;

    //
    // Registrars for finding actors using a class name instead of template parameters
    //

    inline static CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&, bool>           s_find_actors_by_name_registrar_;
    inline static CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::string&>                              s_find_actors_by_tag_registrar_;
    inline static CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 s_find_actors_by_tag_any_registrar_;
    inline static CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 s_find_actors_by_tag_all_registrar_;
    inline static CppFuncRegistrar<std::vector<AActor*>, const UWorld*>                                                  s_find_actors_by_type_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&, bool> s_find_actors_by_name_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::string&>                    s_find_actors_by_tag_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       s_find_actors_by_tag_any_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       s_find_actors_by_tag_all_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*>                                        s_find_actors_by_type_as_map_registrar_;
    inline static CppFuncRegistrar<AActor*, const UWorld*, const std::string&, bool>                                     s_find_actor_by_name_registrar_;
    inline static CppFuncRegistrar<AActor*, const UWorld*, const std::string&, bool, bool>                               s_find_actor_by_tag_registrar_;
    inline static CppFuncRegistrar<AActor*, const UWorld*, const std::vector<std::string>&, bool, bool>                  s_find_actor_by_tag_any_registrar_;
    inline static CppFuncRegistrar<AActor*, const UWorld*, const std::vector<std::string>&, bool, bool>                  s_find_actor_by_tag_all_registrar_;
    inline static CppFuncRegistrar<AActor*, const UWorld*, bool, bool>                                                   s_find_actor_by_type_registrar_;

    //
    // Registrars for getting components using a class name instead of template parameters
    //

    inline static CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>           s_get_components_by_name_registrar_;
    inline static CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::string&>                              s_get_components_by_tag_registrar_;
    inline static CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&>                 s_get_components_by_tag_any_registrar_;
    inline static CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&>                 s_get_components_by_tag_all_registrar_;
    inline static CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*>                                                  s_get_components_by_type_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool> s_get_components_by_name_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::string&>                    s_get_components_by_tag_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&>       s_get_components_by_tag_any_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&>       s_get_components_by_tag_all_as_map_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*>                                        s_get_components_by_type_as_map_registrar_;
    inline static CppFuncRegistrar<UActorComponent*, const AActor*, const std::string&, bool>                                     s_get_component_by_name_registrar_;
    inline static CppFuncRegistrar<UActorComponent*, const AActor*, const std::string&, bool, bool>                               s_get_component_by_tag_registrar_;
    inline static CppFuncRegistrar<UActorComponent*, const AActor*, const std::vector<std::string>&, bool, bool>                  s_get_component_by_tag_any_registrar_;
    inline static CppFuncRegistrar<UActorComponent*, const AActor*, const std::vector<std::string>&, bool, bool>                  s_get_component_by_tag_all_registrar_;
    inline static CppFuncRegistrar<UActorComponent*, const AActor*, bool, bool>                                                   s_get_component_by_type_registrar_;

    //
    // Registrars for getting children components using a class name instead of template parameters
    //

    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           s_get_children_components_by_name_from_actor_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::string&, bool>                              s_get_children_components_by_tag_from_actor_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 s_get_children_components_by_tag_any_from_actor_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 s_get_children_components_by_tag_all_from_actor_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, bool>                                                  s_get_children_components_by_type_from_actor_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> s_get_children_components_by_name_as_map_from_actor_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::string&, bool>                    s_get_children_components_by_tag_as_map_from_actor_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       s_get_children_components_by_tag_any_as_map_from_actor_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       s_get_children_components_by_tag_all_as_map_from_actor_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, bool>                                        s_get_children_components_by_type_as_map_from_actor_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const AActor*, const std::string&, bool, bool>                                     s_get_child_component_by_name_from_actor_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const AActor*, const std::string&, bool, bool, bool>                               s_get_child_component_by_tag_from_actor_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const AActor*, const std::vector<std::string>&, bool, bool, bool>                  s_get_child_component_by_tag_any_from_actor_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const AActor*, const std::vector<std::string>&, bool, bool, bool>                  s_get_child_component_by_tag_all_from_actor_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const AActor*, bool, bool, bool>                                                   s_get_child_component_by_type_from_actor_registrar_;

    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool>           s_get_children_components_by_name_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::string&, bool>                              s_get_children_components_by_tag_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 s_get_children_components_by_tag_any_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 s_get_children_components_by_tag_all_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, bool>                                                  s_get_children_components_by_type_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool> s_get_children_components_by_name_as_map_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::string&, bool>                    s_get_children_components_by_tag_as_map_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       s_get_children_components_by_tag_any_as_map_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       s_get_children_components_by_tag_all_as_map_from_scene_component_registrar_;
    inline static CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, bool>                                        s_get_children_components_by_type_as_map_from_scene_component_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::string&, bool, bool>                                     s_get_child_component_by_name_from_scene_component_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::string&, bool, bool, bool>                               s_get_child_component_by_tag_from_scene_component_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool, bool, bool>                  s_get_child_component_by_tag_any_from_scene_component_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool, bool, bool>                  s_get_child_component_by_tag_all_from_scene_component_registrar_;
    inline static CppFuncRegistrar<USceneComponent*, const USceneComponent*, bool, bool, bool>                                                   s_get_child_component_by_type_from_scene_component_registrar_;
};
