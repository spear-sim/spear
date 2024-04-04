//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealClassRegistrar.h"

#include <map>
#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMeshActor.h>

#include "SpCore/EngineActor.h"

class AActor;
class UWorld;

// Normally we would do the operations in initialize() and terminate(...) in the opposite order. But we make an
// exception here (i.e., we do the operations in the same order) to make it easier and less error-prone to add
// classes and functions.

void UnrealClassRegistrar::initialize()
{
    // Unreal classes
    registerActorClass<AStaticMeshActor>("AStaticMeshActor");
    registerComponentClass<UStaticMeshComponent>("UStaticMeshComponent");

    // SpCore classes
    registerActorClass<AEngineActor>("AEngineActor");
}

void UnrealClassRegistrar::terminate()
{
    // Unreal classes
    unregisterActorClass<AStaticMeshActor>("AStaticMeshActor");
    unregisterComponentClass<UStaticMeshComponent>("UStaticMeshComponent");

    // SpCore classes
    unregisterActorClass<AEngineActor>("AEngineActor");
}

//
// Find actors using a class name instead of template parameters
//

std::vector<AActor*> UnrealClassRegistrar::findActorsByName(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return s_find_actors_by_name_registrar_.create(class_name, world, names, return_null_if_not_found);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return s_find_actors_by_tag_registrar_.create(class_name, world, tag);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_any_registrar_.create(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_all_registrar_.create(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByType(const std::string& class_name, const UWorld* world) {
    return s_find_actors_by_type_registrar_.create(class_name, world);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names) {
    return s_find_actors_by_name_as_map_registrar_.create(class_name, world, names);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAsMap(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return s_find_actors_by_tag_as_map_registrar_.create(class_name, world, tag);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAnyAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_any_as_map_registrar_.create(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAllAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_all_as_map_registrar_.create(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTypeAsMap(const std::string& class_name, const UWorld* world) {
    return s_find_actors_by_type_as_map_registrar_.create(class_name, world);
}
    
AActor* UnrealClassRegistrar::findActorByName(const std::string& class_name, const UWorld* world, const std::string& name, bool assert_if_not_found) {
    return s_find_actor_by_name_registrar_.create(class_name, world, name, assert_if_not_found);
}
    
AActor* UnrealClassRegistrar::findActorByTag(const std::string& class_name, const UWorld* world, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_tag_registrar_.create(class_name, world, tag, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_tag_any_registrar_.create(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_tag_all_registrar_.create(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByType(const std::string& class_name, const UWorld* world, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_type_registrar_.create(class_name, world, assert_if_not_found, assert_if_multiple_found);
}

//
// Get components using a class name instead of template parameters
//

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByName(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return s_get_components_by_name_registrar_.create(class_name, actor, names, return_null_if_not_found);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTag(const std::string& class_name, const AActor* actor, const std::string& tag) {
    return s_get_components_by_tag_registrar_.create(class_name, actor, tag);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_any_registrar_.create(class_name, actor, tags);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_all_registrar_.create(class_name, actor, tags);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByType(const std::string& class_name, const AActor* actor) {
    return s_get_components_by_type_registrar_.create(class_name, actor);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByNameAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names) {
    return s_get_components_by_name_as_map_registrar_.create(class_name, actor, names);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAsMap(const std::string& class_name, const AActor* actor, const std::string& tag) {
    return s_get_components_by_tag_as_map_registrar_.create(class_name, actor, tag);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAnyAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_any_as_map_registrar_.create(class_name, actor, tags);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAllAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_all_as_map_registrar_.create(class_name, actor, tags);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTypeAsMap(const std::string& class_name, const AActor* actor) {
    return s_get_components_by_type_as_map_registrar_.create(class_name, actor);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByName(const std::string& class_name, const AActor* actor, const std::string& name, bool assert_if_not_found) {
    return s_get_component_by_name_registrar_.create(class_name, actor, name, assert_if_not_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTag(const std::string& class_name, const AActor* actor, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_tag_registrar_.create(class_name, actor, tag, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_tag_any_registrar_.create(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_tag_all_registrar_.create(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByType(const std::string& class_name, const AActor* actor, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_type_registrar_.create(class_name, actor, assert_if_not_found, assert_if_multiple_found);
}

//
// Get children components using a class name instead of template parameters
//

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByName(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return s_get_children_components_by_name_registrar_.create(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return s_get_children_components_by_tag_registrar_.create(class_name, parent, tag, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_any_registrar_.create(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_all_registrar_.create(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return s_get_children_components_by_type_registrar_.create(class_name, parent, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByNameAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants) {
    return s_get_children_components_by_name_as_map_registrar_.create(class_name, parent, names, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAsMap(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return s_get_children_components_by_tag_as_map_registrar_.create(class_name, parent, tag, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_any_as_map_registrar_.create(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_all_as_map_registrar_.create(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return s_get_children_components_by_type_as_map_registrar_.create(class_name, parent, include_all_descendants);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByName(const std::string& class_name, const USceneComponent* parent, const std::string& name, bool include_all_descendants, bool assert_if_not_found) {
    return s_get_child_component_by_name_registrar_.create(class_name, parent, name, include_all_descendants, assert_if_not_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_registrar_.create(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_any_registrar_.create(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_all_registrar_.create(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_type_registrar_.create(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
