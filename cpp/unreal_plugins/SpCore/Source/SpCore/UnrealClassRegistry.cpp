//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealClassRegistry.h"

#include <map>
#include <string>
#include <vector>

#include <HAL/Platform.h> // uint32

#include "SpCore/FuncRegistry.h"

class AActor;
class FLinkerInstancingContext;
class UActorComponent;
class UClass;
class UEngineSubsystem;
class UObject;
class UPackage;
class UPackageMap;
class USceneComponent;
class UStruct;
class USubsystem;
class UWorld;
struct FActorSpawnParameters;
struct FObjectInstancingGraph;

//
// We need the variables below to be globals because they are referenced in templated code. This requirement
// arises because templated code gets compiled at each call site. If a call site is in a different module
// (i.e., outside of SpCore), and it references a static variable, then the module will get its own local
// copy of the static variable. This behavior only happens on Clang, because MSVC has a different methodology
// for handling static variables in shared libraries. See the link below for details:
//     https://stackoverflow.com/questions/31495877/i-receive-different-results-on-unix-and-win-when-use-static-members-with-static
//

//
// Registrys for getting subsystems using a class name instead of template parameters
//

FuncRegistry<UEngineSubsystem*>             g_get_engine_subsystem_by_type_func_registry;
FuncRegistry<USubsystem*, UWorld*>          g_get_subsystem_by_type_func_registry;
FuncRegistry<USubsystem*, UWorld*, UClass*> g_get_subsystem_by_class_func_registry;

//
// Registrys for getting static classes using a class name instead of template parameters
//

FuncRegistry<UClass*> g_get_static_class_func_registry;
FuncRegistry<UStruct*> g_get_static_struct_func_registry;

//
// Registrys for finding actors using a class name instead of template parameters
//

FuncRegistry<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&, bool>           g_find_actors_by_name_func_registry;
FuncRegistry<std::vector<AActor*>, const UWorld*, const std::string&>                              g_find_actors_by_tag_func_registry;
FuncRegistry<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 g_find_actors_by_tag_any_func_registry;
FuncRegistry<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 g_find_actors_by_tag_all_func_registry;
FuncRegistry<std::vector<AActor*>, const UWorld*>                                                  g_find_actors_by_type_func_registry;

FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&, bool> g_find_actors_by_name_as_map_func_registry;
FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::string&>                    g_find_actors_by_tag_as_map_func_registry;
FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       g_find_actors_by_tag_any_as_map_func_registry;
FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       g_find_actors_by_tag_all_as_map_func_registry;
FuncRegistry<std::map<std::string, AActor*>, const UWorld*>                                        g_find_actors_by_type_as_map_func_registry;

FuncRegistry<AActor*, const UWorld*, const std::string&>                                           g_find_actor_by_name_func_registry;
FuncRegistry<AActor*, const UWorld*, const std::string&>                                           g_find_actor_by_tag_func_registry;
FuncRegistry<AActor*, const UWorld*, const std::vector<std::string>&>                              g_find_actor_by_tag_any_func_registry;
FuncRegistry<AActor*, const UWorld*, const std::vector<std::string>&>                              g_find_actor_by_tag_all_func_registry;
FuncRegistry<AActor*, const UWorld*>                                                               g_find_actor_by_type_func_registry;

//
// Registrys for getting components using a class name instead of template parameters
//

FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_components_by_name_func_registry;
FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_components_by_path_func_registry;
FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::string&, bool>                              g_get_components_by_tag_func_registry;
FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_components_by_tag_any_func_registry;
FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_components_by_tag_all_func_registry;
FuncRegistry<std::vector<UActorComponent*>, const AActor*, bool>                                                  g_get_components_by_type_func_registry;

FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_components_by_name_as_map_func_registry;
FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_components_by_path_as_map_func_registry;
FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::string&, bool>                    g_get_components_by_tag_as_map_func_registry;
FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_components_by_tag_any_as_map_func_registry;
FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_components_by_tag_all_as_map_func_registry;
FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, bool>                                        g_get_components_by_type_as_map_func_registry;

FuncRegistry<UActorComponent*, const AActor*, const std::string&, bool>                                           g_get_component_by_name_func_registry;
FuncRegistry<UActorComponent*, const AActor*, const std::string&, bool>                                           g_get_component_by_path_func_registry;
FuncRegistry<UActorComponent*, const AActor*, const std::string&, bool>                                           g_get_component_by_tag_func_registry;
FuncRegistry<UActorComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_component_by_tag_any_func_registry;
FuncRegistry<UActorComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_component_by_tag_all_func_registry;
FuncRegistry<UActorComponent*, const AActor*, bool>                                                               g_get_component_by_type_func_registry;

//
// Registrys for getting children components using a class name instead of template parameters
//

FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_children_components_by_name_from_actor_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::string&, bool>                              g_get_children_components_by_tag_from_actor_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_any_from_actor_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_all_from_actor_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const AActor*, bool>                                                  g_get_children_components_by_type_from_actor_func_registry;

FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_children_components_by_name_as_map_from_actor_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::string&, bool>                    g_get_children_components_by_tag_as_map_from_actor_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_any_as_map_from_actor_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_all_as_map_from_actor_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, bool>                                        g_get_children_components_by_type_as_map_from_actor_func_registry;

FuncRegistry<USceneComponent*, const AActor*, const std::string&, bool>                                           g_get_child_component_by_name_from_actor_func_registry;
FuncRegistry<USceneComponent*, const AActor*, const std::string&, bool>                                           g_get_child_component_by_tag_from_actor_func_registry;
FuncRegistry<USceneComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_any_from_actor_func_registry;
FuncRegistry<USceneComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_all_from_actor_func_registry;
FuncRegistry<USceneComponent*, const AActor*, bool>                                                               g_get_child_component_by_type_from_actor_func_registry;

FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool>           g_get_children_components_by_name_from_scene_component_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::string&, bool>                              g_get_children_components_by_tag_from_scene_component_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_any_from_scene_component_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_all_from_scene_component_func_registry;
FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, bool>                                                  g_get_children_components_by_type_from_scene_component_func_registry;

FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool> g_get_children_components_by_name_as_map_from_scene_component_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::string&, bool>                    g_get_children_components_by_tag_as_map_from_scene_component_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_any_as_map_from_scene_component_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_all_as_map_from_scene_component_func_registry;
FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, bool>                                        g_get_children_components_by_type_as_map_from_scene_component_func_registry;

FuncRegistry<USceneComponent*, const USceneComponent*, const std::string&, bool>                                           g_get_child_component_by_name_from_scene_component_func_registry;
FuncRegistry<USceneComponent*, const USceneComponent*, const std::string&, bool>                                           g_get_child_component_by_tag_from_scene_component_func_registry;
FuncRegistry<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_any_from_scene_component_func_registry;
FuncRegistry<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_all_from_scene_component_func_registry;
FuncRegistry<USceneComponent*, const USceneComponent*, bool>                                                               g_get_child_component_by_type_from_scene_component_func_registry;

//
// Registrys for spawning actors using a class name instead of template parameters
//

FuncRegistry<AActor*, UWorld*, const FVector&, const FRotator&, const FActorSpawnParameters&> g_spawn_actor_func_registry;

//
// Registrys for creating components using a class name instead of template parameters
//

FuncRegistry<UActorComponent*, AActor*, const std::string&>                    g_create_component_outside_owner_constructor_func_registry;
FuncRegistry<USceneComponent*, AActor*, const std::string&>                    g_create_scene_component_outside_owner_constructor_from_actor_func_registry;
FuncRegistry<USceneComponent*, UObject*, USceneComponent*, const std::string&> g_create_scene_component_outside_owner_constructor_from_object_func_registry;
FuncRegistry<USceneComponent*, USceneComponent*, const std::string&>           g_create_scene_component_outside_owner_constructor_from_component_func_registry;

//
// Registrys for creating objects using a class name instead of template parameters
//

FuncRegistry<UObject*, UObject*, FName, EObjectFlags, UObject*, bool, FObjectInstancingGraph*, UPackage*>           g_new_object_func_registry;
FuncRegistry<UObject*, UObject*, const TCHAR*, const TCHAR*, uint32, UPackageMap*, const FLinkerInstancingContext*> g_load_object_func_registry;
FuncRegistry<UClass*, UObject*, const TCHAR*, const TCHAR*, uint32, UPackageMap*>                                   g_load_class_func_registry;

//
// These maps are necessary to support getStaticStruct<T>() for special struct types that don't have a
// StaticStruct() method., e.g., FRotator and FVector.
//

std::map<std::string, std::string> g_special_struct_names; // map from platform-dependent type name to user-facing name
std::map<std::string, UStruct*> g_special_structs;         // map from platform-dependent type name to UStruct*

//
// Get engine subsystem using a class name instead of template parameters
//

UEngineSubsystem* UnrealClassRegistry::getEngineSubsystemByType(const std::string& class_name) {
    return g_get_engine_subsystem_by_type_func_registry.call(class_name);
}

//
// Get subsystem using a class name instead of template parameters
//

USubsystem* UnrealClassRegistry::getSubsystemByType(const std::string& class_name, UWorld* world) {
    return g_get_subsystem_by_type_func_registry.call(class_name, world);
}

USubsystem* UnrealClassRegistry::getSubsystemByClass(const std::string& class_name, UWorld* world, UClass* uclass) {
    return g_get_subsystem_by_class_func_registry.call(class_name, world, uclass);
}

//
// Get static class using a class name instead of template parameters
//

UClass* UnrealClassRegistry::getStaticClass(const std::string& class_name) {
    return g_get_static_class_func_registry.call(class_name);
}

UStruct* UnrealClassRegistry::getStaticStruct(const std::string& struct_name) {
    return g_get_static_struct_func_registry.call(struct_name);
}

//
// Find actors using a class name instead of template parameters
//

std::vector<AActor*> UnrealClassRegistry::findActorsByName(const std::string& class_name, const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found) {
    return g_find_actors_by_name_func_registry.call(class_name, world, actor_names, return_null_if_not_found);
}

std::vector<AActor*> UnrealClassRegistry::findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return g_find_actors_by_tag_func_registry.call(class_name, world, tag);
}

std::vector<AActor*> UnrealClassRegistry::findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_any_func_registry.call(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistry::findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_all_func_registry.call(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistry::findActorsByType(const std::string& class_name, const UWorld* world) {
    return g_find_actors_by_type_func_registry.call(class_name, world);
}

//

std::map<std::string, AActor*> UnrealClassRegistry::findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found) {
    return g_find_actors_by_name_as_map_func_registry.call(class_name, world, actor_names, return_null_if_not_found);
}

std::map<std::string, AActor*> UnrealClassRegistry::findActorsByTagAsMap(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return g_find_actors_by_tag_as_map_func_registry.call(class_name, world, tag);
}

std::map<std::string, AActor*> UnrealClassRegistry::findActorsByTagAnyAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_any_as_map_func_registry.call(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistry::findActorsByTagAllAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_all_as_map_func_registry.call(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistry::findActorsByTypeAsMap(const std::string& class_name, const UWorld* world) {
    return g_find_actors_by_type_as_map_func_registry.call(class_name, world);
}

//

AActor* UnrealClassRegistry::findActorByName(const std::string& class_name, const UWorld* world, const std::string& actor_name) {
    return g_find_actor_by_name_func_registry.call(class_name, world, actor_name);
}
    
AActor* UnrealClassRegistry::findActorByTag(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return g_find_actor_by_tag_func_registry.call(class_name, world, tag);
}
    
AActor* UnrealClassRegistry::findActorByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actor_by_tag_any_func_registry.call(class_name, world, tags);
}
    
AActor* UnrealClassRegistry::findActorByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actor_by_tag_all_func_registry.call(class_name, world, tags);
}
    
AActor* UnrealClassRegistry::findActorByType(const std::string& class_name, const UWorld* world) {
    return g_find_actor_by_type_func_registry.call(class_name, world);
}

//
// Get components using a class name instead of template parameters
//

std::vector<UActorComponent*> UnrealClassRegistry::getComponentsByName(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors, bool return_null_if_not_found) {
    return g_get_components_by_name_func_registry.call(class_name, actor, component_names, include_from_child_actors, return_null_if_not_found);
}

std::vector<UActorComponent*> UnrealClassRegistry::getComponentsByPath(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors, bool return_null_if_not_found) {
    return g_get_components_by_path_func_registry.call(class_name, actor, component_paths, include_from_child_actors, return_null_if_not_found);
}

std::vector<UActorComponent*> UnrealClassRegistry::getComponentsByTag(
    const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors) {
    return g_get_components_by_tag_func_registry.call(class_name, actor, tag, include_from_child_actors);
}

std::vector<UActorComponent*> UnrealClassRegistry::getComponentsByTagAny(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_any_func_registry.call(class_name, actor, tags, include_from_child_actors);
}

std::vector<UActorComponent*> UnrealClassRegistry::getComponentsByTagAll(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_all_func_registry.call(class_name, actor, tags, include_from_child_actors);
}

std::vector<UActorComponent*> UnrealClassRegistry::getComponentsByType(
    const std::string& class_name, const AActor* actor, bool include_from_child_actors) {
    return g_get_components_by_type_func_registry.call(class_name, actor, include_from_child_actors);
}

//

std::map<std::string, UActorComponent*> UnrealClassRegistry::getComponentsByNameAsMap(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors, bool return_null_if_not_found) {
    return g_get_components_by_name_as_map_func_registry.call(class_name, actor, component_names, include_from_child_actors, return_null_if_not_found);
}

std::map<std::string, UActorComponent*> UnrealClassRegistry::getComponentsByPathAsMap(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors, bool return_null_if_not_found) {
    return g_get_components_by_path_as_map_func_registry.call(class_name, actor, component_paths, include_from_child_actors, return_null_if_not_found);
}

std::map<std::string, UActorComponent*> UnrealClassRegistry::getComponentsByTagAsMap(
    const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors) {
    return g_get_components_by_tag_as_map_func_registry.call(class_name, actor, tag, include_from_child_actors);
}

std::map<std::string, UActorComponent*> UnrealClassRegistry::getComponentsByTagAnyAsMap(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_any_as_map_func_registry.call(class_name, actor, tags, include_from_child_actors);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistry::getComponentsByTagAllAsMap(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_all_as_map_func_registry.call(class_name, actor, tags, include_from_child_actors);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistry::getComponentsByTypeAsMap(
    const std::string& class_name, const AActor* actor, bool include_from_child_actors) {
    return g_get_components_by_type_as_map_func_registry.call(class_name, actor, include_from_child_actors);
}

//

UActorComponent* UnrealClassRegistry::getComponentByName(
    const std::string& class_name, const AActor* actor, const std::string& component_name, bool include_from_child_actors) {
    return g_get_component_by_name_func_registry.call(class_name, actor, component_name, include_from_child_actors);
}

UActorComponent* UnrealClassRegistry::getComponentByPath(
    const std::string& class_name, const AActor* actor, const std::string& component_path, bool include_from_child_actors) {
    return g_get_component_by_path_func_registry.call(class_name, actor, component_path, include_from_child_actors);
}
    
UActorComponent* UnrealClassRegistry::getComponentByTag(
    const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors) {
    return g_get_component_by_tag_func_registry.call(class_name, actor, tag, include_from_child_actors);
}
    
UActorComponent* UnrealClassRegistry::getComponentByTagAny(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_component_by_tag_any_func_registry.call(class_name, actor, tags, include_from_child_actors);
}
    
UActorComponent* UnrealClassRegistry::getComponentByTagAll(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_component_by_tag_all_func_registry.call(class_name, actor, tags, include_from_child_actors);
}
    
UActorComponent* UnrealClassRegistry::getComponentByType(
    const std::string& class_name, const AActor* actor, bool include_from_child_actors) {
    return g_get_component_by_type_func_registry.call(class_name, actor, include_from_child_actors);
}

//
// Get children components using a class name and an AActor* instead of template parameters
//

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByName(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_from_actor_func_registry.call(class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTag(
    const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_from_actor_func_registry.call(class_name, parent, tag, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAny(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_from_actor_func_registry.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAll(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_from_actor_func_registry.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByType(
    const std::string& class_name, const AActor* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_from_actor_func_registry.call(class_name, parent, include_all_descendants);
}

//

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByNameAsMap(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_as_map_from_actor_func_registry.call(class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found);
}

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAsMap(
    const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_as_map_from_actor_func_registry.call(class_name, parent, tag, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAnyAsMap(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_as_map_from_actor_func_registry.call(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAllAsMap(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_as_map_from_actor_func_registry.call(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTypeAsMap(
    const std::string& class_name, const AActor* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_as_map_from_actor_func_registry.call(class_name, parent, include_all_descendants);
}

//

USceneComponent* UnrealClassRegistry::getChildComponentByName(
    const std::string& class_name, const AActor* parent, const std::string& child_component_name, bool include_all_descendants) {
    return g_get_child_component_by_name_from_actor_func_registry.call(class_name, parent, child_component_name, include_all_descendants);
}
    
USceneComponent* UnrealClassRegistry::getChildComponentByTag(
    const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_child_component_by_tag_from_actor_func_registry.call(class_name, parent, tag, include_all_descendants);
}
    
USceneComponent* UnrealClassRegistry::getChildComponentByTagAny(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_child_component_by_tag_any_from_actor_func_registry.call(class_name, parent, tags, include_all_descendants);
}
    
USceneComponent* UnrealClassRegistry::getChildComponentByTagAll(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_child_component_by_tag_all_from_actor_func_registry.call(class_name, parent, tags, include_all_descendants);
}
    
USceneComponent* UnrealClassRegistry::getChildComponentByType(
    const std::string& class_name, const AActor* parent, bool include_all_descendants) {
    return g_get_child_component_by_type_from_actor_func_registry.call(class_name, parent, include_all_descendants);
}

//
// Get children components using a class name and a USceneComponent* instead of template parameters
//

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByName(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_from_scene_component_func_registry.call(class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTag(
    const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_from_scene_component_func_registry.call(class_name, parent, tag, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAny(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_from_scene_component_func_registry.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAll(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_from_scene_component_func_registry.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistry::getChildrenComponentsByType(
    const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_from_scene_component_func_registry.call(class_name, parent, include_all_descendants);
}

//

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByNameAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_as_map_from_scene_component_func_registry.call(class_name, parent, children_component_names, include_all_descendants, return_null_if_not_found);
}

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_as_map_from_scene_component_func_registry.call(class_name, parent, tag, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAnyAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_as_map_from_scene_component_func_registry.call(class_name, parent, tags, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTagAllAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_as_map_from_scene_component_func_registry.call(class_name, parent, tags, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistry::getChildrenComponentsByTypeAsMap(
    const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_as_map_from_scene_component_func_registry.call(class_name, parent, include_all_descendants);
}

//

USceneComponent* UnrealClassRegistry::getChildComponentByName(
    const std::string& class_name, const USceneComponent* parent, const std::string& child_component_name, bool include_all_descendants) {
    return g_get_child_component_by_name_from_scene_component_func_registry.call(class_name, parent, child_component_name, include_all_descendants);
}

USceneComponent* UnrealClassRegistry::getChildComponentByTag(
    const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_child_component_by_tag_from_scene_component_func_registry.call(class_name, parent, tag, include_all_descendants);
}

USceneComponent* UnrealClassRegistry::getChildComponentByTagAny(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_child_component_by_tag_any_from_scene_component_func_registry.call(class_name, parent, tags, include_all_descendants);
}

USceneComponent* UnrealClassRegistry::getChildComponentByTagAll(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_child_component_by_tag_all_from_scene_component_func_registry.call(class_name, parent, tags, include_all_descendants);
}

USceneComponent* UnrealClassRegistry::getChildComponentByType(
    const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return g_get_child_component_by_type_from_scene_component_func_registry.call(class_name, parent, include_all_descendants);
}

//
// Spawn actor using a class name instead of template parameters
//

AActor* UnrealClassRegistry::spawnActor(const std::string& class_name, UWorld* world, const FVector& location, const FRotator& rotation, const FActorSpawnParameters& spawn_parameters) {
    return g_spawn_actor_func_registry.call(class_name, world, location, rotation, spawn_parameters);
}

//
// Create component using a class name instead of template parameters
//

UActorComponent* UnrealClassRegistry::createComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& component_name) {
    return g_create_component_outside_owner_constructor_func_registry.call(class_name, owner, component_name);
}

USceneComponent* UnrealClassRegistry::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& scene_component_name) {
    return g_create_scene_component_outside_owner_constructor_from_actor_func_registry.call(class_name, owner, scene_component_name);
}

USceneComponent* UnrealClassRegistry::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, UObject* owner, USceneComponent* parent, const std::string& scene_component_name) {
    return g_create_scene_component_outside_owner_constructor_from_object_func_registry.call(class_name, owner, parent, scene_component_name);
}

USceneComponent* UnrealClassRegistry::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, USceneComponent* owner, const std::string& scene_component_name) {
    return g_create_scene_component_outside_owner_constructor_from_component_func_registry.call(class_name, owner, scene_component_name);
}

//
// Create new object using a class name instead of template parameters
//

UObject* UnrealClassRegistry::newObject(
    const std::string& class_name,
    UObject* outer,
    FName name,
    EObjectFlags flags,
    UObject* uobject_template,
    bool copy_transients_from_class_defaults,
    FObjectInstancingGraph* in_instance_graph,
    UPackage* external_package)
{
    return g_new_object_func_registry.call(class_name, outer, name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph, external_package);
}

//
// Load objects and classes using a class name instead of template parameters
//

UObject* UnrealClassRegistry::loadObject(
    const std::string& class_name,
    UObject* outer,
    const TCHAR* name,
    const TCHAR* filename,
    uint32 load_flags,
    UPackageMap* sandbox,
    const FLinkerInstancingContext* instancing_context)
{
    return g_load_object_func_registry.call(class_name, outer, name, filename, load_flags, sandbox, instancing_context);
}

UObject* UnrealClassRegistry::loadClass(
    const std::string& class_name,
    UObject* outer,
    const TCHAR* name,
    const TCHAR* filename,
    uint32 load_flags,
    UPackageMap* sandbox)
{
    return g_load_class_func_registry.call(class_name, outer, name, filename, load_flags, sandbox);
}
