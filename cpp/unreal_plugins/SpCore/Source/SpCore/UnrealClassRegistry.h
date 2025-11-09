//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <utility> // std::move
#include <vector>

#include <Engine/World.h>
#include <HAL/Platform.h>           // SPCORE_API, uint32
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <UObject/NameTypes.h>      // FName
#include <UObject/ObjectMacros.h>   // ELoadFlags, EObjectFlags
#include <UObject/UObjectGlobals.h> // LoadClass, LoadObject, NewObject

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/FuncRegistry.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

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
struct FActorSpawnParameters;
struct FObjectInstancingGraph;

//
// Helper macros to avoid repetitive boilerplate code at call sites.
//

#define SP_REGISTER_ENGINE_SUBSYSTEM_CLASS(engine_subsystem_class_)                UnrealClassRegistry::registerEngineSubsystemClass<engine_subsystem_class_>(#engine_subsystem_class_)
#define SP_REGISTER_SUBSYSTEM_PROVIDER_CLASS(subsystem_provider_class_)            UnrealClassRegistry::registerSubsystemProviderClass<subsystem_provider_class_>(#subsystem_provider_class_)
#define SP_REGISTER_SUBSYSTEM_CLASS(subsystem_class_, subsystem_provider_class_)   UnrealClassRegistry::registerSubsystemClass<subsystem_class_, subsystem_provider_class_>(#subsystem_class_)
#define SP_REGISTER_ACTOR_CLASS(actor_class_)                                      UnrealClassRegistry::registerActorClass<actor_class_>(#actor_class_)
#define SP_REGISTER_COMPONENT_CLASS(component_class_)                              UnrealClassRegistry::registerComponentClass<component_class_>(#component_class_)
#define SP_REGISTER_INTERFACE_CLASS(interface_class_)                              UnrealClassRegistry::registerInterface<interface_class_>(#interface_class_)
#define SP_REGISTER_CLASS(class_)                                                  UnrealClassRegistry::registerClass<class_>(#class_)
#define SP_REGISTER_STRUCT(struct_)                                                UnrealClassRegistry::registerStruct<struct_>(#struct_)
#define SP_REGISTER_SPECIAL_STRUCT(special_struct_)                                UnrealClassRegistry::registerSpecialStruct<special_struct_>(#special_struct_)

#define SP_UNREGISTER_ENGINE_SUBSYSTEM_CLASS(engine_subsystem_class_)              UnrealClassRegistry::unregisterEngineSubsystemClass<engine_subsystem_class_>(#engine_subsystem_class_)
#define SP_UNREGISTER_SUBSYSTEM_PROVIDER_CLASS(subsystem_provider_class_)          UnrealClassRegistry::unregisterSubsystemProviderClass<subsystem_provider_class_>(#subsystem_provider_class_)
#define SP_UNREGISTER_SUBSYSTEM_CLASS(subsystem_class_, subsystem_provider_class_) UnrealClassRegistry::unregisterSubsystemClass<subsystem_class_, subsystem_provider_class_>(#subsystem_class_)
#define SP_UNREGISTER_ACTOR_CLASS(actor_class_)                                    UnrealClassRegistry::unregisterActorClass<actor_class_>(#actor_class_)
#define SP_UNREGISTER_COMPONENT_CLASS(component_class_)                            UnrealClassRegistry::unregisterComponentClass<component_class_>(#component_class_)
#define SP_UNREGISTER_INTERFACE_CLASS(interface_class_)                            UnrealClassRegistry::unregisterInterface<interface_class_>(#interface_class_)
#define SP_UNREGISTER_CLASS(class_)                                                UnrealClassRegistry::unregisterClass<class_>(#class_)
#define SP_UNREGISTER_STRUCT(struct_)                                              UnrealClassRegistry::unregisterStruct<struct_>(#struct_)
#define SP_UNREGISTER_SPECIAL_STRUCT(special_struct_)                              UnrealClassRegistry::unregisterSpecialStruct<special_struct_>(#special_struct_)

//
// We need the variables below to be globals because they are referenced in templated code. This requirement
// arises because templated code gets compiled at each call site. If a call site is in a different module
// (i.e., outside of SpCore), and it references a static variable, then the module will get its own local
// copy of the static variable. This behavior only happens on Clang, because MSVC has a different methodology
// for handling static variables in shared libraries. See the link below for details:
//     https://stackoverflow.com/questions/31495877/i-receive-different-results-on-unix-and-win-when-use-static-members-with-static
//

//
// Registries for getting subsystems using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<UEngineSubsystem*>             g_get_engine_subsystem_by_type_func_registry;
extern SPCORE_API FuncRegistry<USubsystem*, UWorld*>          g_get_subsystem_by_type_func_registry;
extern SPCORE_API FuncRegistry<USubsystem*, UWorld*, UClass*> g_get_subsystem_by_class_func_registry;

//
// Registries for getting a static class or static struct using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<UClass*> g_get_static_class_func_registry;
extern SPCORE_API FuncRegistry<UStruct*> g_get_static_struct_func_registry;

//
// Registries for finding actors using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&, bool>           g_find_actors_by_name_func_registry;
extern SPCORE_API FuncRegistry<std::vector<AActor*>, const UWorld*, const std::string&>                              g_find_actors_by_tag_func_registry;
extern SPCORE_API FuncRegistry<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 g_find_actors_by_tag_any_func_registry;
extern SPCORE_API FuncRegistry<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 g_find_actors_by_tag_all_func_registry;
extern SPCORE_API FuncRegistry<std::vector<AActor*>, const UWorld*>                                                  g_find_actors_by_type_func_registry;

extern SPCORE_API FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&, bool> g_find_actors_by_name_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::string&>                    g_find_actors_by_tag_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       g_find_actors_by_tag_any_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       g_find_actors_by_tag_all_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, AActor*>, const UWorld*>                                        g_find_actors_by_type_as_map_func_registry;

extern SPCORE_API FuncRegistry<AActor*, const UWorld*, const std::string&>                                           g_find_actor_by_name_func_registry;
extern SPCORE_API FuncRegistry<AActor*, const UWorld*, const std::string&>                                           g_find_actor_by_tag_func_registry;
extern SPCORE_API FuncRegistry<AActor*, const UWorld*, const std::vector<std::string>&>                              g_find_actor_by_tag_any_func_registry;
extern SPCORE_API FuncRegistry<AActor*, const UWorld*, const std::vector<std::string>&>                              g_find_actor_by_tag_all_func_registry;
extern SPCORE_API FuncRegistry<AActor*, const UWorld*>                                                               g_find_actor_by_type_func_registry;

//
// Registries for getting components using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_components_by_name_func_registry;
extern SPCORE_API FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_components_by_path_func_registry;
extern SPCORE_API FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::string&, bool>                              g_get_components_by_tag_func_registry;
extern SPCORE_API FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_components_by_tag_any_func_registry;
extern SPCORE_API FuncRegistry<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_components_by_tag_all_func_registry;
extern SPCORE_API FuncRegistry<std::vector<UActorComponent*>, const AActor*, bool>                                                  g_get_components_by_type_func_registry;

extern SPCORE_API FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_components_by_name_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_components_by_path_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::string&, bool>                    g_get_components_by_tag_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_components_by_tag_any_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_components_by_tag_all_as_map_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, UActorComponent*>, const AActor*, bool>                                        g_get_components_by_type_as_map_func_registry;

extern SPCORE_API FuncRegistry<UActorComponent*, const AActor*, const std::string&, bool>                                           g_get_component_by_name_func_registry;
extern SPCORE_API FuncRegistry<UActorComponent*, const AActor*, const std::string&, bool>                                           g_get_component_by_path_func_registry;
extern SPCORE_API FuncRegistry<UActorComponent*, const AActor*, const std::string&, bool>                                           g_get_component_by_tag_func_registry;
extern SPCORE_API FuncRegistry<UActorComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_component_by_tag_any_func_registry;
extern SPCORE_API FuncRegistry<UActorComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_component_by_tag_all_func_registry;
extern SPCORE_API FuncRegistry<UActorComponent*, const AActor*, bool>                                                               g_get_component_by_type_func_registry;

//
// Registries for getting children components using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_children_components_by_name_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::string&, bool>                              g_get_children_components_by_tag_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_any_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_all_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const AActor*, bool>                                                  g_get_children_components_by_type_from_actor_func_registry;

extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_children_components_by_name_as_map_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::string&, bool>                    g_get_children_components_by_tag_as_map_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_any_as_map_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_all_as_map_from_actor_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const AActor*, bool>                                        g_get_children_components_by_type_as_map_from_actor_func_registry;

extern SPCORE_API FuncRegistry<USceneComponent*, const AActor*, const std::string&, bool>                                           g_get_child_component_by_name_from_actor_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const AActor*, const std::string&, bool>                                           g_get_child_component_by_tag_from_actor_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_any_from_actor_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const AActor*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_all_from_actor_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const AActor*, bool>                                                               g_get_child_component_by_type_from_actor_func_registry;

extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool>           g_get_children_components_by_name_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::string&, bool>                              g_get_children_components_by_tag_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_any_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_all_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::vector<USceneComponent*>, const USceneComponent*, bool>                                                  g_get_children_components_by_type_from_scene_component_func_registry;

extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool> g_get_children_components_by_name_as_map_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::string&, bool>                    g_get_children_components_by_tag_as_map_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_any_as_map_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_all_as_map_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<std::map<std::string, USceneComponent*>, const USceneComponent*, bool>                                        g_get_children_components_by_type_as_map_from_scene_component_func_registry;

extern SPCORE_API FuncRegistry<USceneComponent*, const USceneComponent*, const std::string&, bool>                                           g_get_child_component_by_name_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const USceneComponent*, const std::string&, bool>                                           g_get_child_component_by_tag_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_any_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool>                              g_get_child_component_by_tag_all_from_scene_component_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, const USceneComponent*, bool>                                                               g_get_child_component_by_type_from_scene_component_func_registry;

//
// Registries for spawning actors using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<AActor*, UWorld*, const FVector&, const FRotator&, const FActorSpawnParameters&> g_spawn_actor_func_registry;

//
// Registries for creating components using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<UActorComponent*, AActor*, const std::string&>                    g_create_component_outside_owner_constructor_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, AActor*, const std::string&>                    g_create_scene_component_outside_owner_constructor_from_actor_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, UObject*, USceneComponent*, const std::string&> g_create_scene_component_outside_owner_constructor_from_object_func_registry;
extern SPCORE_API FuncRegistry<USceneComponent*, USceneComponent*, const std::string&>           g_create_scene_component_outside_owner_constructor_from_component_func_registry;

//
// Registries for creating objects and classes using a class name instead of template parameters
//

extern SPCORE_API FuncRegistry<UObject*, UObject*, FName, EObjectFlags, UObject*, bool, FObjectInstancingGraph*, UPackage*>           g_new_object_func_registry;
extern SPCORE_API FuncRegistry<UObject*, UObject*, const TCHAR*, const TCHAR*, uint32, UPackageMap*, const FLinkerInstancingContext*> g_load_object_func_registry;
extern SPCORE_API FuncRegistry<UClass*, UObject*, const TCHAR*, const TCHAR*, uint32, UPackageMap*>                                   g_load_class_func_registry;

//
// These maps are necessary to support getStaticStruct<T>() for special struct types that don't have a
// StaticStruct() method., e.g., FRotator and FVector.
//

extern SPCORE_API std::map<std::string, std::string> g_special_struct_names; // map from platform-dependent type name to user-facing name
extern SPCORE_API std::map<std::string, UStruct*> g_special_structs;         // map from platform-dependent type name to UStruct*

class SPCORE_API UnrealClassRegistry
{
public:
    UnrealClassRegistry() = delete;
    ~UnrealClassRegistry() = delete;

    //
    // Get engine subsystem using a class name instead of template parameters
    //

    static UEngineSubsystem* getEngineSubsystemByType(const std::string& class_name);

    //
    // Get subsystem using a class name instead of template parameters
    //

    static USubsystem* getSubsystemByType(const std::string& class_name, UWorld* world);
    static USubsystem* getSubsystemByClass(const std::string& class_name, UWorld* world, UClass* uclass);

    //
    // Get static class or static struct using a class name instead of template parameters
    //

    static UClass* getStaticClass(const std::string& class_name);
    static UStruct* getStaticStruct(const std::string& struct_name);

    //
    // Find actors using a class name instead of template parameters
    //

    static std::vector<AActor*> findActorsByName(const std::string& class_name, const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found = true);
    static std::vector<AActor*> findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag);
    static std::vector<AActor*> findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::vector<AActor*> findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::vector<AActor*> findActorsByType(const std::string& class_name, const UWorld* world);

    static std::map<std::string, AActor*> findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found = true);
    static std::map<std::string, AActor*> findActorsByTagAsMap(const std::string& class_name, const UWorld* world, const std::string& tag);
    static std::map<std::string, AActor*> findActorsByTagAnyAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::map<std::string, AActor*> findActorsByTagAllAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static std::map<std::string, AActor*> findActorsByTypeAsMap(const std::string& class_name, const UWorld* world);

    static AActor* findActorByName(const std::string& class_name, const UWorld* world, const std::string& actor_name);
    static AActor* findActorByTag(const std::string& class_name, const UWorld* world, const std::string& tag);
    static AActor* findActorByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static AActor* findActorByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags);
    static AActor* findActorByType(const std::string& class_name, const UWorld* world);

    //
    // Get components using a class name instead of template parameters
    //

    static std::vector<UActorComponent*> getComponentsByName(const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors = false, bool return_null_if_not_found = true);
    static std::vector<UActorComponent*> getComponentsByPath(const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors = false, bool return_null_if_not_found = true);
    static std::vector<UActorComponent*> getComponentsByTag(const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors = false);
    static std::vector<UActorComponent*> getComponentsByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false);
    static std::vector<UActorComponent*> getComponentsByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false);
    static std::vector<UActorComponent*> getComponentsByType(const std::string& class_name, const AActor* actor, bool include_from_child_actors = false);

    static std::map<std::string, UActorComponent*> getComponentsByNameAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors = false, bool return_null_if_not_found = true);
    static std::map<std::string, UActorComponent*> getComponentsByPathAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors = false, bool return_null_if_not_found = true);
    static std::map<std::string, UActorComponent*> getComponentsByTagAsMap(const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors = false);
    static std::map<std::string, UActorComponent*> getComponentsByTagAnyAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false);
    static std::map<std::string, UActorComponent*> getComponentsByTagAllAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false);
    static std::map<std::string, UActorComponent*> getComponentsByTypeAsMap(const std::string& class_name, const AActor* actor, bool include_from_child_actors = false);

    static UActorComponent* getComponentByName(const std::string& class_name, const AActor* actor, const std::string& component_name, bool include_from_child_actors = false);
    static UActorComponent* getComponentByPath(const std::string& class_name, const AActor* actor, const std::string& component_path, bool include_from_child_actors = false);
    static UActorComponent* getComponentByTag(const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors = false);
    static UActorComponent* getComponentByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false);
    static UActorComponent* getComponentByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors = false);
    static UActorComponent* getComponentByType(const std::string& class_name, const AActor* actor, bool include_from_child_actors = false);

    //
    // Get children components using a class name and an AActor* instead of template parameters
    //

    static std::vector<USceneComponent*> getChildrenComponentsByName(const std::string& class_name, const AActor* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTag(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAny(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAll(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByType(const std::string& class_name, const AActor* parent, bool include_all_descendants = true);

    static std::map<std::string, USceneComponent*> getChildrenComponentsByNameAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAsMap(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAnyAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAllAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTypeAsMap(const std::string& class_name, const AActor* parent, bool include_all_descendants = true);

    static USceneComponent* getChildComponentByName(const std::string& class_name, const AActor* parent, const std::string& child_component_name, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByTag(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByTagAny(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByTagAll(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByType(const std::string& class_name, const AActor* parent, bool include_all_descendants = true);

    //
    // Get children components using a class name and an USceneComponent* instead of template parameters
    //

    static std::vector<USceneComponent*> getChildrenComponentsByName(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::vector<USceneComponent*> getChildrenComponentsByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants = true);

    static std::map<std::string, USceneComponent*> getChildrenComponentsByNameAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants = true, bool return_null_if_not_found = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAsMap(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAnyAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTagAllAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static std::map<std::string, USceneComponent*> getChildrenComponentsByTypeAsMap(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants = true);

    static USceneComponent* getChildComponentByName(const std::string& class_name, const USceneComponent* parent, const std::string& child_component_name, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants = true);
    static USceneComponent* getChildComponentByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants = true);

    //
    // Spawn actor using a class name instead of template parameters
    //

    static AActor* spawnActor(const std::string& class_name, UWorld* world, const FVector& location, const FRotator& rotation, const FActorSpawnParameters& spawn_parameters);

    //
    // Create component using a class name instead of template parameters
    //

    static UActorComponent* createComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& component_name);
    static USceneComponent* createSceneComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& scene_component_name);
    static USceneComponent* createSceneComponentOutsideOwnerConstructor(const std::string& class_name, UObject* owner, USceneComponent* parent, const std::string& scene_component_name);
    static USceneComponent* createSceneComponentOutsideOwnerConstructor(const std::string& class_name, USceneComponent* owner, const std::string& scene_component_name);

    //
    // Create new object using a class name instead of template parameters
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

    //
    // Load object and class using a class name instead of template parameters
    //

    static UObject* loadObject(
        const std::string& class_name,
        UObject* outer,
        const TCHAR* name,
        const TCHAR* filename = nullptr,
        uint32 load_flags = ELoadFlags::LOAD_None,
        UPackageMap* sandbox = nullptr,
        const FLinkerInstancingContext* instancing_context = nullptr);

    static UObject* loadClass(
        const std::string& class_name,
        UObject* outer,
        const TCHAR* name,
        const TCHAR* filename = nullptr,
        uint32 load_flags = ELoadFlags::LOAD_None,
        UPackageMap* sandbox = nullptr);

    //
    // Register engine subsystem class
    //

    template <CEngineSubsystem TEngineSubsystem>
    static void registerEngineSubsystemClass(const std::string& class_name)
    {
        registerClassCommon<TEngineSubsystem>(class_name);

        g_get_engine_subsystem_by_type_func_registry.registerFunc(
            class_name, []() -> UEngineSubsystem* {
                return Unreal::getEngineSubsystemByType<TEngineSubsystem>();
            });
    }

    //
    // Register subsystem provider class
    //

    template <CSubsystemProvider TSubsystemProvider>
    static void registerSubsystemProviderClass(const std::string& class_name)
    {
        g_get_subsystem_by_class_func_registry.registerFunc(
            class_name, [](UWorld* world, UClass* uclass) -> USubsystem* {
                return Unreal::getSubsystemByClass<TSubsystemProvider>(world, uclass);
            });
    }

    //
    // Register subsystem class
    //

    template <CSubsystem TSubsystem, CSubsystemProvider TSubsystemProvider>
    static void registerSubsystemClass(const std::string& class_name)
    {
        registerClassCommon<TSubsystem>(class_name);

        g_get_subsystem_by_type_func_registry.registerFunc(
            class_name, [](UWorld* world) -> USubsystem* {
                return Unreal::getSubsystemByType<TSubsystem, TSubsystemProvider>(world);
            });
    }

    //
    // Register actor class
    //

    template <CActor TActor>
    static void registerActorClass(const std::string& class_name)
    {
        registerClassCommon<TActor>(class_name);

        //
        // Spawn actor
        //

        g_spawn_actor_func_registry.registerFunc(
            class_name, [](UWorld* world, const FVector& location, const FRotator& rotation, const FActorSpawnParameters& spawn_parameters) -> AActor* {
                SP_ASSERT(world); return world->SpawnActor<TActor>(location, rotation, spawn_parameters); });

        //
        // Find actors by name or tag or type and return an std::vector
        //

        g_find_actors_by_name_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found) -> std::vector<AActor*> {
                return Unreal::findActorsByName<TActor, AActor>(world, actor_names, return_null_if_not_found); });

        g_find_actors_by_tag_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::string& tag) -> std::vector<AActor*> {
                return Unreal::findActorsByTag<TActor, AActor>(world, tag); });

        g_find_actors_by_tag_any_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::vector<AActor*> {
                return Unreal::findActorsByTagAny<TActor, AActor>(world, tags); });

        g_find_actors_by_tag_all_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::vector<AActor*> {
                return Unreal::findActorsByTagAll<TActor, AActor>(world, tags); });

        g_find_actors_by_type_func_registry.registerFunc(
            class_name, [](const UWorld* world) -> std::vector<AActor*> {
                return Unreal::findActorsByType<TActor, AActor>(world); });

        //
        // Find actors by name or tag or type and return an std::map
        //

        g_find_actors_by_name_as_map_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& actor_names, bool return_null_if_not_found) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByNameAsMap<TActor, AActor>(world, actor_names, return_null_if_not_found); });

        g_find_actors_by_tag_as_map_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::string& tag) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAsMap<TActor, AActor>(world, tag); });
        
        g_find_actors_by_tag_any_as_map_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAnyAsMap<TActor, AActor>(world, tags); });
        
        g_find_actors_by_tag_all_as_map_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTagAllAsMap<TActor, AActor>(world, tags); });
        
        g_find_actors_by_type_as_map_func_registry.registerFunc(
            class_name, [](const UWorld* world) -> std::map<std::string, AActor*> {
                return Unreal::findActorsByTypeAsMap<TActor, AActor>(world); });
        
        //
        // Find actor by name or tag or type and return a pointer
        //

        g_find_actor_by_name_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::string& actor_name) -> AActor* {
                return Unreal::findActorByName<TActor, AActor>(world, actor_name); });
        
        g_find_actor_by_tag_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::string& tag) -> AActor* {
                return Unreal::findActorByTag<TActor, AActor>(world, tag); });
        
        g_find_actor_by_tag_any_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> AActor* {
                return Unreal::findActorByTagAny<TActor, AActor>(world, tags); });
        
        g_find_actor_by_tag_all_func_registry.registerFunc(
            class_name, [](const UWorld* world, const std::vector<std::string>& tags) -> AActor* {
                return Unreal::findActorByTagAll<TActor, AActor>(world, tags); });
        
        g_find_actor_by_type_func_registry.registerFunc(
            class_name, [](const UWorld* world) -> AActor* {
                return Unreal::findActorByType<TActor, AActor>(world); });
    }

    //
    // Register component class
    //

    template <CNonSceneComponent TNonSceneComponent>
    static void registerComponentClass(const std::string& class_name)
    {
        registerComponentClassCommon<TNonSceneComponent>(class_name);

        //
        // Create component
        //

        g_create_component_outside_owner_constructor_func_registry.registerFunc(
            class_name, [](AActor* owner, const std::string& component_name) -> UActorComponent* {
                return Unreal::createComponentOutsideOwnerConstructor<TNonSceneComponent, UActorComponent>(owner, component_name); });
    }

    template <CSceneComponent TSceneComponent>
    static void registerComponentClass(const std::string& class_name)
    {
        registerComponentClassCommon<TSceneComponent>(class_name);

        //
        // Create component
        //

        g_create_scene_component_outside_owner_constructor_from_actor_func_registry.registerFunc(
            class_name, [](AActor* owner, const std::string& scene_component_name) -> USceneComponent* {
                return Unreal::createSceneComponentOutsideOwnerConstructor<TSceneComponent, USceneComponent>(owner, scene_component_name); });

        g_create_scene_component_outside_owner_constructor_from_object_func_registry.registerFunc(
            class_name, [](UObject* owner, USceneComponent* parent, const std::string& scene_component_name) -> USceneComponent* {
                return Unreal::createSceneComponentOutsideOwnerConstructor<TSceneComponent, USceneComponent>(owner, parent, scene_component_name); });

        g_create_scene_component_outside_owner_constructor_from_component_func_registry.registerFunc(
            class_name, [](USceneComponent* owner, const std::string& scene_component_name) -> USceneComponent* {
                return Unreal::createSceneComponentOutsideOwnerConstructor<TSceneComponent, USceneComponent>(owner, scene_component_name); });

        //
        // Get children components by name or tag or type from an AActor* and return an std::vector
        //

        g_get_children_components_by_name_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByName<AActor, TSceneComponent, USceneComponent>(parent, children_component_names, include_all_descendants, return_null_if_not_found); });

        g_get_children_components_by_tag_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::string& tag, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTag<AActor, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants); });

        g_get_children_components_by_tag_any_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAny<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        g_get_children_components_by_tag_all_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAll<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        g_get_children_components_by_type_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByType<TSceneComponent, USceneComponent>(parent, include_all_descendants); });

        //
        // Get children components by name or tag or type from an AActor* and return an std::map
        //

        g_get_children_components_by_name_as_map_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByNameAsMap<AActor, TSceneComponent, USceneComponent>(parent, children_component_names, include_all_descendants, return_null_if_not_found);
            });

        g_get_children_components_by_tag_as_map_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::string& tag, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAsMap<AActor, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants);
            });

        g_get_children_components_by_tag_any_as_map_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAnyAsMap<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_children_components_by_tag_all_as_map_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAllAsMap<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_children_components_by_type_as_map_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTypeAsMap<AActor, TSceneComponent, USceneComponent>(parent, include_all_descendants);
            });

        //
        // Get child component by name or tag or type from an AActor* and return a pointer
        //

        g_get_child_component_by_name_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::string& child_component_name, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByName<AActor, TSceneComponent, USceneComponent>(parent, child_component_name, include_all_descendants);
            });

        g_get_child_component_by_tag_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::string& tag, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByTag<AActor, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants);
            });

        g_get_child_component_by_tag_any_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByTagAny<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_child_component_by_tag_all_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByTagAll<AActor, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_child_component_by_type_from_actor_func_registry.registerFunc(
            class_name, [](const AActor* parent, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByType<AActor, TSceneComponent, USceneComponent>(parent, include_all_descendants);
            });

        //
        // Get children components by name or tag or type from a USceneComponent* and return an std::vector
        //

        g_get_children_components_by_name_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByName<USceneComponent, TSceneComponent, USceneComponent>(parent, children_component_names, include_all_descendants, return_null_if_not_found); });

        g_get_children_components_by_tag_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& tag, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTag<USceneComponent, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants); });

        g_get_children_components_by_tag_any_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAny<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        g_get_children_components_by_tag_all_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAll<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants); });

        g_get_children_components_by_type_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, bool include_all_descendants) -> std::vector<USceneComponent*> {
                return Unreal::getChildrenComponentsByType<TSceneComponent, USceneComponent>(parent, include_all_descendants); });

        //
        // Get children components by name or tag or type from a USceneComponent* and return an std::map
        //

        g_get_children_components_by_name_as_map_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& children_component_names, bool include_all_descendants, bool return_null_if_not_found) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByNameAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, children_component_names, include_all_descendants, return_null_if_not_found);
            });

        g_get_children_components_by_tag_as_map_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& tag, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants);
            });

        g_get_children_components_by_tag_any_as_map_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAnyAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_children_components_by_tag_all_as_map_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTagAllAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_children_components_by_type_as_map_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, bool include_all_descendants) -> std::map<std::string, USceneComponent*> {
                return Unreal::getChildrenComponentsByTypeAsMap<USceneComponent, TSceneComponent, USceneComponent>(parent, include_all_descendants);
            });

        //
        // Get child component by name or tag or type from a USceneComponent* and return a pointer
        //

        g_get_child_component_by_name_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& child_component_name, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByName<USceneComponent, TSceneComponent, USceneComponent>(parent, child_component_name, include_all_descendants);
            });

        g_get_child_component_by_tag_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::string& tag, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByTag<USceneComponent, TSceneComponent, USceneComponent>(parent, tag, include_all_descendants);
            });

        g_get_child_component_by_tag_any_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByTagAny<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_child_component_by_tag_all_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByTagAll<USceneComponent, TSceneComponent, USceneComponent>(parent, tags, include_all_descendants);
            });

        g_get_child_component_by_type_from_scene_component_func_registry.registerFunc(
            class_name, [](const USceneComponent* parent, bool include_all_descendants) -> USceneComponent* {
                return Unreal::getChildComponentByType<USceneComponent, TSceneComponent, USceneComponent>(parent, include_all_descendants);
            });
    }

    template <CComponent TComponent>
    static void registerComponentClassCommon(const std::string& class_name)
    {
        registerClassCommon<TComponent>(class_name);

        //
        // Get components by name or tag or type and return an std::vector
        //

        g_get_components_by_name_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors, bool return_null_if_not_found) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByName<TComponent, UActorComponent>(actor, component_names, include_from_child_actors, return_null_if_not_found); });

        g_get_components_by_path_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors, bool return_null_if_not_found) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByPath<TComponent, UActorComponent>(actor, component_paths, include_from_child_actors, return_null_if_not_found); });

        g_get_components_by_tag_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::string& tag, bool include_from_child_actors) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTag<TComponent, UActorComponent>(actor, tag, include_from_child_actors); });

        g_get_components_by_tag_any_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTagAny<TComponent, UActorComponent>(actor, tags, include_from_child_actors); });

        g_get_components_by_tag_all_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByTagAll<TComponent, UActorComponent>(actor, tags, include_from_child_actors); });

        g_get_components_by_type_func_registry.registerFunc(
            class_name, [](const AActor* actor, bool include_from_child_actors) -> std::vector<UActorComponent*> {
                return Unreal::getComponentsByType<TComponent, UActorComponent>(actor, include_from_child_actors); });

        //
        // Get components by name or tag or type and return an std::map
        //

        g_get_components_by_name_as_map_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& component_names, bool include_from_child_actors, bool return_null_if_not_found) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByNameAsMap<TComponent, UActorComponent>(actor, component_names, include_from_child_actors, return_null_if_not_found); });

        g_get_components_by_path_as_map_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& component_paths, bool include_from_child_actors, bool return_null_if_not_found) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByPathAsMap<TComponent, UActorComponent>(actor, component_paths, include_from_child_actors, return_null_if_not_found); });

        g_get_components_by_tag_as_map_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::string& tag, bool include_from_child_actors) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAsMap<TComponent, UActorComponent>(actor, tag, include_from_child_actors); });
        
        g_get_components_by_tag_any_as_map_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAnyAsMap<TComponent, UActorComponent>(actor, tags, include_from_child_actors); });
        
        g_get_components_by_tag_all_as_map_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTagAllAsMap<TComponent, UActorComponent>(actor, tags, include_from_child_actors); });
        
        g_get_components_by_type_as_map_func_registry.registerFunc(
            class_name, [](const AActor* actor, bool include_from_child_actors) -> std::map<std::string, UActorComponent*> {
                return Unreal::getComponentsByTypeAsMap<TComponent, UActorComponent>(actor, include_from_child_actors); });

        //
        // Get component by name or tag or type and return a pointer
        //

        g_get_component_by_name_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::string& component_name, bool include_from_child_actors) -> UActorComponent* {
                return Unreal::getComponentByName<TComponent, UActorComponent>(actor, component_name, include_from_child_actors); });

        g_get_component_by_path_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::string& component_path, bool include_from_child_actors) -> UActorComponent* {
                return Unreal::getComponentByPath<TComponent, UActorComponent>(actor, component_path, include_from_child_actors); });
        
        g_get_component_by_tag_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::string& tag, bool include_from_child_actors) -> UActorComponent* {
                return Unreal::getComponentByTag<TComponent, UActorComponent>(actor, tag, include_from_child_actors); });
        
        g_get_component_by_tag_any_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) -> UActorComponent* {
                return Unreal::getComponentByTagAny<TComponent, UActorComponent>(actor, tags, include_from_child_actors); });
        
        g_get_component_by_tag_all_func_registry.registerFunc(
            class_name, [](const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) -> UActorComponent* {
                return Unreal::getComponentByTagAll<TComponent, UActorComponent>(actor, tags, include_from_child_actors); });
        
        g_get_component_by_type_func_registry.registerFunc(
            class_name, [](const AActor* actor, bool include_from_child_actors) -> UActorComponent* {
                return Unreal::getComponentByType<TComponent, UActorComponent>(actor, include_from_child_actors); });
    }

    template <CInterface TInterface>
    static void registerInterface(const std::string& class_name)
    {
        registerClassCommon<TInterface>(class_name);
    }

    template <CClass TClass>
    static void registerClass(const std::string& class_name)
    {
        registerClassCommon<TClass>(class_name);

        //
        // Create object
        //

        g_new_object_func_registry.registerFunc(
            class_name, [](UObject* outer, FName name, EObjectFlags flags, UObject* uobject_template, bool copy_transients_from_class_defaults, FObjectInstancingGraph* in_instance_graph, UPackage* external_package) -> UObject* {
                return NewObject<UObject>(outer, TClass::StaticClass(), name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph, external_package);
            });

        g_load_object_func_registry.registerFunc(
            class_name, [](UObject* outer, const TCHAR* name, const TCHAR* filename, uint32 load_flags, UPackageMap* sandbox, const FLinkerInstancingContext* instancing_context) -> UObject* {
                return LoadObject<TClass>(outer, name, filename, load_flags, sandbox, instancing_context);
            });

        g_load_class_func_registry.registerFunc(
            class_name, [](UObject* outer, const TCHAR* name, const TCHAR* filename, uint32 load_flags, UPackageMap* sandbox) -> UClass* {
                return LoadClass<TClass>(outer, name, filename, load_flags, sandbox);
            });
    }

    template <typename TClass>
    static void registerClassCommon(const std::string& class_name)
    {
        registerStructCommon<TClass>(class_name);

        //
        // Get static class
        //

        g_get_static_class_func_registry.registerFunc(
            class_name, []() -> UClass* {
                return Unreal::getStaticClass<TClass>();
            });
    }

    template <CStruct TStruct>
    static void registerStruct(const std::string& struct_name)
    {
        registerStructCommon<TStruct>(struct_name);
    }

    //
    // The functions below are required to support the getStaticStruct<T>() method in cases where an otherwise
    // well-formed struct type doesn't define a StaticStruct() method, as is the case with FRotator and
    // FVector. See the following file for more examples of similar special struct types:
    //     Engine/Source/CoreUObject/Public/UObject/NoExportTypes.h
    //
    // For the getStaticStruct<T>() method to behave as expected for a type T, the type must meet the
    // following conditions. First, the type needs to be registered by calling registerSpecialStruct<T>(...)
    // before calling getStaticStruct<T>(). If the type is registered by UStruct*, then there are no other
    // requirements. If the type is registered by name, then getStaticStruct<T>() will call Unreal::findSpecialStructByName(...)
    // internally to find the appropriate UStruct*. For a type to be visible to Unreal::findSpecialStructByName(...),
    // the USpecialStructs class must define a UPROPERTY of type T that is named according to a particular
    // naming convention. When a type no longer needs to be visible to getStaticStruct<T>(), it should be
    // unregistered by calling UnrealClassRegistry::unregisterSpecialStruct<T>().
    // 
    // Registering special structs by name is more convenient because it doesn't require the caller to obtain
    // a valid UStruct*, but it requires ASpecialStructActor to have defined the appropriate UPROPERTY. On
    // the other hand, registering special structs by UStruct* is is more flexible because it enables the
    // caller to register special structs that are not present in ASpecialStructActor.
    //

    template <typename TSpecialStruct> requires
        (!CStruct<TSpecialStruct>)
    static void registerSpecialStruct(const std::string& struct_name)
    {
        registerStructCommon<TSpecialStruct>(struct_name);

        std::string type_id_string = Std::getTypeIdString<TSpecialStruct>();
        SP_ASSERT(!Std::containsKey(g_special_struct_names, type_id_string));
        SP_ASSERT(!Std::containsKey(g_special_structs, type_id_string));
        Std::insert(g_special_struct_names, std::move(type_id_string), struct_name);
    }

    template <typename TSpecialStruct> requires
        (!CStruct<TSpecialStruct>)
    static void registerSpecialStruct(const std::string& struct_name, UStruct* ustruct)
    {
        registerStructCommon<TSpecialStruct>(struct_name);

        SP_ASSERT(ustruct);
        std::string type_id_string = Std::getTypeIdString<TSpecialStruct>();
        SP_ASSERT(!Std::containsKey(g_special_struct_names, type_id_string));
        SP_ASSERT(!Std::containsKey(g_special_structs, type_id_string));
        Std::insert(g_special_structs, std::move(type_id_string), ustruct);
    }

    template <typename TStruct>
    static void registerStructCommon(const std::string& struct_name)
    {
        g_get_static_struct_func_registry.registerFunc(
            struct_name, []() -> UStruct* {
                return getStaticStruct<TStruct>();
            });
    }

    //
    // Unregister classes
    //

    template <CEngineSubsystem TEngineSubsystem>
    static void unregisterEngineSubsystemClass(const std::string& class_name)
    {
        unregisterClassCommon<TEngineSubsystem>(class_name);

        g_get_engine_subsystem_by_type_func_registry.unregisterFunc(class_name);
    }

    template <CSubsystemProvider TSubsystemProvider>
    static void unregisterSubsystemProviderClass(const std::string& class_name)
    {
        g_get_subsystem_by_class_func_registry.unregisterFunc(class_name);
    }

    template <CSubsystem TSubsystem, CSubsystemProvider TSubsystemProvider>
    static void unregisterSubsystemClass(const std::string& class_name)
    {
        unregisterClassCommon<TSubsystem>(class_name);

        g_get_subsystem_by_type_func_registry.unregisterFunc(class_name);
    }

    template <CActor TActor>
    static void unregisterActorClass(const std::string& class_name)
    {
        unregisterClassCommon<TActor>(class_name);

        g_spawn_actor_func_registry.unregisterFunc(class_name);

        g_find_actors_by_name_func_registry.unregisterFunc(class_name);
        g_find_actors_by_tag_func_registry.unregisterFunc(class_name);
        g_find_actors_by_tag_any_func_registry.unregisterFunc(class_name);
        g_find_actors_by_tag_all_func_registry.unregisterFunc(class_name);
        g_find_actors_by_type_func_registry.unregisterFunc(class_name);

        g_find_actors_by_name_as_map_func_registry.unregisterFunc(class_name);
        g_find_actors_by_tag_as_map_func_registry.unregisterFunc(class_name);
        g_find_actors_by_tag_any_as_map_func_registry.unregisterFunc(class_name);
        g_find_actors_by_tag_all_as_map_func_registry.unregisterFunc(class_name);
        g_find_actors_by_type_as_map_func_registry.unregisterFunc(class_name);

        g_find_actor_by_name_func_registry.unregisterFunc(class_name);
        g_find_actor_by_tag_func_registry.unregisterFunc(class_name);
        g_find_actor_by_tag_any_func_registry.unregisterFunc(class_name);
        g_find_actor_by_tag_all_func_registry.unregisterFunc(class_name);
        g_find_actor_by_type_func_registry.unregisterFunc(class_name);
    }

    template <CNonSceneComponent TNonSceneComponent>
    static void unregisterComponentClass(const std::string& class_name)
    {
        unregisterComponentClassCommon<TNonSceneComponent>(class_name);

        g_create_component_outside_owner_constructor_func_registry.unregisterFunc(class_name);
    }

    template <CSceneComponent TSceneComponent>
    static void unregisterComponentClass(const std::string& class_name)
    {
        unregisterComponentClassCommon<TSceneComponent>(class_name);

        g_create_scene_component_outside_owner_constructor_from_actor_func_registry.unregisterFunc(class_name);
        g_create_scene_component_outside_owner_constructor_from_object_func_registry.unregisterFunc(class_name);
        g_create_scene_component_outside_owner_constructor_from_component_func_registry.unregisterFunc(class_name);

        g_get_children_components_by_name_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_any_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_all_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_type_from_actor_func_registry.unregisterFunc(class_name);

        g_get_children_components_by_name_as_map_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_as_map_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_any_as_map_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_all_as_map_from_actor_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_type_as_map_from_actor_func_registry.unregisterFunc(class_name);

        g_get_child_component_by_name_from_actor_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_tag_from_actor_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_tag_any_from_actor_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_tag_all_from_actor_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_type_from_actor_func_registry.unregisterFunc(class_name);

        g_get_children_components_by_name_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_any_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_all_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_type_from_scene_component_func_registry.unregisterFunc(class_name);

        g_get_children_components_by_name_as_map_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_as_map_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_any_as_map_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_tag_all_as_map_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_children_components_by_type_as_map_from_scene_component_func_registry.unregisterFunc(class_name);

        g_get_child_component_by_name_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_tag_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_tag_any_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_tag_all_from_scene_component_func_registry.unregisterFunc(class_name);
        g_get_child_component_by_type_from_scene_component_func_registry.unregisterFunc(class_name);
    }

    template <CComponent TComponent>
    static void unregisterComponentClassCommon(const std::string& class_name)
    {
        unregisterClassCommon<TComponent>(class_name);

        g_get_components_by_name_func_registry.unregisterFunc(class_name);
        g_get_components_by_path_func_registry.unregisterFunc(class_name);
        g_get_components_by_tag_func_registry.unregisterFunc(class_name);
        g_get_components_by_tag_any_func_registry.unregisterFunc(class_name);
        g_get_components_by_tag_all_func_registry.unregisterFunc(class_name);
        g_get_components_by_type_func_registry.unregisterFunc(class_name);

        g_get_components_by_name_as_map_func_registry.unregisterFunc(class_name);
        g_get_components_by_path_as_map_func_registry.unregisterFunc(class_name);
        g_get_components_by_tag_as_map_func_registry.unregisterFunc(class_name);
        g_get_components_by_tag_any_as_map_func_registry.unregisterFunc(class_name);
        g_get_components_by_tag_all_as_map_func_registry.unregisterFunc(class_name);
        g_get_components_by_type_as_map_func_registry.unregisterFunc(class_name);

        g_get_component_by_name_func_registry.unregisterFunc(class_name);
        g_get_component_by_path_func_registry.unregisterFunc(class_name);
        g_get_component_by_tag_func_registry.unregisterFunc(class_name);
        g_get_component_by_tag_any_func_registry.unregisterFunc(class_name);
        g_get_component_by_tag_all_func_registry.unregisterFunc(class_name);
        g_get_component_by_type_func_registry.unregisterFunc(class_name);
    }

    template <CInterface TInterface>
    static void unregisterInterface(const std::string& class_name)
    {
        unregisterClassCommon<TInterface>(class_name);
    }

    template <CClass TClass>
    static void unregisterClass(const std::string& class_name)
    {
        unregisterClassCommon<TClass>(class_name);

        g_new_object_func_registry.unregisterFunc(class_name);
        g_load_object_func_registry.unregisterFunc(class_name);
        g_load_class_func_registry.unregisterFunc(class_name);
    }

    template <typename TClass>
    static void unregisterClassCommon(const std::string& class_name)
    {
        unregisterStructCommon<TClass>(class_name);

        g_get_static_class_func_registry.unregisterFunc(class_name);
    }

    template <CStruct TStruct>
    static void unregisterStruct(const std::string& struct_name)
    {
        unregisterStructCommon<TStruct>(struct_name);
    }

    template <typename TSpecialStruct> requires
        (!CStruct<TSpecialStruct>)
    static void unregisterSpecialStruct(const std::string& struct_name)
    {
        unregisterStructCommon<TSpecialStruct>(struct_name);

        std::string type_id_string = Std::getTypeIdString<TSpecialStruct>();
        SP_ASSERT(Std::containsKey(g_special_struct_names, type_id_string) || Std::containsKey(g_special_structs, type_id_string));
        if (Std::containsKey(g_special_struct_names, type_id_string)) {
            SP_ASSERT(!Std::containsKey(g_special_structs, type_id_string));
            Std::remove(g_special_struct_names, type_id_string);
        }
        if (Std::containsKey(g_special_structs, type_id_string)) {
            SP_ASSERT(!Std::containsKey(g_special_struct_names, type_id_string));
            Std::remove(g_special_structs, type_id_string);
        }
    }

    template <typename TStruct>
    static void unregisterStructCommon(const std::string& struct_name)
    {
        g_get_static_struct_func_registry.unregisterFunc(struct_name);
    }

    //
    // Templated helper functions for getting static structs, can't be private because it is used by UnrealObj,
    // and can't be in the Unreal namespace because the special struct implementation depends on a map of
    // types maintained by UnrealClassRegistry.
    //

    template <typename TStructOrClassOrInterface> requires
        CStruct<TStructOrClassOrInterface> || CClass<TStructOrClassOrInterface> || CInterface<TStructOrClassOrInterface>
    static UStruct* getStaticStruct()
    {
        return Unreal::getStaticStruct<TStructOrClassOrInterface>();
    }

    template <typename TSpecialStruct> requires
        (!(CStruct<TSpecialStruct> || CClass<TSpecialStruct> || CInterface<TSpecialStruct>))
    static UStruct* getStaticStruct()
    {
        std::string type_id_string = Std::getTypeIdString<TSpecialStruct>();
        SP_ASSERT(Std::containsKey(g_special_struct_names, type_id_string) || Std::containsKey(g_special_structs, type_id_string));
        if (Std::containsKey(g_special_struct_names, type_id_string)) {
            SP_ASSERT(!Std::containsKey(g_special_structs, type_id_string));
            std::string name = g_special_struct_names.at(type_id_string);
            return Unreal::findSpecialStructByName(name);
        }
        if (Std::containsKey(g_special_structs, type_id_string)) {
            SP_ASSERT(!Std::containsKey(g_special_struct_names, type_id_string));
            return g_special_structs.at(type_id_string);
        }
        SP_ASSERT(false);
        return nullptr;
    }
};
