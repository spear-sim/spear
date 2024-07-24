//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealClassRegistrar.h"

#include <map>
#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/PoseableMeshComponent.h>
#include <Components/SceneComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/TextureRenderTarget2D.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <Materials/Material.h>
#include <Materials/MaterialInterface.h>
#include <Math/Rotator.h>
#include <Math/Transform.h>
#include <Math/Vector.h>
#include <UObject/Object.h> // UObject

class FLinkerInstancingContext;
class UClass;
class UPackage;
class UPackageMap;
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
// Registrars for getting static classes using a class name instead of template parameters
//

CppFuncRegistrar<UClass*> g_get_static_class_registrar;
CppFuncRegistrar<UStruct*> g_get_static_struct_registrar;

//
// Registrars for spawning actors using a class name instead of template parameters
//

CppFuncRegistrar<AActor*, UWorld*, const FVector&, const FRotator&, const FActorSpawnParameters&> g_spawn_actor_registrar;

//
// Registrars for creating components using a class name instead of template parameters
//

CppFuncRegistrar<UActorComponent*, AActor*, const std::string&>                    g_create_component_outside_owner_constructor_registrar;
CppFuncRegistrar<USceneComponent*, AActor*, const std::string&>                    g_create_scene_component_outside_owner_constructor_from_actor_registrar;
CppFuncRegistrar<USceneComponent*, UObject*, USceneComponent*, const std::string&> g_create_scene_component_outside_owner_constructor_from_object_registrar;
CppFuncRegistrar<USceneComponent*, USceneComponent*, const std::string&>           g_create_scene_component_outside_owner_constructor_from_scene_component_registrar;

//
// Registrars for creating objects using a class name instead of template parameters
//

CppFuncRegistrar<UObject*, UObject*, FName, EObjectFlags, UObject*, bool, FObjectInstancingGraph*, UPackage*> g_new_object_registrar;
CppFuncRegistrar<UObject*, UObject*, const TCHAR*, const TCHAR*, uint32, UPackageMap*, const FLinkerInstancingContext*> g_load_object_registrar;
CppFuncRegistrar<UClass*, UObject*, const TCHAR*, const TCHAR*, uint32, UPackageMap*> g_load_class_registrar;

//
// Registrars for finding actors using a class name instead of template parameters
//

CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&, bool>           g_find_actors_by_name_registrar;
CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::string&>                              g_find_actors_by_tag_registrar;
CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 g_find_actors_by_tag_any_registrar;
CppFuncRegistrar<std::vector<AActor*>, const UWorld*, const std::vector<std::string>&>                 g_find_actors_by_tag_all_registrar;
CppFuncRegistrar<std::vector<AActor*>, const UWorld*>                                                  g_find_actors_by_type_registrar;

CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&, bool> g_find_actors_by_name_as_map_registrar;
CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::string&>                    g_find_actors_by_tag_as_map_registrar;
CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       g_find_actors_by_tag_any_as_map_registrar;
CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*, const std::vector<std::string>&>       g_find_actors_by_tag_all_as_map_registrar;
CppFuncRegistrar<std::map<std::string, AActor*>, const UWorld*>                                        g_find_actors_by_type_as_map_registrar;

CppFuncRegistrar<AActor*, const UWorld*, const std::string&, bool>                                     g_find_actor_by_name_registrar;
CppFuncRegistrar<AActor*, const UWorld*, const std::string&, bool, bool>                               g_find_actor_by_tag_registrar;
CppFuncRegistrar<AActor*, const UWorld*, const std::vector<std::string>&, bool, bool>                  g_find_actor_by_tag_any_registrar;
CppFuncRegistrar<AActor*, const UWorld*, const std::vector<std::string>&, bool, bool>                  g_find_actor_by_tag_all_registrar;
CppFuncRegistrar<AActor*, const UWorld*, bool, bool>                                                   g_find_actor_by_type_registrar;

//
// Registrars for getting components using a class name instead of template parameters
//

CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_components_by_name_registrar;
CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::string&, bool>                              g_get_components_by_tag_registrar;
CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_components_by_tag_any_registrar;
CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_components_by_tag_all_registrar;
CppFuncRegistrar<std::vector<UActorComponent*>, const AActor*, bool>                                                  g_get_components_by_type_registrar;

CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_components_by_name_as_map_registrar;
CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::string&, bool>                    g_get_components_by_tag_as_map_registrar;
CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_components_by_tag_any_as_map_registrar;
CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_components_by_tag_all_as_map_registrar;
CppFuncRegistrar<std::map<std::string, UActorComponent*>, const AActor*, bool>                                        g_get_components_by_type_as_map_registrar;

CppFuncRegistrar<UActorComponent*, const AActor*, const std::string&, bool, bool>                                     g_get_component_by_name_registrar;
CppFuncRegistrar<UActorComponent*, const AActor*, const std::string&, bool, bool, bool>                               g_get_component_by_tag_registrar;
CppFuncRegistrar<UActorComponent*, const AActor*, const std::vector<std::string>&, bool, bool, bool>                  g_get_component_by_tag_any_registrar;
CppFuncRegistrar<UActorComponent*, const AActor*, const std::vector<std::string>&, bool, bool, bool>                  g_get_component_by_tag_all_registrar;
CppFuncRegistrar<UActorComponent*, const AActor*, bool, bool, bool>                                                   g_get_component_by_type_registrar;

//
// Registrars for getting children components using a class name instead of template parameters
//

CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool>           g_get_children_components_by_name_from_actor_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::string&, bool>                              g_get_children_components_by_tag_from_actor_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_any_from_actor_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_all_from_actor_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const AActor*, bool>                                                  g_get_children_components_by_type_from_actor_registrar;

CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool, bool> g_get_children_components_by_name_as_map_from_actor_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::string&, bool>                    g_get_children_components_by_tag_as_map_from_actor_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_any_as_map_from_actor_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_all_as_map_from_actor_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const AActor*, bool>                                        g_get_children_components_by_type_as_map_from_actor_registrar;

CppFuncRegistrar<USceneComponent*, const AActor*, const std::string&, bool, bool>                                     g_get_child_component_by_name_from_actor_registrar;
CppFuncRegistrar<USceneComponent*, const AActor*, const std::string&, bool, bool, bool>                               g_get_child_component_by_tag_from_actor_registrar;
CppFuncRegistrar<USceneComponent*, const AActor*, const std::vector<std::string>&, bool, bool, bool>                  g_get_child_component_by_tag_any_from_actor_registrar;
CppFuncRegistrar<USceneComponent*, const AActor*, const std::vector<std::string>&, bool, bool, bool>                  g_get_child_component_by_tag_all_from_actor_registrar;
CppFuncRegistrar<USceneComponent*, const AActor*, bool, bool, bool>                                                   g_get_child_component_by_type_from_actor_registrar;

CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool>           g_get_children_components_by_name_from_scene_component_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::string&, bool>                              g_get_children_components_by_tag_from_scene_component_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_any_from_scene_component_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>                 g_get_children_components_by_tag_all_from_scene_component_registrar;
CppFuncRegistrar<std::vector<USceneComponent*>, const USceneComponent*, bool>                                                  g_get_children_components_by_type_from_scene_component_registrar;

CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool, bool> g_get_children_components_by_name_as_map_from_scene_component_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::string&, bool>                    g_get_children_components_by_tag_as_map_from_scene_component_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_any_as_map_from_scene_component_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, const std::vector<std::string>&, bool>       g_get_children_components_by_tag_all_as_map_from_scene_component_registrar;
CppFuncRegistrar<std::map<std::string, USceneComponent*>, const USceneComponent*, bool>                                        g_get_children_components_by_type_as_map_from_scene_component_registrar;

CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::string&, bool, bool>                                     g_get_child_component_by_name_from_scene_component_registrar;
CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::string&, bool, bool, bool>                               g_get_child_component_by_tag_from_scene_component_registrar;
CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool, bool, bool>                  g_get_child_component_by_tag_any_from_scene_component_registrar;
CppFuncRegistrar<USceneComponent*, const USceneComponent*, const std::vector<std::string>&, bool, bool, bool>                  g_get_child_component_by_tag_all_from_scene_component_registrar;
CppFuncRegistrar<USceneComponent*, const USceneComponent*, bool, bool, bool>                                                   g_get_child_component_by_type_from_scene_component_registrar;

//
// These maps are necessary to support getStaticStruct<T>() for special struct types that don't have a
// StaticStruct() method., e.g., FRotator and FVector.
//

std::map<std::string, std::string> g_special_struct_names; // map from platform-dependent type name to user-facing name
std::map<std::string, UStruct*> g_special_structs;         // map from platform-dependent type name to UStruct*


// Normally we would do the operations in initialize() and terminate(...) in the opposite order. But we make an
// exception here (i.e., we do the operations in the same order) to make it easier and less error-prone to add
// classes and functions.

void UnrealClassRegistrar::initialize()
{
    // Unreal classes
    registerActorClass<AActor>("AActor");
    registerActorClass<AStaticMeshActor>("AStaticMeshActor");
    registerComponentClass<UActorComponent>("UActorComponent");
    registerComponentClass<USceneComponent>("USceneComponent");
    registerComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    registerComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    registerClass<UObject>("UObject");
    registerClass<UGameplayStatics>("UGameplayStatics");
    registerClass<UMaterial>("UMaterial");
    registerClass<UMaterialInterface>("UMaterialInterface");
    registerClass<UStaticMesh>("UStaticMesh");
    registerClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");
    registerSpecialStruct<FRotator>("FRotator");
    registerSpecialStruct<FTransform>("FTransform");
    registerSpecialStruct<FVector>("FVector");
}

void UnrealClassRegistrar::terminate()
{
    // Unreal classes
    unregisterActorClass<AActor>("AActor");
    unregisterActorClass<AStaticMeshActor>("AStaticMeshActor");
    unregisterComponentClass<UActorComponent>("UActorComponent");
    unregisterComponentClass<USceneComponent>("USceneComponent");
    unregisterComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    unregisterComponentClass<UPoseableMeshComponent>("UPoseableMeshComponent");
    unregisterClass<UObject>("UObject");
    unregisterClass<UGameplayStatics>("UGameplayStatics");
    unregisterClass<UMaterial>("UMaterial");
    unregisterClass<UMaterialInterface>("UMaterialInterface");
    unregisterClass<UStaticMesh>("UStaticMesh");
    unregisterClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");
    unregisterSpecialStruct<FRotator>("FRotator");
    unregisterSpecialStruct<FTransform>("FTransform");
    unregisterSpecialStruct<FVector>("FVector");
}

//
// Get static class using a class name instead of template parameters
//

UClass* UnrealClassRegistrar::getStaticClass(const std::string& class_name) {
    return g_get_static_class_registrar.call(class_name);
}

UStruct* UnrealClassRegistrar::getStaticStruct(const std::string& struct_name) {
    return g_get_static_struct_registrar.call(struct_name);
}

//
// Find actors using a class name instead of template parameters
//

std::vector<AActor*> UnrealClassRegistrar::findActorsByName(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return g_find_actors_by_name_registrar.call(class_name, world, names, return_null_if_not_found);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return g_find_actors_by_tag_registrar.call(class_name, world, tag);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_any_registrar.call(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_all_registrar.call(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByType(const std::string& class_name, const UWorld* world) {
    return g_find_actors_by_type_registrar.call(class_name, world);
}

//

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return g_find_actors_by_name_as_map_registrar.call(class_name, world, names, return_null_if_not_found);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAsMap(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return g_find_actors_by_tag_as_map_registrar.call(class_name, world, tag);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAnyAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_any_as_map_registrar.call(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAllAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return g_find_actors_by_tag_all_as_map_registrar.call(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTypeAsMap(const std::string& class_name, const UWorld* world) {
    return g_find_actors_by_type_as_map_registrar.call(class_name, world);
}

//

AActor* UnrealClassRegistrar::findActorByName(const std::string& class_name, const UWorld* world, const std::string& name, bool assert_if_not_found) {
    return g_find_actor_by_name_registrar.call(class_name, world, name, assert_if_not_found);
}
    
AActor* UnrealClassRegistrar::findActorByTag(const std::string& class_name, const UWorld* world, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_find_actor_by_tag_registrar.call(class_name, world, tag, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_find_actor_by_tag_any_registrar.call(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_find_actor_by_tag_all_registrar.call(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByType(const std::string& class_name, const UWorld* world, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_find_actor_by_type_registrar.call(class_name, world, assert_if_not_found, assert_if_multiple_found);
}

//
// Get components using a class name instead of template parameters
//

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByName(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& names, bool include_from_child_actors, bool return_null_if_not_found) {
    return g_get_components_by_name_registrar.call(class_name, actor, names, include_from_child_actors, return_null_if_not_found);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTag(
    const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors) {
    return g_get_components_by_tag_registrar.call(class_name, actor, tag, include_from_child_actors);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTagAny(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_any_registrar.call(class_name, actor, tags, include_from_child_actors);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTagAll(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_all_registrar.call(class_name, actor, tags, include_from_child_actors);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByType(
    const std::string& class_name, const AActor* actor, bool include_from_child_actors) {
    return g_get_components_by_type_registrar.call(class_name, actor, include_from_child_actors);
}

//

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByNameAsMap(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& names, bool include_from_child_actors, bool return_null_if_not_found) {
    return g_get_components_by_name_as_map_registrar.call(class_name, actor, names, include_from_child_actors, return_null_if_not_found);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAsMap(
    const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors) {
    return g_get_components_by_tag_as_map_registrar.call(class_name, actor, tag, include_from_child_actors);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAnyAsMap(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_any_as_map_registrar.call(class_name, actor, tags, include_from_child_actors);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAllAsMap(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors) {
    return g_get_components_by_tag_all_as_map_registrar.call(class_name, actor, tags, include_from_child_actors);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTypeAsMap(
    const std::string& class_name, const AActor* actor, bool include_from_child_actors) {
    return g_get_components_by_type_as_map_registrar.call(class_name, actor, include_from_child_actors);
}

//

UActorComponent* UnrealClassRegistrar::getComponentByName(
    const std::string& class_name, const AActor* actor, const std::string& name, bool include_from_child_actors, bool assert_if_not_found) {
    return g_get_component_by_name_registrar.call(class_name, actor, name, include_from_child_actors, assert_if_not_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTag(
    const std::string& class_name, const AActor* actor, const std::string& tag, bool include_from_child_actors, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_component_by_tag_registrar.call(class_name, actor, tag, include_from_child_actors, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTagAny(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_component_by_tag_any_registrar.call(class_name, actor, tags, include_from_child_actors, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTagAll(
    const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool include_from_child_actors, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_component_by_tag_all_registrar.call(class_name, actor, tags, include_from_child_actors, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByType(
    const std::string& class_name, const AActor* actor, bool include_from_child_actors, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_component_by_type_registrar.call(class_name, actor, include_from_child_actors, assert_if_not_found, assert_if_multiple_found);
}

//
// Get children components using a class name and an AActor* instead of template parameters
//

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByName(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_from_actor_registrar.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTag(
    const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_from_actor_registrar.call(class_name, parent, tag, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAny(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_from_actor_registrar.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAll(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_from_actor_registrar.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByType(
    const std::string& class_name, const AActor* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_from_actor_registrar.call(class_name, parent, include_all_descendants);
}

//

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByNameAsMap(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_as_map_from_actor_registrar.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAsMap(
    const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_as_map_from_actor_registrar.call(class_name, parent, tag, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_as_map_from_actor_registrar.call(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_as_map_from_actor_registrar.call(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(
    const std::string& class_name, const AActor* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_as_map_from_actor_registrar.call(class_name, parent, include_all_descendants);
}

//

USceneComponent* UnrealClassRegistrar::getChildComponentByName(
    const std::string& class_name, const AActor* parent, const std::string& name, bool include_all_descendants, bool assert_if_not_found) {
    return g_get_child_component_by_name_from_actor_registrar.call(class_name, parent, name, include_all_descendants, assert_if_not_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTag(
    const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_tag_from_actor_registrar.call(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTagAny(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_tag_any_from_actor_registrar.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTagAll(
    const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_tag_all_from_actor_registrar.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByType(
    const std::string& class_name, const AActor* parent, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_type_from_actor_registrar.call(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

//
// Get children components using a class name and a USceneComponent* instead of template parameters
//

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByName(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_from_scene_component_registrar.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTag(
    const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_from_scene_component_registrar.call(class_name, parent, tag, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAny(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_from_scene_component_registrar.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAll(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_from_scene_component_registrar.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByType(
    const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_from_scene_component_registrar.call(class_name, parent, include_all_descendants);
}

//

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByNameAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return g_get_children_components_by_name_as_map_from_scene_component_registrar.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return g_get_children_components_by_tag_as_map_from_scene_component_registrar.call(class_name, parent, tag, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_any_as_map_from_scene_component_registrar.call(class_name, parent, tags, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return g_get_children_components_by_tag_all_as_map_from_scene_component_registrar.call(class_name, parent, tags, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(
    const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return g_get_children_components_by_type_as_map_from_scene_component_registrar.call(class_name, parent, include_all_descendants);
}

//

USceneComponent* UnrealClassRegistrar::getChildComponentByName(
    const std::string& class_name, const USceneComponent* parent, const std::string& name, bool include_all_descendants, bool assert_if_not_found) {
    return g_get_child_component_by_name_from_scene_component_registrar.call(class_name, parent, name, include_all_descendants, assert_if_not_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByTag(
    const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_tag_from_scene_component_registrar.call(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByTagAny(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_tag_any_from_scene_component_registrar.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByTagAll(
    const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_tag_all_from_scene_component_registrar.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByType(
    const std::string& class_name, const USceneComponent* parent, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return g_get_child_component_by_type_from_scene_component_registrar.call(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

//
// Spawn actor using a class name instead of template parameters
//

AActor* UnrealClassRegistrar::spawnActor(const std::string& class_name, UWorld* world, const FVector& location, const FRotator& rotation, const FActorSpawnParameters& spawn_parameters) {
    return g_spawn_actor_registrar.call(class_name, world, location, rotation, spawn_parameters);
}

//
// Create component using a class name instead of template parameters
//

UActorComponent* UnrealClassRegistrar::createComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& name) {
    return g_create_component_outside_owner_constructor_registrar.call(class_name, owner, name);
}

USceneComponent* UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& name) {
    return g_create_scene_component_outside_owner_constructor_from_actor_registrar.call(class_name, owner, name);
}

USceneComponent* UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, UObject* owner, USceneComponent* parent, const std::string& name) {
    return g_create_scene_component_outside_owner_constructor_from_object_registrar.call(class_name, owner, parent, name);
}

USceneComponent* UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, USceneComponent* owner, const std::string& name) {
    return g_create_scene_component_outside_owner_constructor_from_scene_component_registrar.call(class_name, owner, name);
}

//
// Create new object using a class name instead of template parameters
//

UObject* UnrealClassRegistrar::newObject(
    const std::string& class_name,
    UObject* outer,
    FName name,
    EObjectFlags flags,
    UObject* uobject_template,
    bool copy_transients_from_class_defaults,
    FObjectInstancingGraph* in_instance_graph,
    UPackage* external_package)
{
    return g_new_object_registrar.call(class_name, outer, name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph, external_package);
}

//
// Load objects and classes using a class name instead of template parameters
//

UObject* UnrealClassRegistrar::loadObject(
    const std::string& class_name, UObject* outer, const TCHAR* name, const TCHAR* filename, uint32 load_flags, UPackageMap* sandbox, const FLinkerInstancingContext* instancing_context)
{
    return g_load_object_registrar.call(class_name, outer, name, filename, load_flags, sandbox, instancing_context);
}

UObject* UnrealClassRegistrar::loadClass(const std::string& class_name, UObject* outer, const TCHAR* name, const TCHAR* filename, uint32 load_flags, UPackageMap* sandbox)
{
    return g_load_class_registrar.call(class_name, outer, name, filename, load_flags, sandbox);
}
