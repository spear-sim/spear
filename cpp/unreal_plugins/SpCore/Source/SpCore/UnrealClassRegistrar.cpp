//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealClassRegistrar.h"

#include <map>
#include <string>
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMesh.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Kismet/GameplayStatics.h>
#include <Materials/Material.h>
#include <Materials/MaterialInterface.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "SpCore/SpCoreActor.h"

class AActor;
class FLinkerInstancingContext;
class UActorComponent;
class UClass;
class UObject;
class UPackage;
class UPackageMap;
class USceneComponent;
class UWorld;
struct FActorSpawnParameters;
struct FObjectInstancingGraph;

// Normally we would do the operations in initialize() and terminate(...) in the opposite order. But we make an
// exception here (i.e., we do the operations in the same order) to make it easier and less error-prone to add
// classes and functions.

void UnrealClassRegistrar::initialize()
{
    // Unreal classes
    registerActorClass<AStaticMeshActor>("AStaticMeshActor");
    registerComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    registerClass<UGameplayStatics>("UGameplayStatics");
    registerClass<UMaterial>("UMaterial");
    registerClass<UMaterialInterface>("UMaterialInterface");
    registerClass<UStaticMesh>("UStaticMesh");
    registerClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");

    // SpCore classes
    registerActorClass<ASpCoreActor>("ASpCoreActor");

    // need to register special structs, i.e., structs that don't define a StaticStruct() method
    registerSpecialStruct<FRotator>("FRotator");
    registerSpecialStruct<FVector>("FVector");
}

void UnrealClassRegistrar::terminate()
{
    // Unreal classes
    unregisterActorClass<AStaticMeshActor>("AStaticMeshActor");
    unregisterComponentClass<UStaticMeshComponent>("UStaticMeshComponent");
    unregisterClass<UGameplayStatics>("UGameplayStatics");
    unregisterClass<UMaterial>("UMaterial");
    unregisterClass<UMaterialInterface>("UMaterialInterface");
    unregisterClass<UStaticMesh>("UStaticMesh");
    unregisterClass<UTextureRenderTarget2D>("UTextureRenderTarget2D");

    // SpCore classes
    unregisterActorClass<ASpCoreActor>("ASpCoreActor");

    // need to unregister special structs
    unregisterSpecialStruct<FRotator>();
    unregisterSpecialStruct<FVector>();
}

//
// Get static class using a class name instead of template parameters
//

UClass* UnrealClassRegistrar::getStaticClass(const std::string& class_name) {
    return s_get_static_class_registrar_.call(class_name);
}

//
// Get Default object using a class name instead of template parameters
//

UObject* UnrealClassRegistrar::getDefaultObject(const std::string& class_name) {
    return s_get_default_object_registrar_.call(class_name);
}

//
// Spawn actor using a class name instead of template parameters
//

AActor* UnrealClassRegistrar::spawnActor(const std::string& class_name, UWorld* world, const FVector& location, const FRotator& rotation, const FActorSpawnParameters& spawn_parameters) {
    return s_spawn_actor_registrar_.call(class_name, world, location, rotation, spawn_parameters);
}

//
// Create component using a class name instead of template parameters
//

UActorComponent* UnrealClassRegistrar::createComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& name) {
    return s_create_component_outside_owner_constructor_registrar_.call(class_name, owner, name);
}

USceneComponent* UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, AActor* owner, const std::string& name) {
    return s_create_scene_component_outside_owner_constructor_from_actor_registrar_.call(class_name, owner, name);
}

USceneComponent* UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, UObject* owner, USceneComponent* parent, const std::string& name) {
    return s_create_scene_component_outside_owner_constructor_from_object_registrar_.call(class_name, owner, parent, name);
}

USceneComponent* UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(const std::string& class_name, USceneComponent* owner, const std::string& name) {
    return s_create_scene_component_outside_owner_constructor_from_scene_component_registrar_.call(class_name, owner, name);
}

//
// Create object using a class name instead of template parameters
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
    return s_new_object_registrar_.call(class_name, outer, name, flags, uobject_template, copy_transients_from_class_defaults, in_instance_graph, external_package);
}

UObject* UnrealClassRegistrar::loadObject(
    const std::string& class_name,
    UObject* outer,
    const TCHAR* name,
    const TCHAR* filename,
    uint32 load_flags,
    UPackageMap* sandbox,
    const FLinkerInstancingContext* instancing_context)
{
    return s_load_object_registrar_.call(class_name, outer, name, filename, load_flags, sandbox, instancing_context);
}

//
// Find actors using a class name instead of template parameters
//

std::vector<AActor*> UnrealClassRegistrar::findActorsByName(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return s_find_actors_by_name_registrar_.call(class_name, world, names, return_null_if_not_found);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTag(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return s_find_actors_by_tag_registrar_.call(class_name, world, tag);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_any_registrar_.call(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_all_registrar_.call(class_name, world, tags);
}

std::vector<AActor*> UnrealClassRegistrar::findActorsByType(const std::string& class_name, const UWorld* world) {
    return s_find_actors_by_type_registrar_.call(class_name, world);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByNameAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return s_find_actors_by_name_as_map_registrar_.call(class_name, world, names, return_null_if_not_found);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAsMap(const std::string& class_name, const UWorld* world, const std::string& tag) {
    return s_find_actors_by_tag_as_map_registrar_.call(class_name, world, tag);
}

std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAnyAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_any_as_map_registrar_.call(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTagAllAsMap(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags) {
    return s_find_actors_by_tag_all_as_map_registrar_.call(class_name, world, tags);
}
    
std::map<std::string, AActor*> UnrealClassRegistrar::findActorsByTypeAsMap(const std::string& class_name, const UWorld* world) {
    return s_find_actors_by_type_as_map_registrar_.call(class_name, world);
}
    
AActor* UnrealClassRegistrar::findActorByName(const std::string& class_name, const UWorld* world, const std::string& name, bool assert_if_not_found) {
    return s_find_actor_by_name_registrar_.call(class_name, world, name, assert_if_not_found);
}
    
AActor* UnrealClassRegistrar::findActorByTag(const std::string& class_name, const UWorld* world, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_tag_registrar_.call(class_name, world, tag, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByTagAny(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_tag_any_registrar_.call(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByTagAll(const std::string& class_name, const UWorld* world, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_tag_all_registrar_.call(class_name, world, tags, assert_if_not_found, assert_if_multiple_found);
}
    
AActor* UnrealClassRegistrar::findActorByType(const std::string& class_name, const UWorld* world, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_find_actor_by_type_registrar_.call(class_name, world, assert_if_not_found, assert_if_multiple_found);
}

//
// Get components using a class name instead of template parameters
//

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByName(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return s_get_components_by_name_registrar_.call(class_name, actor, names, return_null_if_not_found);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTag(const std::string& class_name, const AActor* actor, const std::string& tag) {
    return s_get_components_by_tag_registrar_.call(class_name, actor, tag);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_any_registrar_.call(class_name, actor, tags);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_all_registrar_.call(class_name, actor, tags);
}

std::vector<UActorComponent*> UnrealClassRegistrar::getComponentsByType(const std::string& class_name, const AActor* actor) {
    return s_get_components_by_type_registrar_.call(class_name, actor);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByNameAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& names, bool return_null_if_not_found) {
    return s_get_components_by_name_as_map_registrar_.call(class_name, actor, names, return_null_if_not_found);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAsMap(const std::string& class_name, const AActor* actor, const std::string& tag) {
    return s_get_components_by_tag_as_map_registrar_.call(class_name, actor, tag);
}

std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAnyAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_any_as_map_registrar_.call(class_name, actor, tags);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTagAllAsMap(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags) {
    return s_get_components_by_tag_all_as_map_registrar_.call(class_name, actor, tags);
}
    
std::map<std::string, UActorComponent*> UnrealClassRegistrar::getComponentsByTypeAsMap(const std::string& class_name, const AActor* actor) {
    return s_get_components_by_type_as_map_registrar_.call(class_name, actor);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByName(const std::string& class_name, const AActor* actor, const std::string& name, bool assert_if_not_found) {
    return s_get_component_by_name_registrar_.call(class_name, actor, name, assert_if_not_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTag(const std::string& class_name, const AActor* actor, const std::string& tag, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_tag_registrar_.call(class_name, actor, tag, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTagAny(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_tag_any_registrar_.call(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByTagAll(const std::string& class_name, const AActor* actor, const std::vector<std::string>& tags, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_tag_all_registrar_.call(class_name, actor, tags, assert_if_not_found, assert_if_multiple_found);
}
    
UActorComponent* UnrealClassRegistrar::getComponentByType(const std::string& class_name, const AActor* actor, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_component_by_type_registrar_.call(class_name, actor, assert_if_not_found, assert_if_multiple_found);
}

//
// Get children components using a class name and an AActor* instead of template parameters
//

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByName(const std::string& class_name, const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return s_get_children_components_by_name_from_actor_registrar_.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTag(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants) {
    return s_get_children_components_by_tag_from_actor_registrar_.call(class_name, parent, tag, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAny(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_any_from_actor_registrar_.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAll(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_all_from_actor_registrar_.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByType(const std::string& class_name, const AActor* parent, bool include_all_descendants) {
    return s_get_children_components_by_type_from_actor_registrar_.call(class_name, parent, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByNameAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return s_get_children_components_by_name_as_map_from_actor_registrar_.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAsMap(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants) {
    return s_get_children_components_by_tag_as_map_from_actor_registrar_.call(class_name, parent, tag, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_any_as_map_from_actor_registrar_.call(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_all_as_map_from_actor_registrar_.call(class_name, parent, tags, include_all_descendants);
}
    
std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(const std::string& class_name, const AActor* parent, bool include_all_descendants) {
    return s_get_children_components_by_type_as_map_from_actor_registrar_.call(class_name, parent, include_all_descendants);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByName(const std::string& class_name, const AActor* parent, const std::string& name, bool include_all_descendants, bool assert_if_not_found) {
    return s_get_child_component_by_name_from_actor_registrar_.call(class_name, parent, name, include_all_descendants, assert_if_not_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTag(const std::string& class_name, const AActor* parent, const std::string& tag, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_from_actor_registrar_.call(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTagAny(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_any_from_actor_registrar_.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByTagAll(const std::string& class_name, const AActor* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_all_from_actor_registrar_.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
    
USceneComponent* UnrealClassRegistrar::getChildComponentByType(const std::string& class_name, const AActor* parent, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_type_from_actor_registrar_.call(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

//
// Get children components using a class name and a USceneComponent* instead of template parameters
//

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByName(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return s_get_children_components_by_name_from_scene_component_registrar_.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return s_get_children_components_by_tag_from_scene_component_registrar_.call(class_name, parent, tag, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_any_from_scene_component_registrar_.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_all_from_scene_component_registrar_.call(class_name, parent, tags, include_all_descendants);
}

std::vector<USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return s_get_children_components_by_type_from_scene_component_registrar_.call(class_name, parent, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByNameAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& names, bool include_all_descendants, bool return_null_if_not_found) {
    return s_get_children_components_by_name_as_map_from_scene_component_registrar_.call(class_name, parent, names, include_all_descendants, return_null_if_not_found);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAsMap(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants) {
    return s_get_children_components_by_tag_as_map_from_scene_component_registrar_.call(class_name, parent, tag, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_any_as_map_from_scene_component_registrar_.call(class_name, parent, tags, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants) {
    return s_get_children_components_by_tag_all_as_map_from_scene_component_registrar_.call(class_name, parent, tags, include_all_descendants);
}

std::map<std::string, USceneComponent*> UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants) {
    return s_get_children_components_by_type_as_map_from_scene_component_registrar_.call(class_name, parent, include_all_descendants);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByName(const std::string& class_name, const USceneComponent* parent, const std::string& name, bool include_all_descendants, bool assert_if_not_found) {
    return s_get_child_component_by_name_from_scene_component_registrar_.call(class_name, parent, name, include_all_descendants, assert_if_not_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByTag(const std::string& class_name, const USceneComponent* parent, const std::string& tag, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_from_scene_component_registrar_.call(class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByTagAny(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_any_from_scene_component_registrar_.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByTagAll(const std::string& class_name, const USceneComponent* parent, const std::vector<std::string>& tags, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_tag_all_from_scene_component_registrar_.call(class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}

USceneComponent* UnrealClassRegistrar::getChildComponentByType(const std::string& class_name, const USceneComponent* parent, bool include_all_descendants, bool assert_if_not_found, bool assert_if_multiple_found) {
    return s_get_child_component_by_type_from_scene_component_registrar_.call(class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found);
}
