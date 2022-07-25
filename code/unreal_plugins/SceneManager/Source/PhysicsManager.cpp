#include "PhysicsManager.h"

#include <sstream>

#include <PhysicalMaterials/PhysicalMaterial.h>

#include "Assert.h"

std::map<int, UPhysicalMaterial*> PhysicsManager::physical_material_map_;
int PhysicsManager::physical_material_counter;

void PhysicsManager::initialize()
{
    for (int physical_material_id = 1000; physical_material_id < 1081; physical_material_id++)
    {
        std::ostringstream oss;
        oss << "/Game/Scene/PhyMaterials/PM_" << physical_material_id << ".PM_" << physical_material_id;
        UPhysicalMaterial* physical_material = LoadObject<UPhysicalMaterial>(nullptr, UTF8_TO_TCHAR(oss.str().c_str()));
        if (physical_material)
        {
            physical_material_map_[physical_material_id] = physical_material;
        }
    }
    // initialize counter from 2000 to avoid conflict with existing pm
    physical_material_counter = 2000;
}

void PhysicsManager::terminate()
{
    for (auto& kvp : physical_material_map_)
    {
        UPhysicalMaterial* physical_material = kvp.second;
        if (physical_material)
        {
            physical_material->ConditionalBeginDestroy();
        }
    }
    physical_material_map_.clear();
}

void PhysicsManager::setActorPhysicalMaterials(const std::vector<AActor*>& actors, int physical_material_id)
{
    // find physical material uasset
    UPhysicalMaterial* physical_material = physical_material_map_.at(physical_material_id);
    // check if physical material is valid
    ASSERT(physical_material);

    for (auto& actor : actors)
    {
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components)
        {
            FBodyInstance* body_instance = component->GetBodyInstance();
            body_instance->SetPhysMaterialOverride(physical_material);
        }
    }
}

int PhysicsManager::createPhysicalMaterial(float friction, float density)
{
    int physical_material_id = physical_material_counter;
    std::ostringstream oss;
    oss << "PM_" << physical_material_id;
    UPhysicalMaterial* physical_material = NewObject<UPhysicalMaterial>((UObject*)GetTransientPackage(), FName(UTF8_TO_TCHAR(oss.str().c_str())), EObjectFlags::RF_Standalone);

    ASSERT(physical_material);

    physical_material->Friction = friction;
    physical_material->Density = density;

    physical_material_map_[physical_material_id] = physical_material;
    physical_material_counter++;

    return physical_material_id;
}
