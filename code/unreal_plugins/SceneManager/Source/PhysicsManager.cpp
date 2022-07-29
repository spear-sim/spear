#include "PhysicsManager.h"

#include <sstream>

#include <PhysicalMaterials/PhysicalMaterial.h>

#include "Assert.h"

std::map<int, UPhysicalMaterial*> PhysicsManager::physical_material_map_;

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
}

void PhysicsManager::terminate()
{
    // clear pointer
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
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);
            for (int32 i = 0; i < materials.Num(); i++)
            {
                auto& material = materials[i];
                if (!material->IsA(UMaterialInstanceDynamic::StaticClass()))
                {
                    UMaterialInstanceDynamic* dynamic_material = UMaterialInstanceDynamic::Create(material, component, FName(material->GetName() + "_Dynamic"));
                    dynamic_material->PhysMaterial = physical_material;
                    // update material
                    component->SetMaterial(i, dynamic_material);
                }
                else
                {
                    UMaterialInstanceDynamic* dynamic_material = Cast<UMaterialInstanceDynamic>(material);
                    dynamic_material->PhysMaterial = physical_material;
                    // update physical material if material is not changed
                    FBodyInstance* body_instance = component->GetBodyInstance();
                    if (body_instance && body_instance->IsValidBodyInstance())
                    {
                        body_instance->UpdatePhysicalMaterials();
                    }
                }
            }
        }
    }
}
