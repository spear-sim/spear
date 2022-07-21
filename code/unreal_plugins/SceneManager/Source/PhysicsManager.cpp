#include "PhysicsManager.h"

#include <PhysicalMaterials/PhysicalMaterial.h>

#include "Assert.h"

void PhysicsManager::setPhysicalMaterial(const std::vector<AActor*>& actors, int physical_material_id)
{
    //find physical material uasset
    char char_array[100];
    snprintf(char_array, sizeof(char_array), "/Game/Scene/PhyMaterials/PM_%d.PM_%d", physical_material_id,physical_material_id);
    UPhysicalMaterial* override_physical_material = LoadObject<UPhysicalMaterial>(nullptr, *FString(UTF8_TO_TCHAR(char_array)));

    // check if physical material is valid
    ASSERT(override_physical_material);

    for (auto& actor : actors) {
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components) {
            FBodyInstance* body_instance = component->GetBodyInstance();
            body_instance->SetPhysMaterialOverride(override_physical_material);
        }
    }
}
