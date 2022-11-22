#include "PhysicsManager.h"

#include <sstream>

#include <Components/StaticMeshComponent.h>
#include <GameFramework/Actor.h>
#include <PhysicalMaterials/PhysicalMaterial.h>
#include <Materials/MaterialInstanceDynamic.h>

#include "Assert/Assert.h"

const int PHYSICAL_MATERIAL_ID_DEFAULT_START = 1000;
const int PHYSICAL_MATERIAL_ID_DEFAULT_END   = 1081;

const int PHYSICAL_MATERIAL_ID_DYNAMIC_START = 2000;

std::map<int, UPhysicalMaterial*> physical_materials_ = {};
int current_physical_material_id_ = -1;

void PhysicsManager::initialize()
{
    for (int physical_material_id = PHYSICAL_MATERIAL_ID_DEFAULT_START; physical_material_id < PHYSICAL_MATERIAL_ID_DEFAULT_END; physical_material_id++) {
        auto physical_material = LoadObject<UPhysicalMaterial>(nullptr, *FString::Printf(TEXT("/Game/Scene/PhyMaterials/PM_%d.PM_%d"), physical_material_id, physical_material_id));
        ASSERT(physical_material);
        physical_materials_[physical_material_id] = physical_material;
    }
    current_physical_material_id_ = PHYSICAL_MATERIAL_ID_DYNAMIC_START;
}

void PhysicsManager::terminate()
{
    physical_materials_.clear();
}

void PhysicsManager::setActorPhysicalMaterials(const std::vector<AActor*>& actors, int physical_material_id)
{
    // find physical material uasset
    UPhysicalMaterial* physical_material = physical_materials_.at(physical_material_id);
    ASSERT(physical_material);

    // for all actors...
    for (auto& actor : actors) {
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);

        // for all components...
        for (auto& component : components) {
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);

            // for all materials...
            for (int i = 0; i < materials.Num(); i++) {
                auto& material = materials[i];

                // if material is not a UMaterialInstanceDynamic, then create a new UMaterialInstanceDynamic with the desired properties and assign to the current material slot
                if (!material->IsA(UMaterialInstanceDynamic::StaticClass())) {
                    UMaterialInstanceDynamic* material_instance_dynamic = UMaterialInstanceDynamic::Create(material, component, FName(material->GetName() + "_Dynamic"));
                    material_instance_dynamic->PhysMaterial = physical_material;
                    component->SetMaterial(i, material_instance_dynamic);

                // otherwise update the material with the desired properties
                } else {
                    auto material_instance_dynamic = dynamic_cast<UMaterialInstanceDynamic*>(material);
                    material_instance_dynamic->PhysMaterial = physical_material;

                    FBodyInstance* body_instance = component->GetBodyInstance();
                    if (body_instance && body_instance->IsValidBodyInstance()) {
                        body_instance->UpdatePhysicalMaterials();
                    }
                }
            }
        }
    }
}

int PhysicsManager::createPhysicalMaterial(float friction, float density)
{
    int physical_material_id = current_physical_material_id_;

    auto physical_material = NewObject<UPhysicalMaterial>(static_cast<UObject*>(GetTransientPackage()), *FString::Printf(TEXT("PM_%d"), physical_material_id), EObjectFlags::RF_Standalone);
    physical_material->Friction = friction;
    physical_material->Density = density;
    physical_materials_[physical_material_id] = physical_material;

    current_physical_material_id_++;

    return physical_material_id;
}
