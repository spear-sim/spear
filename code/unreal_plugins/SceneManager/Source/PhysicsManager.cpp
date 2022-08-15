#include "PhysicsManager.h"

#include <sstream>

#include <PhysicalMaterials/PhysicalMaterial.h>

#include "Assert.h"

// range for default physical material id
const static int PHYSICAL_MATERIAL_ID_DEFAULT_START = 1000;
const static int PHYSICAL_MATERIAL_ID_DEFAULT_END = 1081;
// starting point for dynamically created physical material id
const static int PHYSICAL_MATERIAL_ID_DYNAMIC_START = 2000;

std::map<int, UPhysicalMaterial*> PhysicsManager::physical_material_map_;
int PhysicsManager::physical_material_id_counter_;

void PhysicsManager::initialize()
{
    for (int physical_material_id = PHYSICAL_MATERIAL_ID_DEFAULT_START; physical_material_id < PHYSICAL_MATERIAL_ID_DEFAULT_END; physical_material_id++){
        std::ostringstream oss;
        oss << "/Game/Scene/PhyMaterials/PM_" << physical_material_id << ".PM_" << physical_material_id;
        UPhysicalMaterial* physical_material = LoadObject<UPhysicalMaterial>(nullptr, UTF8_TO_TCHAR(oss.str().c_str()));
        if (physical_material){
            physical_material_map_[physical_material_id] = physical_material;
        }
    }
    physical_material_id_counter_ = PHYSICAL_MATERIAL_ID_DYNAMIC_START;
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

    for (auto& actor : actors){
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components){
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);
            for (int32 i = 0; i < materials.Num(); i++){
                auto& material = materials[i];
                if (!material->IsA(UMaterialInstanceDynamic::StaticClass())){
                    UMaterialInstanceDynamic* dynamic_material = UMaterialInstanceDynamic::Create(material, component, FName(material->GetName() + "_Dynamic"));
                    dynamic_material->PhysMaterial = physical_material;
                    // update material
                    component->SetMaterial(i, dynamic_material);
                }else{
                    UMaterialInstanceDynamic* dynamic_material = Cast<UMaterialInstanceDynamic>(material);
                    dynamic_material->PhysMaterial = physical_material;
                    // update physical material if material is not changed
                    FBodyInstance* body_instance = component->GetBodyInstance();
                    if (body_instance && body_instance->IsValidBodyInstance()){
                        body_instance->UpdatePhysicalMaterials();
                    }
                }
            }
        }
    }
}

int PhysicsManager::createPhysicalMaterial(float friction, float density)
{
    int physical_material_id = physical_material_id_counter_;
    std::ostringstream oss;
    oss << "PM_" << physical_material_id;
    UPhysicalMaterial* physical_material = NewObject<UPhysicalMaterial>((UObject*)GetTransientPackage(), FName(UTF8_TO_TCHAR(oss.str().c_str())), EObjectFlags::RF_Standalone);
    physical_material->Friction = friction;
    physical_material->Density = density;

    physical_material_map_[physical_material_id] = physical_material;
    physical_material_id_counter_++;
    return physical_material_id;
}
