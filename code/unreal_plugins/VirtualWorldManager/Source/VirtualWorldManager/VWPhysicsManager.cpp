#include "VWPhysicsManager.h"

bool VWPhysicsManager::updatePhysicalMaterial(TArray<AActor*>& actors, int physical_material_id)
{
    FString val = FString::FromInt(physical_material_id);
    FString formatted = FString::Printf(TEXT("/Game/Scene/PhyMaterials/PM_%s.PM_%s"), *(val), *(val));
    UPhysicalMaterial* overridePhysicalMaterial = LoadObject<UPhysicalMaterial>(nullptr, *formatted);
    if (overridePhysicalMaterial == nullptr) {
        UE_LOG(LogTemp, Warning, TEXT("overridePhysicalMaterial not found :%s"), *formatted);
        return false;
    }
    for (auto& actor : actors) {
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components) {
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);
            auto& material = materials[0];
            if (!material->IsA(UMaterialInstanceDynamic::StaticClass())) {
                UMaterialInstanceDynamic* dynamic_material = UMaterialInstanceDynamic::Create(material, component, FName(material->GetName() + "_Dynamic"));
                dynamic_material->PhysMaterial = overridePhysicalMaterial;
                component->SetMaterial(0, dynamic_material);
            }
            else {
                UMaterialInstanceDynamic* dynamic_material = Cast<UMaterialInstanceDynamic>(material);
                dynamic_material->PhysMaterial = overridePhysicalMaterial;
                FBodyInstance* BodyInst = component->GetBodyInstance();
                if (BodyInst && BodyInst->IsValidBodyInstance()) {
                    BodyInst->UpdatePhysicalMaterials();
                }
            }
        }
    }
    return true;
}