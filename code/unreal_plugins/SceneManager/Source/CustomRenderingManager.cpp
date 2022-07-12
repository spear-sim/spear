#include "CustomRenderingManager.h"

void CustomRenderingManager::setLambertianRendering(std::vector<AActor*>& actors, bool enable)
{
    for (auto& actor : actors) {
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components) {
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);
            for (UMaterialInterface*& material : materials) {
                if (enable && !material->IsA(UMaterialInstanceDynamic::StaticClass())) {
                    // replace current material with MID, and disable metallic, roughness and specular
                    UMaterialInstanceDynamic* dynamic_material = UMaterialInstanceDynamic::Create(material, component, FName(material->GetName() + "_Dynamic"));
                    dynamic_material->SetVectorParameterValue("Metallic_Color", FVector::ZeroVector);
                    dynamic_material->SetVectorParameterValue("Gloss_Color", FVector::ZeroVector);
                    dynamic_material->SetVectorParameterValue("Specular_Color", FVector::ZeroVector);
                    component->SetMaterial(0, dynamic_material);
                }
                else {
                    // if mid is already setup, switch config
                    UMaterialInstanceDynamic* dynamic_material = Cast<UMaterialInstanceDynamic>(material);
                    if (dynamic_material != nullptr) {
                        if (enable) {
                            // override settings if not exist
                            dynamic_material->SetVectorParameterValue("Metallic_Color", FVector::ZeroVector);
                            dynamic_material->SetVectorParameterValue("Gloss_Color", FVector::ZeroVector);
                            dynamic_material->SetVectorParameterValue("Specular_Color", FVector::ZeroVector);
                        }
                        else {
                            // clear override setting to disable lambert
                            dynamic_material->ClearParameterValues();
                        }
                    }
                }
            }
        }
    }
}
