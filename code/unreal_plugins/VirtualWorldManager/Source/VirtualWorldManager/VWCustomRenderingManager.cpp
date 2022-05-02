#include "VWCustomRenderingManager.h"


UVWCustomRenderingManager::UVWCustomRenderingManager()
{
    ConstructorHelpers::FObjectFinder<UMaterialInterface> finder_diffuse(
        TEXT("Material'/VirtualWorldManager/Koolab/Materials/CelShader/PP_Diffuse.PP_Diffuse'"));
    if (finder_diffuse.Succeeded()) {
        this->pp_material_diffuse_ = finder_diffuse.Object;
    }

    ConstructorHelpers::FObjectFinder<UMaterial> finder_cel_shader(
        TEXT("Material'/VirtualWorldManager/Koolab/Materials/CelShader/PP_CelShader.PP_CelShader'"));
    if (finder_cel_shader.Succeeded()) {
        this->pp_material_cel_shader_ = finder_cel_shader.Object;
    }

    ConstructorHelpers::FObjectFinder<UMaterialInterface> finder_painter(
        TEXT("MaterialInstanceConstant'/VirtualWorldManager/Koolab/Materials/Painter/PPI_Kuwahara.PPI_Kuwahara'"));
    if (finder_painter.Succeeded()) {
        this->pp_material_painter_ = finder_painter.Object;
    }
}

void UVWCustomRenderingManager::setLambertianRendering(std::vector<AActor*>& actors, bool enable)
{
    for (auto& actor : actors) {
        UE_LOG(LogTemp, Warning, TEXT("actor %s"), *((actor)->GetName()));
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components) {
            UE_LOG(LogTemp, Warning, TEXT("----- comp %s"), *((component)->GetName()));
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);
            for (UMaterialInterface*& material : materials) {
                UE_LOG(LogTemp, Warning, TEXT("----- ---- %s"), *((material)->GetName()));
                if (enable && !material->IsA(UMaterialInstanceDynamic::StaticClass())) {
                    // replace current material with MID, and disable metallic,roughness and specular
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
