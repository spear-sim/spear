#pragma once

#include "CoreMinimal.h"
//#include "IPlatformFilePak.h"
#include "VWLevelManager.generated.h"

enum EPostProcessMaterialType
{
    Default = 0,
    Semantic,
    Diffuse,
    CelShader,
    Painter
};

UCLASS()
class VIRTUALWORLDMANAGER_API AVWLevelManager : public AActor
{
public:
    GENERATED_BODY()
    AVWLevelManager();

    bool mountPakFromPath(const FString& pak_path);

    void getAllMapsInPak(TArray<FString>& map_list);
    /**
     * @brief acchieve lambert rendering by adjust asset material by disable metallic,roughness and specular
     *
     * @return true - enable lambert rendering
     * @return false - disable lambert rendering
     */
    void LambertRendering(bool is_enable = true);

    UMaterialInterface* getPostProcessMaterial(EPostProcessMaterialType material_type);

    UPROPERTY()
    UMaterialInterface* pp_material_semantic_ = nullptr;
    UPROPERTY()
    UMaterialInterface* pp_material_diffuse_ = nullptr;
    UPROPERTY()
    UMaterialInterface* pp_material_cel_shader_ = nullptr;
    UPROPERTY()
    UMaterialInterface* pp_material_painter_ = nullptr;
};
