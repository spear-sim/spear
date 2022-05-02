#pragma once

#include "CoreMinimal.h"

#include <vector>

#include "VWCustomRenderingManager.generated.h"

UCLASS()
class VIRTUALWORLDMANAGER_API UVWCustomRenderingManager : public UObject
{
public:
    GENERATED_BODY()
    UVWCustomRenderingManager();
	/**
     * @brief achieve lambertian rendering by disable metallic, roughness and specular parts in asset material
     *
     * @param actors - actors in the world that like to change rendering mode
     * @param enable - enable lambert rendering
     */
	void setLambertianRendering(std::vector<AActor*>& actors, bool enable);
	
    UMaterialInterface* pp_material_diffuse_ = nullptr;
    UMaterialInterface* pp_material_cel_shader_ = nullptr;
    UMaterialInterface* pp_material_painter_ = nullptr;

};
