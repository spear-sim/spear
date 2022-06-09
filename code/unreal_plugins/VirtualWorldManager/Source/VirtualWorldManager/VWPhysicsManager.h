#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

class VWPhysicsManager
{
public:
    /**
    * override physical material and change friction, density and restition
    * 
    * @param actors: 
    * @param physical_material_id: target physical material id£¬ valid range (1000-1081)
    * @return true if physical_material_id is valid and override existing value
    */
    static bool updatePhysicalMaterial(TArray<AActor*>& actors, int physical_material_id);
};
