#pragma once

#include <vector>

#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

class SCENEMANAGER_API PhysicsManager
{
public:
    /**
    * override physical material and change friction, density and restitution
    * for physical properties, see https://docs.google.com/spreadsheets/d/11JbTSHHoz9FwQuzz22PhfiPR1exE8vxkJBjvkPc4Ceo/edit
    * 
    * @param actors: target actor need to override physical property
    * @param physical_material_id: target physical material id, valid range (1000-1081)
    * @return true if physical_material_id is valid and override existing value
    */
    static bool updatePhysicalMaterial(std::vector<AActor*>& actors, int physical_material_id);
};
