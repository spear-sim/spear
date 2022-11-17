#pragma once

#include <map>
#include <string>
#include <vector>

#include <CoreMinimal.h>

class AActor;

class SCENEMANAGER_API PhysicsManager
{
public:
    static void initialize();
    static void terminate();

    // Override physical material for a collection of actors. For physical properties, see:
    // https://docs.google.com/spreadsheets/d/11JbTSHHoz9FwQuzz22PhfiPR1exE8vxkJBjvkPc4Ceo/edit
    static void setActorPhysicalMaterials(const std::vector<AActor*>& actors, int physical_material_id);

    // Create new physical material
    static int createPhysicalMaterial(float friction, float density);
};
