#pragma once

#include "CoreMinimal.h"
#include "Engine/Light.h"
#include "Components/LightComponent.h"
#include "VWLightManager.generated.h"

UCLASS()
class VIRTUALWORLDMANAGER_API UVWLightManager : public UObject
{
public:
    GENERATED_BODY()
         
    static void SetGI(UWorld* world, float val);

    static void SetDistanceField(UWorld* world, bool enable);
};
