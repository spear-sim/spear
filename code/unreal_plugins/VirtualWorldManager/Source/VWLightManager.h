#pragma once

#include <CoreMinimal.h>
#include <Components/LightComponent.h>
#include <Engine/Light.h>

class VIRTUALWORLDMANAGER_API VWLightManager
{
public:
    // tune global illumination scale
    static void SetGlobalIlluminationIntensity(UWorld* world, float intensity_scale);

    // enable or disable distance field shadows for all lights in current world.
    // Distance Field Shadows will calculate real time shadow with better efficiency and soft shadow boundary.
    // However, it introduces artifact for interior scene, and Skeletal Mesh(OpenBot) have no shadow when it is enabled.
    static void EnableDistanceFieldShadows(UWorld* world, bool enable);
};
