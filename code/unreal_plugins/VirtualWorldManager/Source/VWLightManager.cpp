#include "VWLightManager.h"

#include <EngineUtils.h>
#include <Engine/PostProcessVolume.h>
#include <Kismet/GameplayStatics.h>

#include "Assert.h"

void VWLightManager::SetGlobalIlluminationIntensity(UWorld* world, float intensity_scale)
{
    APostProcessVolume* post_process_volume = Cast<APostProcessVolume>(UGameplayStatics::GetActorOfClass(world, APostProcessVolume::StaticClass()));

    ASSERT(post_process_volume);

    post_process_volume->Settings.bOverride_IndirectLightingIntensity = true;
    post_process_volume->Settings.IndirectLightingIntensity = intensity_scale;
}

void VWLightManager::EnableDistanceFieldShadows(UWorld* world, bool enable)
{
    for (TActorIterator<ALight> it(world, ALight::StaticClass()); it; ++it) {
        ALight* light = *it;
        // override DistanceFieldShadow setting
        light->GetLightComponent()->bUseRayTracedDistanceFieldShadows = enable;
        // notify render configuration change
        light->MarkComponentsRenderStateDirty();
    }
}
