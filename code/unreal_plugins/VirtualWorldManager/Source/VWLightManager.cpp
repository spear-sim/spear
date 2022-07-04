#include "VWLightManager.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"

#include "Engine/PostProcessVolume.h"

void UVWLightManager::SetGI(UWorld* world, float val)
{
    APostProcessVolume* ppVolume = Cast<APostProcessVolume>(UGameplayStatics::GetActorOfClass(world, APostProcessVolume::StaticClass()));
    if (ppVolume == nullptr) {
        UE_LOG(LogTemp, Warning, TEXT("PostProcessVolume not found"));
        return;
    }
    ppVolume->Settings.bOverride_IndirectLightingIntensity = true;
    ppVolume->Settings.IndirectLightingIntensity = val;
}

void UVWLightManager::SetDistanceField(UWorld* world, bool enable)
{
    for (TActorIterator<ALight> it(world, ALight::StaticClass()); it; ++it) {
        ALight* light = *it;
        light->GetLightComponent()->bUseRayTracedDistanceFieldShadows = enable;
        light->MarkComponentsRenderStateDirty();
    }
}