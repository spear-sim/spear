//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>              // int32
#include <Kismet/BlueprintFunctionLibrary.h>
#include <PerQualityLevelProperties.h> // EPerQualityLevels
#include <Scalability.h>               // Scalability::FQualityLevels, Scalability::GetQualityLevels, Scalability::SetQualityLevels
#include <UObject/ObjectMacros.h>      // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY, USTRUCT

#include "SpCore/Unreal.h"

#include "SpScalability.generated.h"

//
// FSpQualityLevels mirrors the scalability state applied by Scalability::SetQualityLevels(...), i.e., the
// ResolutionQuality and the eleven scalability group levels of Scalability::FQualityLevels declared in
// Engine/Source/Runtime/Engine/Public/Scalability.h. We omit the CPU/GPU benchmark members of FQualityLevels
// because they are not part of the quality state that GetQualityLevels()/SetQualityLevels() reads or writes.
//
// The group levels are stored as int32 in FQualityLevels, but their canonical values (0-4) correspond to the
// named Low/Medium/High/Epic/Cinematic levels, so we expose them here as EPerQualityLevels and cast when
// translating to and from FQualityLevels.
//

USTRUCT(BlueprintType)
struct FSpQualityLevels
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    float ResolutionQuality = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels ViewDistanceQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels AntiAliasingQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels ShadowQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels GlobalIlluminationQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels ReflectionQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels PostProcessQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels TextureQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels EffectsQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels FoliageQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels ShadingQuality = EPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    EPerQualityLevels LandscapeQuality = EPerQualityLevels::Epic;
};

UCLASS()
class USpScalability : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevels()
    {
        return toSpQualityLevels(Scalability::GetQualityLevels());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetQualityLevels(const FSpQualityLevels& QualityLevels)
    {
        Scalability::SetQualityLevels(toQualityLevels(QualityLevels));
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetFromSingleQualityLevelRelativeToMax(FSpQualityLevels& QualityLevels, int32 Value)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetFromSingleQualityLevelRelativeToMax(Value);
        QualityLevels = toSpQualityLevels(quality_levels);
    }

private:

    static FSpQualityLevels toSpQualityLevels(const Scalability::FQualityLevels& quality_levels)
    {
        FSpQualityLevels sp_quality_levels;
        sp_quality_levels.ResolutionQuality          = quality_levels.ResolutionQuality;
        sp_quality_levels.ViewDistanceQuality        = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.ViewDistanceQuality);
        sp_quality_levels.AntiAliasingQuality        = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.AntiAliasingQuality);
        sp_quality_levels.ShadowQuality              = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.ShadowQuality);
        sp_quality_levels.GlobalIlluminationQuality  = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.GlobalIlluminationQuality);
        sp_quality_levels.ReflectionQuality          = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.ReflectionQuality);
        sp_quality_levels.PostProcessQuality         = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.PostProcessQuality);
        sp_quality_levels.TextureQuality             = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.TextureQuality);
        sp_quality_levels.EffectsQuality             = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.EffectsQuality);
        sp_quality_levels.FoliageQuality             = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.FoliageQuality);
        sp_quality_levels.ShadingQuality             = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.ShadingQuality);
        sp_quality_levels.LandscapeQuality           = Unreal::getEnumValueAs<EPerQualityLevels>(quality_levels.LandscapeQuality);
        return sp_quality_levels;
    }

    static Scalability::FQualityLevels toQualityLevels(const FSpQualityLevels& sp_quality_levels)
    {
        // Scalability::FQualityLevels default-constructs with SetDefaults(), which we rely on to initialize the
        // CPU/GPU benchmark members that we don't mirror. Scalability::SetQualityLevels(...) clamps each level
        // internally, so we can assign the mirrored fields directly.

        Scalability::FQualityLevels quality_levels;
        quality_levels.ResolutionQuality          = sp_quality_levels.ResolutionQuality;
        quality_levels.ViewDistanceQuality        = Unreal::getEnumValue(sp_quality_levels.ViewDistanceQuality);
        quality_levels.AntiAliasingQuality        = Unreal::getEnumValue(sp_quality_levels.AntiAliasingQuality);
        quality_levels.ShadowQuality              = Unreal::getEnumValue(sp_quality_levels.ShadowQuality);
        quality_levels.GlobalIlluminationQuality  = Unreal::getEnumValue(sp_quality_levels.GlobalIlluminationQuality);
        quality_levels.ReflectionQuality          = Unreal::getEnumValue(sp_quality_levels.ReflectionQuality);
        quality_levels.PostProcessQuality         = Unreal::getEnumValue(sp_quality_levels.PostProcessQuality);
        quality_levels.TextureQuality             = Unreal::getEnumValue(sp_quality_levels.TextureQuality);
        quality_levels.EffectsQuality             = Unreal::getEnumValue(sp_quality_levels.EffectsQuality);
        quality_levels.FoliageQuality             = Unreal::getEnumValue(sp_quality_levels.FoliageQuality);
        quality_levels.ShadingQuality             = Unreal::getEnumValue(sp_quality_levels.ShadingQuality);
        quality_levels.LandscapeQuality           = Unreal::getEnumValue(sp_quality_levels.LandscapeQuality);
        return quality_levels;
    }
};
