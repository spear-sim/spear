//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>              // int32
#include <Kismet/BlueprintFunctionLibrary.h>
#include <PerQualityLevelProperties.h> // EPerQualityLevels
#include <Scalability.h>               // Scalability::FQualityLevels, Scalability::GetQualityLevels, Scalability::SetQualityLevels
#include <UObject/ObjectMacros.h>      // GENERATED_BODY, UCLASS, UENUM, UFUNCTION, UPROPERTY, USTRUCT

#include "SpCore/Unreal.h"

#include "SpScalability.generated.h"

//
// ESpPerQualityLevels mirrors the named Low/Medium/High/Epic/Cinematic levels of EPerQualityLevels declared in
// Engine/Source/Runtime/Engine/Public/PerQualityLevelProperties.h, and additionally defines an Error constant set
// to -1. The Error constant lets us represent the "custom" result that Scalability::FQualityLevels::GetSingleQualityLevel()
// returns when the group levels are not all set to the same value, which EPerQualityLevels (a uint8 enum) cannot
// represent.
//

UENUM() // -1 not supported for BlueprintType
enum class ESpPerQualityLevels
{
    Error     = -1,
    Low       = Unreal::getConstEnumValue(EPerQualityLevels::Low),
    Medium    = Unreal::getConstEnumValue(EPerQualityLevels::Medium),
    High      = Unreal::getConstEnumValue(EPerQualityLevels::High),
    Epic      = Unreal::getConstEnumValue(EPerQualityLevels::Epic),
    Cinematic = Unreal::getConstEnumValue(EPerQualityLevels::Cinematic),
    Num       = Unreal::getConstEnumValue(EPerQualityLevels::Num)
};

//
// FSpQualityLevels mirrors the scalability state applied by Scalability::SetQualityLevels(...), i.e., the
// ResolutionQuality and the eleven scalability group levels of Scalability::FQualityLevels declared in
// Engine/Source/Runtime/Engine/Public/Scalability.h. We omit the CPU/GPU benchmark members of FQualityLevels
// because they are not part of the quality state that GetQualityLevels()/SetQualityLevels() reads or writes.
//
// The group levels are stored as int32 in FQualityLevels, but their canonical values (0-4) correspond to the
// named Low/Medium/High/Epic/Cinematic levels, so we expose them here as ESpPerQualityLevels and cast when
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
    ESpPerQualityLevels ViewDistanceQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels AntiAliasingQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels ShadowQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels GlobalIlluminationQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels ReflectionQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels PostProcessQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels TextureQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels EffectsQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels FoliageQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels ShadingQuality = ESpPerQualityLevels::Epic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpPerQualityLevels LandscapeQuality = ESpPerQualityLevels::Epic;
};

UCLASS()
class USpQualityLevelsUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevels()
    {
        return toSpQualityLevels(Scalability::FQualityLevels());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromSingleQualityLevel(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetFromSingleQualityLevel(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetFromSingleQualityLevel(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetFromSingleQualityLevel(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromSingleQualityLevelRelativeToMax(int32 Value)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetFromSingleQualityLevelRelativeToMax(Value);
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetFromSingleQualityLevelRelativeToMax(FSpQualityLevels& QualityLevels, int32 Value)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetFromSingleQualityLevelRelativeToMax(Value);
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromViewDistanceQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetViewDistanceQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetViewDistanceQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetViewDistanceQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromAntiAliasingQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetAntiAliasingQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetAntiAliasingQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetAntiAliasingQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromShadowQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetShadowQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetShadowQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetShadowQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromGlobalIlluminationQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetGlobalIlluminationQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetGlobalIlluminationQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetGlobalIlluminationQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromReflectionQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetReflectionQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetReflectionQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetReflectionQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromPostProcessQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetPostProcessQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetPostProcessQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetPostProcessQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromTextureQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetTextureQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetTextureQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetTextureQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromEffectsQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetEffectsQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetEffectsQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetEffectsQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromFoliageQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetFoliageQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetFoliageQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetFoliageQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromShadingQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetShadingQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetShadingQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetShadingQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromLandscapeQuality(ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetLandscapeQuality(Unreal::getEnumValue(QualityLevel));
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetLandscapeQuality(FSpQualityLevels& QualityLevels, ESpPerQualityLevels QualityLevel)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetLandscapeQuality(Unreal::getEnumValue(QualityLevel));
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevelsFromBenchmarkFallback()
    {
        Scalability::FQualityLevels quality_levels;
        quality_levels.SetBenchmarkFallback();
        return toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetBenchmarkFallback(FSpQualityLevels& QualityLevels)
    {
        Scalability::FQualityLevels quality_levels = toQualityLevels(QualityLevels);
        quality_levels.SetBenchmarkFallback();
        QualityLevels = toSpQualityLevels(quality_levels);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static ESpPerQualityLevels GetSingleQualityLevel(const FSpQualityLevels& QualityLevels)
    {
        return Unreal::getEnumValueAs<ESpPerQualityLevels>(toQualityLevels(QualityLevels).GetSingleQualityLevel());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static ESpPerQualityLevels GetMinQualityLevel(const FSpQualityLevels& QualityLevels)
    {
        return Unreal::getEnumValueAs<ESpPerQualityLevels>(toQualityLevels(QualityLevels).GetMinQualityLevel());
    }

    static FSpQualityLevels toSpQualityLevels(const Scalability::FQualityLevels& quality_levels)
    {
        FSpQualityLevels sp_quality_levels;
        sp_quality_levels.ResolutionQuality          = quality_levels.ResolutionQuality;
        sp_quality_levels.ViewDistanceQuality        = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.ViewDistanceQuality);
        sp_quality_levels.AntiAliasingQuality        = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.AntiAliasingQuality);
        sp_quality_levels.ShadowQuality              = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.ShadowQuality);
        sp_quality_levels.GlobalIlluminationQuality  = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.GlobalIlluminationQuality);
        sp_quality_levels.ReflectionQuality          = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.ReflectionQuality);
        sp_quality_levels.PostProcessQuality         = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.PostProcessQuality);
        sp_quality_levels.TextureQuality             = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.TextureQuality);
        sp_quality_levels.EffectsQuality             = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.EffectsQuality);
        sp_quality_levels.FoliageQuality             = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.FoliageQuality);
        sp_quality_levels.ShadingQuality             = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.ShadingQuality);
        sp_quality_levels.LandscapeQuality           = Unreal::getEnumValueAs<ESpPerQualityLevels>(quality_levels.LandscapeQuality);
        return sp_quality_levels;
    }

    static Scalability::FQualityLevels toQualityLevels(const FSpQualityLevels& sp_quality_levels)
    {
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

UCLASS()
class USpScalability : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FSpQualityLevels GetQualityLevels()
    {
        return USpQualityLevelsUtils::toSpQualityLevels(Scalability::GetQualityLevels());
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetQualityLevels(const FSpQualityLevels& QualityLevels)
    {
        Scalability::SetQualityLevels(USpQualityLevelsUtils::toQualityLevels(QualityLevels));
    }
};
