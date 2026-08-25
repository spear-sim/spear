//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>         // uint8
#include <RHIFeatureLevel.h>      // ERHIFeatureLevel
#include <UObject/ObjectMacros.h> // UENUM

#include "SpCore/Unreal.h"

#include "SpRHIFeatureLevel.generated.h"

//
// This enum corresponds to ERHIFeatureLevel::Type declared in Engine/Source/Runtime/RHI/Public/RHIFeatureLevel.h
//

UENUM(BlueprintType)
enum class ESpRHIFeatureLevel : uint8
{
    ES2_REMOVED = Unreal::getConstEnumValue(ERHIFeatureLevel::ES2_REMOVED),
    ES3_1       = Unreal::getConstEnumValue(ERHIFeatureLevel::ES3_1),
    SM4_REMOVED = Unreal::getConstEnumValue(ERHIFeatureLevel::SM4_REMOVED),
    SM5         = Unreal::getConstEnumValue(ERHIFeatureLevel::SM5),
    SM6         = Unreal::getConstEnumValue(ERHIFeatureLevel::SM6),
    Num         = Unreal::getConstEnumValue(ERHIFeatureLevel::Num)
};
