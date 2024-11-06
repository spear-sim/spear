//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "SpCore/SpFuncArray.h"
#include "SpCore/Unreal.h"

#include "SpFuncArrayTypes.generated.h"

UENUM()
enum class ESpFuncArrayDataType
{
    Invalid = Unreal::getConstEnumValue(SpFuncArrayDataType::Invalid),
    UInt8   = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt8),
    Int8    = Unreal::getConstEnumValue(SpFuncArrayDataType::Int8),
    UInt16  = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt16),
    Int16   = Unreal::getConstEnumValue(SpFuncArrayDataType::Int16),
    UInt32  = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt32),
    Int32   = Unreal::getConstEnumValue(SpFuncArrayDataType::Int32),
    UInt64  = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt64),
    Int64   = Unreal::getConstEnumValue(SpFuncArrayDataType::Int64),
    Float32 = Unreal::getConstEnumValue(SpFuncArrayDataType::Float32),
    Float64 = Unreal::getConstEnumValue(SpFuncArrayDataType::Float64)
};
