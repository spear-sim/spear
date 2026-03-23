//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>         // uint64
#include <Kismet/BlueprintFunctionLibrary.h>
#include <Misc/FeedbackContext.h> // GWarn
#include <UObject/Object.h>       // UObject

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

#include "SpFuncUtils.generated.h"

UCLASS()
class USpFuncUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int64 ToHandleFromObject(UObject* Object)
    {
        SP_ASSERT(Object);
        return reinterpret_cast<int64>(Object);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UObject* ToObjectFromHandle(int64 Handle)
    {
        UObject* uobject = reinterpret_cast<UObject*>(Handle);
        SP_ASSERT(uobject);
        return uobject;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString ToStringFromObject(UObject* Object)
    {
        SP_ASSERT(Object);
        return Unreal::toFString(Std::toStringFromPtr(Object));
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UObject* ToObjectFromString(FString String)
    {
        UObject* uobject = Std::toPtrFromString<UObject>(Unreal::toStdString(String));
        SP_ASSERT(uobject);
        return uobject;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetStructPropertiesAsJsonStringFromExportText(UScriptStruct* ScriptStruct, const FString& ExportText)
    {
        SP_ASSERT(ScriptStruct);

        uint32 num_bytes = ScriptStruct->GetStructureSize();
        uint8_t initial_value = 0;
        std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> struct_data(num_bytes, initial_value);
        SP_ASSERT(ScriptStruct->GetMinAlignment() <= 4096);
        SP_ASSERT(4096 % ScriptStruct->GetMinAlignment() == 0);
        SP_ASSERT(Std::isPtrSufficientlyAligned(struct_data.data(), 4096));

        ScriptStruct->InitializeStruct(struct_data.data());
        const TCHAR* result = ScriptStruct->ImportText(
            Unreal::toTCharPtr(ExportText), // Buffer
            struct_data.data(),             // Value
            nullptr,                        // OwnerObject
            EPropertyPortFlags::PPF_None,   // PortFlags
            GWarn,                          // ErrorText
            ScriptStruct->GetName());       // StructName
        SP_ASSERT(result);

        FString json_string = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(struct_data.data(), ScriptStruct));
        ScriptStruct->DestroyStruct(struct_data.data());

        return json_string;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString GetStructPropertiesAsExportTextFromJsonString(UScriptStruct* ScriptStruct, const FString& PropertiesString)
    {
        SP_ASSERT(ScriptStruct);

        uint32 num_bytes = ScriptStruct->GetStructureSize();
        uint8_t initial_value = 0;
        std::vector<uint8_t, SpAlignedAllocator<uint8_t, 4096>> struct_data(num_bytes, initial_value);
        SP_ASSERT(ScriptStruct->GetMinAlignment() <= 4096);
        SP_ASSERT(4096 % ScriptStruct->GetMinAlignment() == 0);
        SP_ASSERT(Std::isPtrSufficientlyAligned(struct_data.data(), 4096));

        ScriptStruct->InitializeStruct(struct_data.data());
        UnrealUtils::setObjectPropertiesFromString(struct_data.data(), ScriptStruct, Unreal::toStdString(PropertiesString));

        FString export_text;
        ScriptStruct->ExportText(
            export_text,                    // ValueStr
            struct_data.data(),             // Value
            nullptr,                        // Defaults
            nullptr,                        // OwnerObject
            EPropertyPortFlags::PPF_None,   // PortFlags
            nullptr);                       // ExportRootScope
        ScriptStruct->DestroyStruct(struct_data.data());

        return export_text;
    }
};
