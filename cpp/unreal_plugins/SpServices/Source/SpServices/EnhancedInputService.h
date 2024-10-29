//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <string>
#include <vector>

#include <Engine/LocalPlayer.h>
#include <EnhancedInputSubsystems.h> // UEnhancedInputLocalPlayerSubsystem
#include <InputAction.h>
#include <InputModifiers.h>          // UInputModifierScalar
#include <InputTriggers.h>           // UInputTriggerPressed

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

#include "EnhancedInputService.generated.h"

// This struct is intended to be identical to Unreal's FInputActionValue struct, see Engine/Plugins/EnhancedInput/Source/EnhancedInput/Public/InputActionValue.h
USTRUCT()
struct FSpInputActionValue
{
    GENERATED_BODY()

    UPROPERTY()
    FVector Value;

    UPROPERTY()
    EInputActionValueType ValueType;
};

class EnhancedInputService : public Service {
public:
    EnhancedInputService() = delete;
    EnhancedInputService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        // Register EnhancedInput types. We can't do this in SpCore, because if SpCore depends on the EnhancedInput
        // plugin, we get cooking errors.
        UnrealClassRegistrar::registerSubsystemClass<ULocalPlayer, UEnhancedInputLocalPlayerSubsystem>("UEnhancedInputLocalPlayerSubsystem");
        UnrealClassRegistrar::registerClass<UInputAction>("UInputAction");
        UnrealClassRegistrar::registerClass<UInputModifierScalar>("UInputModifierScalar");
        UnrealClassRegistrar::registerClass<UInputTriggerPressed>("UInputTriggerPressed");

        unreal_entry_point_binder->bindFuncUnreal("enhanced_input_service", "inject_input_for_action",
            [this](
                uint64_t& enhanced_input_subsystem,
                uint64_t& action,
                std::string& raw_value_string,
                std::vector<uint64_t>& modifiers,
                std::vector<uint64_t>& triggers) -> void {

                UEnhancedInputLocalPlayerSubsystem* enhanced_input_subsystem_ptr = toPtr<UEnhancedInputLocalPlayerSubsystem>(enhanced_input_subsystem);

                FSpInputActionValue sp_raw_value;
                Unreal::setObjectPropertiesFromString(&sp_raw_value, FSpInputActionValue::StaticStruct(), raw_value_string);
                FInputActionValue raw_value(sp_raw_value.ValueType, sp_raw_value.Value);

                enhanced_input_subsystem_ptr->InjectInputForAction(
                    toPtr<UInputAction>(action),
                    raw_value,
                    Unreal::toTArray(toPtr<UInputModifier>(modifiers)),
                    Unreal::toTArray(toPtr<UInputTrigger>(triggers)));
            });
    }

    ~EnhancedInputService()
    {
        // Unregister EnhancedInput types.
        UnrealClassRegistrar::unregisterSubsystemClass<ULocalPlayer, UEnhancedInputLocalPlayerSubsystem>("UEnhancedInputLocalPlayerSubsystem");
        UnrealClassRegistrar::unregisterClass<UInputAction>("UInputAction");
        UnrealClassRegistrar::unregisterClass<UInputModifierScalar>("UInputModifierScalar");
        UnrealClassRegistrar::unregisterClass<UInputTriggerPressed>("UInputTriggerPressed");        
    }
};
