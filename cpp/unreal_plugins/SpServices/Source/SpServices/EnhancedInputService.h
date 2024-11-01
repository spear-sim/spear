//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <string>
#include <vector>

#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Engine/EngineBaseTypes.h>  // EInputEvent
#include <Engine/LocalPlayer.h>
#include <EnhancedInputComponent.h>  // FEnhancedInputActionEventBinding, FInputDebugKeyBinding, UEnhancedInputComponent
#include <EnhancedInputSubsystems.h> // UEnhancedInputLocalPlayerSubsystem
#include <Framework/Commands/InputChord.h>
#include <InputAction.h>             // FInputActionInstance
#include <InputActionValue.h>        // EInputActionValueType
#include <InputModifiers.h>          // UInputModifier, UInputModifierScalar
#include <InputTriggers.h>           // ETriggerEvent, UInputTrigger, UInputTriggerPressed
#include <Math/Vector.h>
#include <Templates/UniquePtr.h>
#include <UObject/ObjectPtr.h>

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
    FVector Value = FVector::ZeroVector;

    UPROPERTY()
    EInputActionValueType ValueType = EInputActionValueType::Axis3D;
};

USTRUCT()
struct FSpInputActionInstance : public FInputActionInstance
{
    GENERATED_BODY()

    // these set functions are needed to access protected data in FInputActionInstance
    void setInputActionValue(const FInputActionValue& input_action_value) { Value = input_action_value; }
    void setModifiers(const TArray<TObjectPtr<UInputModifier>>& modifiers) { Modifiers = modifiers; }
    void setTriggers(const TArray<TObjectPtr<UInputTrigger>>& triggers) { Triggers = triggers; }
};

class EnhancedInputService : public Service {
public:
    EnhancedInputService() = delete;
    EnhancedInputService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        // Register EnhancedInput types. We can't do this in SpCore, because if we set SpCore to depend on
        // the EnhancedInputPlugin, then we get cooking errors.
        UnrealClassRegistrar::registerSubsystemClass<ULocalPlayer, UEnhancedInputLocalPlayerSubsystem>("UEnhancedInputLocalPlayerSubsystem");
        UnrealClassRegistrar::registerClass<UInputAction>("UInputAction");
        UnrealClassRegistrar::registerClass<UInputModifierScalar>("UInputModifierScalar");
        UnrealClassRegistrar::registerClass<UInputTriggerPressed>("UInputTriggerPressed");

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("enhanced_input_service", "inject_input",
            [this](
                uint64_t& enhanced_input_subsystem,
                uint64_t& input_action,
                std::string& input_action_value_string,
                std::vector<uint64_t>& modifiers,
                std::vector<uint64_t>& triggers) -> void {

                UEnhancedInputLocalPlayerSubsystem* enhanced_input_subsystem_ptr = toPtr<UEnhancedInputLocalPlayerSubsystem>(enhanced_input_subsystem);

                FSpInputActionValue sp_input_action_value;
                Unreal::setObjectPropertiesFromString(&sp_input_action_value, FSpInputActionValue::StaticStruct(), input_action_value_string);
                FInputActionValue input_action_value(sp_input_action_value.ValueType, sp_input_action_value.Value);

                enhanced_input_subsystem_ptr->InjectInputForAction(
                    toPtr<UInputAction>(input_action),
                    input_action_value,
                    Unreal::toTArray(toPtr<UInputModifier>(modifiers)),
                    Unreal::toTArray(toPtr<UInputTrigger>(triggers)));
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("enhanced_input_service", "inject_input_for_actor",
            [this](
                uint64_t& actor,
                std::string& input_action_name,
                std::string& trigger_event_string,
                std::string& input_action_value_string,
                std::string& input_action_instance_string,
                std::vector<uint64_t>& modifiers,
                std::vector<uint64_t>& triggers,
                bool verbose) -> void {

                AActor* actor_ptr = toPtr<AActor>(actor);

                // we choose to give up, rather than assert, for consistent behavior with inject_input and inject_debug_key_for_actor
                std::vector<UEnhancedInputComponent*> enhanced_input_components = Unreal::getComponentsByType<UEnhancedInputComponent>(actor_ptr);
                if (enhanced_input_components.empty()) {
                    std::string actor_name;
                    if (Unreal::hasStableName(actor_ptr)) {
                        actor_name = Unreal::getStableName(actor_ptr);
                    } else {
                        actor_name = Unreal::toStdString(actor_ptr->GetName());
                    }
                    SP_LOG("Couldn't find a UEnhancedInputComponent on actor ", actor_name, ", giving up...");
                    return;
                }
                UEnhancedInputComponent* enhanced_input_component = enhanced_input_components.at(0);

                ETriggerEvent trigger_event = Unreal::getEnumValueFromString<ETriggerEvent>(trigger_event_string);

                TArray<TObjectPtr<UInputModifier>> modifiers_tarray = Unreal::toTArrayOf<TObjectPtr<UInputModifier>>(toPtr<UInputModifier>(modifiers));
                TArray<TObjectPtr<UInputTrigger>> triggers_tarray = Unreal::toTArrayOf<TObjectPtr<UInputTrigger>>(toPtr<UInputTrigger>(triggers));

                FSpInputActionValue sp_input_action_value;
                Unreal::setObjectPropertiesFromString(&sp_input_action_value, FSpInputActionValue::StaticStruct(), input_action_value_string);
                FInputActionValue input_action_value(sp_input_action_value.ValueType, sp_input_action_value.Value);

                FSpInputActionInstance sp_input_action_instance;
                Unreal::setObjectPropertiesFromString(&sp_input_action_instance, FSpInputActionInstance::StaticStruct(), input_action_instance_string);

                // The internal FInputActionValue in FInputActionInstance isn't exposed to the property
                // system, so we handle it separately using this set function. 
                sp_input_action_instance.setInputActionValue(input_action_value);

                // setObjectPropertiesFromString(...) can't be used to set pointers, so we handle modifiers
                // and triggers separately. We could use Unreal::setPropertyValueFromString(...) to do this,
                // but instead we choose to do it directly through these set functions that we define on our
                // FSpInputActionInstance class, which derives from FInputActionInstance to access protected
                // member variables.
                sp_input_action_instance.setModifiers(modifiers_tarray);
                sp_input_action_instance.setTriggers(triggers_tarray);

                const TArray<TUniquePtr<FEnhancedInputActionEventBinding>>& event_bindings = enhanced_input_component->GetActionEventBindings();
                for (const auto& event_binding : event_bindings) {
                    const UInputAction* input_action = event_binding->GetAction();
                    SP_ASSERT(input_action);

                    if (verbose) {
                        SP_LOG(Unreal::toStdString(input_action->GetName()));
                        SP_LOG(Unreal::getStringFromEnumValue(trigger_event));
                        SP_LOG();
                    }

                    if (input_action_name == Unreal::toStdString(input_action->GetName()) && trigger_event == event_binding->GetTriggerEvent()) {

                        // SourceAction is a private member variable on FSpInputActionInstance, but it is
                        // accessible through Unreal's property system, so we can still get and set it.
                        Unreal::PropertyDesc property_desc = Unreal::findPropertyByName(
                            &sp_input_action_instance, FSpInputActionInstance::StaticStruct(), "SourceAction");
                        Unreal::setPropertyValueFromString(property_desc, Std::toStringFromPtr(input_action));

                        event_binding->Execute(sp_input_action_instance);
                    }
                }
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("enhanced_input_service", "inject_debug_key_for_actor",
            [this](
                uint64_t& actor,
                std::string& chord_string,
                std::string& key_event_string,
                std::string& input_action_value_string,
                bool verbose) -> void {

                AActor* actor_ptr = toPtr<AActor>(actor);

                // we choose to give up, rather than assert, for consistent behavior with inject_input and inject_input_for_actor
                std::vector<UEnhancedInputComponent*> enhanced_input_components = Unreal::getComponentsByType<UEnhancedInputComponent>(actor_ptr);
                if (enhanced_input_components.empty()) {
                    std::string actor_name;
                    if (Unreal::hasStableName(actor_ptr)) {
                        actor_name = Unreal::getStableName(actor_ptr);
                    } else {
                        actor_name = Unreal::toStdString(actor_ptr->GetName());
                    }
                    SP_LOG("Couldn't find a UEnhancedInputComponent on actor ", actor_name, ", giving up...");
                    return;
                }
                UEnhancedInputComponent* enhanced_input_component = enhanced_input_components.at(0);

                FInputChord chord;
                Unreal::setObjectPropertiesFromString(&chord, FInputChord::StaticStruct(), chord_string);
                EInputEvent key_event = Unreal::getEnumValueFromString<EInputEvent>(key_event_string);

                FSpInputActionValue sp_input_action_value;
                Unreal::setObjectPropertiesFromString(&sp_input_action_value, FSpInputActionValue::StaticStruct(), input_action_value_string);
                FInputActionValue input_action_value(sp_input_action_value.ValueType, sp_input_action_value.Value);

                const TArray<TUniquePtr<FInputDebugKeyBinding>>& debug_key_bindings = enhanced_input_component->GetDebugKeyBindings();
                for (const auto& debug_key_binding : debug_key_bindings) {

                    if (verbose) {
                        SP_LOG(Unreal::getObjectPropertiesAsString(&(debug_key_binding->Chord), FInputChord::StaticStruct()));
                        SP_LOG(Unreal::getStringFromEnumValue(debug_key_binding->KeyEvent.GetValue()));
                        SP_LOG();
                    }

                    if (chord == debug_key_binding->Chord && key_event == debug_key_binding->KeyEvent.GetValue()) {
                        debug_key_binding->Execute(input_action_value);
                    }
                }
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
