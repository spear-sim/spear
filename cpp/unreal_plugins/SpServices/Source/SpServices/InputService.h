//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <string>
#include <vector>

#include <Components/InputComponent.h>
#include <Engine/EngineBaseTypes.h> // EInputEvent
#include <InputCoreTypes.h>         // ETouchType
#include <Framework/Commands/InputChord.h>
#include <GameFramework/Pawn.h>
#include <Math/Vector.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

// The purpose of this wrapper class is to enable us to call APawn::SetupPlayerInputComponent(...), which is
// protected, from outside the APawn class hierarchy. We don't use the usual Unreal naming convention here to
// emphasize that InputServicePawnWrapper is not intended to be a UCLASS.
class InputServicePawnWrapper : public APawn
{
public:
    using APawn::SetupPlayerInputComponent; // equivalent to creating a public method that calls the protected method
};

class InputService : public Service
{
public:
    InputService() = delete;
    InputService(CUnrealEntryPointBinder auto* unreal_entry_point_binder) : Service("InputService")
    {
        SP_ASSERT(unreal_entry_point_binder);

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "setup_player_input_component", 
            [this](uint64_t& actor, uint64_t& input_component) -> void {
                APawn* pawn_ptr = toPtr<APawn>(actor);
                SP_ASSERT(pawn_ptr);

                UInputComponent* input_component_ptr = toPtr<UInputComponent>(input_component);
                SP_ASSERT(input_component_ptr);

                //
                // The following cast is unsafe because the runtime type of pawn is not InputServicePawnWrapper.
                // But we need to apply this unsafe operation because we want to call APawn::SetupPlayerInputComponent(...),
                // which is protected, from outside the APawn class hierarchy.
                //
                // Our overall approach here is unsafe because it makes the following two assumptions,
                // neither of which is mandated by the C++ standard, but both of which are valid on Clang and
                // MSVC. First, we must assume that offsetof(InputServicePawnWrapper, vptr) == offsetof(APawn, vptr),
                // where InputServicePawnWrapper::vptr and APawn::vptr are the implicit member variables that
                // point to the vtables for InputServicePawnWrapper and APawn respectively, and are located
                // at offset 0 in typical vtable implementations. Second, we must assume that the vtable
                // entries corresponding to InputServicePawnWrapper::SetupPlayerInputComponent(...) and
                // APawn::SetupPlayerInputComponent(...) occur at the same vtable index, which would be the
                // case in typical vtable implementations, because InputServicePawnWrapper inherits from APawn.
                //

                static_cast<InputServicePawnWrapper*>(pawn_ptr)->SetupPlayerInputComponent(input_component_ptr);
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "inject_key_for_actor",
            [this](uint64_t& actor, std::string& chord_string, std::string& key_event_string) -> void {

                UInputComponent* input_component = getInputComponent(actor);
                if (!input_component) {
                    return;
                }

                FInputChord chord;
                Unreal::setObjectPropertiesFromString(&chord, FInputChord::StaticStruct(), chord_string);
                EInputEvent key_event = Unreal::getEnumValueFromString<EInputEvent>(key_event_string);

                for (auto& key_binding : input_component->KeyBindings) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INPUT_SERVICE.PRINT_INJECT_DEBUG_INFO")) {
                        SP_LOG(Unreal::getObjectPropertiesAsString(&(key_binding.Chord), FInputChord::StaticStruct()));
                        SP_LOG(Unreal::getStringFromEnumValue(key_binding.KeyEvent.GetValue()));
                        SP_LOG();
                    }

                    if (chord == key_binding.Chord && key_event == key_binding.KeyEvent.GetValue()) {
                        key_binding.KeyDelegate.Execute(chord.Key);
                    }
                }
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "inject_touch_for_actor",
            [this](uint64_t& actor, std::string& key_event_string, std::string& finger_index_string, std::string& location_string) -> void {

                UInputComponent* input_component = getInputComponent(actor);
                if (!input_component) {
                    return;
                }

                EInputEvent key_event = Unreal::getEnumValueFromString<EInputEvent>(key_event_string);
                ETouchIndex::Type finger_index = Unreal::getEnumValueFromString<ETouchIndex::Type>(finger_index_string);
                FVector location;
                Unreal::setObjectPropertiesFromString(&location, UnrealClassRegistrar::getStaticStruct<FVector>(), location_string);

                for (auto& touch_binding : input_component->TouchBindings) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INPUT_SERVICE.PRINT_INJECT_DEBUG_INFO")) {
                        SP_LOG(Unreal::getStringFromEnumValue(touch_binding.KeyEvent.GetValue()));
                        SP_LOG();
                    }

                    if (key_event == touch_binding.KeyEvent.GetValue()) {
                        touch_binding.TouchDelegate.Execute(finger_index, location);
                    }
                }
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "inject_axis_for_actor",
            [this](uint64_t& actor, std::string& axis_name, float& axis_value) -> void {

                UInputComponent* input_component = getInputComponent(actor);
                if (!input_component) {
                    return;
                }

                for (auto& axis_binding : input_component->AxisBindings) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INPUT_SERVICE.PRINT_INJECT_DEBUG_INFO")) {
                        SP_LOG(Unreal::toStdString(axis_binding.AxisName));
                        SP_LOG(axis_binding.AxisValue);
                        SP_LOG();
                    }

                    if (axis_name == Unreal::toStdString(axis_binding.AxisName)) {
                        axis_binding.AxisDelegate.Execute(axis_value);
                    }
                }
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "inject_axis_key_for_actor",
            [this](uint64_t& actor, std::string& axis_key_name, float& axis_key_value) -> void {

                UInputComponent* input_component = getInputComponent(actor);
                if (!input_component) {
                    return;
                }

                for (auto& axis_key_binding : input_component->AxisKeyBindings) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INPUT_SERVICE.PRINT_INJECT_DEBUG_INFO")) {
                        SP_LOG(Unreal::toStdString(axis_key_binding.AxisKey.GetDisplayName()));
                        SP_LOG(Unreal::toStdString(axis_key_binding.AxisKey.GetFName()));
                        SP_LOG(Unreal::toStdString(axis_key_binding.AxisKey.ToString()));
                        SP_LOG(axis_key_binding.AxisValue);
                        SP_LOG();
                    }

                    if (axis_key_name == Unreal::toStdString(axis_key_binding.AxisKey.GetDisplayName())) {
                        axis_key_binding.AxisDelegate.Execute(axis_key_value);
                    }
                }
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "inject_vector_axis_for_actor",
            [this](uint64_t& actor, std::string& vector_axis_name, std::string& vector_axis_value_string) -> void {

                UInputComponent* input_component = getInputComponent(actor);
                if (!input_component) {
                    return;
                }

                FVector vector_axis_value;
                Unreal::setObjectPropertiesFromString(&vector_axis_value, UnrealClassRegistrar::getStaticStruct<FVector>(), vector_axis_value_string);

                for (auto& vector_axis_binding : input_component->VectorAxisBindings) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INPUT_SERVICE.PRINT_INJECT_DEBUG_INFO")) {
                        SP_LOG(Unreal::toStdString(vector_axis_binding.AxisKey.GetDisplayName()));
                        SP_LOG(Unreal::toStdString(vector_axis_binding.AxisKey.GetFName()));
                        SP_LOG(Unreal::toStdString(vector_axis_binding.AxisKey.ToString()));
                        SP_LOG(Unreal::getObjectPropertiesAsString(&(vector_axis_binding.AxisValue), UnrealClassRegistrar::getStaticStruct<FVector>()));
                        SP_LOG();
                    }

                    if (vector_axis_name == Unreal::toStdString(vector_axis_binding.AxisKey.GetDisplayName())) {
                        vector_axis_binding.AxisDelegate.Execute(vector_axis_value);
                    }
                }
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "inject_gesture_for_actor",
            [this](uint64_t& actor, std::string& gesture_name, float& gesture_value) -> void {

                UInputComponent* input_component = getInputComponent(actor);
                if (!input_component) {
                    return;
                }

                for (auto& gesture_binding : input_component->GestureBindings) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INPUT_SERVICE.PRINT_INJECT_DEBUG_INFO")) {
                        SP_LOG(Unreal::toStdString(gesture_binding.GestureKey.GetDisplayName()));
                        SP_LOG(Unreal::toStdString(gesture_binding.GestureKey.GetFName()));
                        SP_LOG(Unreal::toStdString(gesture_binding.GestureKey.ToString()));
                        SP_LOG(gesture_binding.GestureValue);
                        SP_LOG();
                    }

                    if (gesture_name == Unreal::toStdString(gesture_binding.GestureKey.GetDisplayName())) {
                        gesture_binding.GestureDelegate.Execute(gesture_value);
                    }
                }
            });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("input_service", "inject_action_for_actor",
            [this](uint64_t& actor, std::string& action_name, std::string& key_event_string, std::string& key_name) -> void {

                UInputComponent* input_component = getInputComponent(actor);
                if (!input_component) {
                    return;
                }

                EInputEvent key_event = Unreal::getEnumValueFromString<EInputEvent>(key_event_string);
                FKey key(Unreal::toFName(key_name));

                for (int i = 0; i < input_component->GetNumActionBindings(); i++) {
                    FInputActionBinding& action_binding = input_component->GetActionBinding(i);

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INPUT_SERVICE.PRINT_INJECT_DEBUG_INFO")) {
                        SP_LOG(Unreal::toStdString(action_binding.GetActionName()));
                        SP_LOG(Unreal::getStringFromEnumValue(action_binding.KeyEvent.GetValue()));
                    }

                    if (action_name == Unreal::toStdString(action_binding.GetActionName()) && key_event == action_binding.KeyEvent.GetValue()) {
                        action_binding.ActionDelegate.Execute(key);
                    }
                }
            });
    }

private:
    UInputComponent* getInputComponent(uint64_t& actor)
    {
        AActor* actor_ptr = toPtr<AActor>(actor);
        std::vector<UInputComponent*> input_components = Unreal::getComponentsByType<UInputComponent>(actor_ptr);
        if (input_components.size() != 1) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    Couldn't find a unique UInputComponent on actor ", Unreal::tryGetStableName(actor_ptr), ", giving up...");
            return nullptr;
        } else {
            return input_components.at(0);
        }
    }
};
