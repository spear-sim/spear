//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpDebugManager.h"

#include <stdint.h> // uint8_t, uint64_t

#include <map>
#include <memory> // std::make_unique
#include <ranges> // std::views::all, std::views::transform
#include <vector>

#include <Components/ActorComponent.h>
#include <Components/PoseableMeshComponent.h>
#include <Components/SkinnedMeshComponent.h> // EBoneSpaces
#include <Components/StaticMeshComponent.h>
#include <Containers/Array.h>
#include <Containers/Map.h>
#include <Containers/UnrealString.h>         // FString
#include <Engine/StaticMeshActor.h>
#include <Engine/Engine.h>                   // GEngine
#include <Engine/EngineTypes.h>              // EEndPlayReason
#include <Engine/World.h>                    // FActorSpawnParameters
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <PhysicsEngine/BodyInstance.h>
#include <UObject/Class.h>                   // UClass, UScriptStruct
#include <UObject/Object.h>                  // UObject
#include <UObject/ObjectMacros.h>            // EPropertyFlags
#include <UObject/Script.h>                  // EFunctionFlags
#include <UObject/UnrealType.h>              // FProperty

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"
#include "SpCore/UnrealObj.h"
#include "SpCore/Yaml.h"
#include "SpCore/YamlCpp.h"

#include "SpUnrealTypes/SpActorHitManager.h"
#include "SpUnrealTypes/SpSceneCaptureComponent2D.h"

// CALLED for the CDO
// CALLED for a newly added editor-world object, when adding the object to a map
// CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing play (SpFuncComponent is set to nullptr initially, and then is set to a new USpFuncComponent* not owned by any other actor)
ASpDebugManager::ASpDebugManager()
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent = UnrealUtils::createSceneComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);
}

// CALLED for the CDO
// CALLED for a newly removed editor-world object, when removing the object from the map (but only called when the map is unloaded)
// CALLED for an existing editor-world object, when unloading the map
// CALLED for an existing PIE-world object, when pressing stop
ASpDebugManager::~ASpDebugManager()
{
    SP_LOG_CURRENT_FUNCTION();
}

// CALLED for the CDO
// CALLED for a newly added editor-world object, when adding the object to a map
// CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing play (SpFuncComponent is set to the USpFuncComponent* owned by the CDO in this function)
void ASpDebugManager::PostInitProperties()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::PostInitProperties();
}

// NOT CALLED for the CDO
// CALLED for a newly added editor-world object, when adding the object to a map
// NOT CALLED for an existing editor-world object, when loading the map
// NOT CALLED for an existing PIE-world object, when pressing play
void ASpDebugManager::PostActorCreated()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::PostActorCreated();
}

// NOT CALLED for the CDO
// NOT CALLED for a newly added editor-world object, when adding the object to a map
// CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing play (SpFuncComponent is set to the new USpFuncComponent* from the constructor in this function)
void ASpDebugManager::PostLoad()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::PostLoad();
}

// NOT CALLED for the CDO
// NOT CALLED for a newly added editor-world object, when adding the object to a map
// NOT CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing play (SpFuncComponent is set to the new USpFuncComponent* from the constructor in this function)
void ASpDebugManager::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::BeginPlay();
}

// NOT CALLED for the CDO
// NOT CALLED for a newly added editor-world object, when adding the object to a map
// NOT CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing stop (SpFuncComponent is set to the new USpFuncComponent* from the constructor in this function)
void ASpDebugManager::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::EndPlay(end_play_reason);
}

// CALLED for the CDO
// CALLED for a newly removed editor-world object, when removing the object from the map (but only called when the map is unloaded)
// CALLED for an existing editor-world object, when unloading the map
// CALLED for an existing PIE-world object, when pressing stop (SpFuncComponent is set to the new USpFuncComponent* from the constructor in this function)
void ASpDebugManager::BeginDestroy()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::BeginDestroy();
}

void ASpDebugManager::Initialize()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!shared_memory_region_) {
        SP_ASSERT(SpFuncComponent);

        uint64_t shared_memory_num_bytes = 1024;
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(shared_memory_num_bytes);
        SP_ASSERT(shared_memory_region_);

        // The name chosen here for our shared memory region does not need to be globally unique. It only needs
        // unique within this SpFuncComponent.
        shared_memory_view_ = SpArraySharedMemoryView(shared_memory_region_->getView(), "smem:sp_debug_manager", SpArraySharedMemoryUsageFlags::ReturnValue);
        SpFuncComponent->registerSharedMemoryView(shared_memory_view_);

        SpFuncComponent->registerFunc("hello_world", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {

            // define arg objects
            SpArrayView<double> action("action");
            SpArrayView<double> action_shared("action_shared");
            UnrealObj<FVector> in_location("in_location");
            UnrealObj<FRotator> in_rotation("in_rotation");

            // initialize arg objects from the data bundle that was passed in
            SpArrayUtils::setViews({action.getPtr(), action_shared.getPtr()}, args.packed_arrays_);
            UnrealObjUtils::setObjectPropertiesFromStrings({in_location.getPtr(), in_rotation.getPtr()}, args.unreal_obj_strings_);

            SP_LOG("    action[0]:        ", Std::at(action.getView(), 0));
            SP_LOG("    action[1]:        ", Std::at(action.getView(), 1));
            SP_LOG("    action[2]:        ", Std::at(action.getView(), 2));
            SP_LOG("    action_shared[0]: ", Std::at(action_shared.getView(), 0));
            SP_LOG("    action_shared[1]: ", Std::at(action_shared.getView(), 1));
            SP_LOG("    action_shared[2]: ", Std::at(action_shared.getView(), 2));
            SP_LOG("    in_location:      ", in_location.getObj().X, " ", in_location.getObj().Y, " ", in_location.getObj().Z);
            SP_LOG("    in_rotation:      ", in_rotation.getObj().Pitch, " ", in_rotation.getObj().Yaw, " ", in_rotation.getObj().Roll);
            SP_LOG("    info:             ", args.info_);

            // define return value objects
            SpArray<double> observation("observation");
            SpArray<double> observation_shared("observation_shared");
            UnrealObj<FVector> out_location("out_location");
            UnrealObj<FRotator> out_rotation("out_rotation");

            // set return value objects
            observation.setDataSource({12.0, 13.0, 14.0});
            observation_shared.setDataSource(shared_memory_view_, {3});
            observation_shared.setDataValues({15.0, 16.0, 17.0});
            out_location.setObj(FVector(18.0, 19.0, 20.0));
            out_rotation.setObj(FRotator(21.0, 22.0, 23.0));
            std::string info = "Success";

            SP_LOG("    observation[0]:        ", Std::at(observation.getView(), 0));
            SP_LOG("    observation[1]:        ", Std::at(observation.getView(), 1));
            SP_LOG("    observation[2]:        ", Std::at(observation.getView(), 2));
            SP_LOG("    observation_shared[0]: ", Std::at(observation_shared.getView(), 0));
            SP_LOG("    observation_shared[1]: ", Std::at(observation_shared.getView(), 1));
            SP_LOG("    observation_shared[2]: ", Std::at(observation_shared.getView(), 2));
            SP_LOG("    out_location:          ", out_location.getObj().X, " ", out_location.getObj().Y, " ", out_location.getObj().Z);
            SP_LOG("    out_rotation:          ", out_rotation.getObj().Pitch, " ", out_rotation.getObj().Yaw, " ", out_rotation.getObj().Roll);
            SP_LOG("    info:                  ", info);

            // initialize output data bundle from return value objects
            SpFuncDataBundle return_values;
            return_values.packed_arrays_ = SpArrayUtils::moveToPackedArrays({observation.getPtr(), observation_shared.getPtr()});
            return_values.unreal_obj_strings_ = UnrealObjUtils::getObjectPropertiesAsStrings({out_location.getPtr(), out_rotation.getPtr()});
            return_values.info_ = info;

            return return_values;
        });
    }
}

void ASpDebugManager::Terminate()
{
    SP_LOG_CURRENT_FUNCTION();

    if (shared_memory_region_) {
        SpFuncComponent->unregisterFunc("hello_world");
        SpFuncComponent->unregisterSharedMemoryView(shared_memory_view_);

        shared_memory_region_ = nullptr;
    }
}

void ASpDebugManager::LoadConfig()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::LoadConfig();
}

void ASpDebugManager::SaveConfig()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::SaveConfig();
}

void ASpDebugManager::PrintDebugString() const
{
    SP_LOG_CURRENT_FUNCTION();
    SP_LOG("DebugString: ", Unreal::toStdString(DebugString));
}

void ASpDebugManager::GetAndSetObjectProperties()
{
    SP_LOG_CURRENT_FUNCTION();

    UWorld* world = GetWorld();
    SP_ASSERT(world);

    std::map<std::string, AStaticMeshActor*> static_mesh_actors = UnrealUtils::findActorsByNameAsMap<AStaticMeshActor>(world, "Debug/SM_Prop_04");
    SP_ASSERT(Std::containsKey(static_mesh_actors, "Debug/SM_Prop_04"));
    SP_ASSERT(static_mesh_actors.at("Debug/SM_Prop_04"));

    AStaticMeshActor* static_mesh_actor = UnrealUtils::findActorByName<AStaticMeshActor>(world, "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    // Get actor from class
    std::vector<AActor*> static_mesh_actors_from_class = UnrealUtils::findActorsByClass(AStaticMeshActor::StaticClass(), world);
    SP_ASSERT(!static_mesh_actors_from_class.empty());
    AActor* static_mesh_actor_from_class = static_mesh_actors_from_class.at(0);
    SP_LOG(Unreal::toStdString(static_mesh_actor_from_class->GetName()));

    // Get and set object properties from UObject*
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(static_mesh_actor));

    UnrealUtils::setObjectPropertiesFromString(static_mesh_actor, "{\"bHidden\": true}"); // partial updates are allowed
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(static_mesh_actor));

    UnrealUtils::setObjectPropertiesFromString(static_mesh_actor, "{\"bHidden\": false}");
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(static_mesh_actor));

    // Find properties by fully qualified name (pointers like RootComponent are handled correctly)
    SpPropertyDesc root_component_property_desc = UnrealUtils::findPropertyByName(static_mesh_actor, "RootComponent");
    SpPropertyDesc relative_location_property_desc = UnrealUtils::findPropertyByName(static_mesh_actor, "RootComponent.RelativeLocation");

    // Get property value from SpPropertyDesc
    SP_LOG(Std::toStringFromPtr(static_mesh_actor->GetStaticMeshComponent()));
    SP_LOG(UnrealUtils::getPropertyValueAsString(root_component_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_property_desc).value_);

    // Pointer properties can be set by converting the desired pointer value to a string
    UnrealUtils::setPropertyValueFromString(root_component_property_desc, Std::toStringFromPtr(static_mesh_actor->GetStaticMeshComponent()));
    SP_LOG(UnrealUtils::getPropertyValueAsString(root_component_property_desc).value_);

    UStaticMeshComponent* static_mesh_component = UnrealUtils::getComponentByType<UStaticMeshComponent>(static_mesh_actor);
    SP_ASSERT(static_mesh_component);
    SP_LOG(static_mesh_component);

    // Get component from class
    UActorComponent* actor_component = UnrealUtils::getComponentByClass(USceneComponent::StaticClass(), static_mesh_actor);
    SP_ASSERT(actor_component);
    SP_LOG(actor_component);

    USceneComponent* scene_component = UnrealUtils::getChildComponentByClass(USceneComponent::StaticClass(), static_mesh_actor);
    SP_ASSERT(scene_component);
    SP_LOG(scene_component);

    SpPropertyDesc relative_location_property_desc_  = UnrealUtils::findPropertyByName(static_mesh_component, "RelativeLocation");
    SpPropertyDesc relative_location_x_property_desc = UnrealUtils::findPropertyByName(static_mesh_component, "RelativeLocation.X");
    SpPropertyDesc relative_location_y_property_desc = UnrealUtils::findPropertyByName(static_mesh_component, "RelativeLocation.Y");
    SpPropertyDesc relative_location_z_property_desc = UnrealUtils::findPropertyByName(static_mesh_component, "RelativeLocation.Z");
    SpPropertyDesc body_instance_property_desc       = UnrealUtils::findPropertyByName(static_mesh_component, "BodyInstance");
    SpPropertyDesc com_nudge_property_desc           = UnrealUtils::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge");
    SpPropertyDesc com_nudge_x_property_desc         = UnrealUtils::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge.X");
    SpPropertyDesc com_nudge_y_property_desc         = UnrealUtils::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge.Y");
    SpPropertyDesc com_nudge_z_property_desc         = UnrealUtils::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge.Z");
    SpPropertyDesc com_nudge_property_desc_          = UnrealUtils::findPropertyByName(static_mesh_component, "bodyinstance.comnudge"); // not case-sensitive
    SpPropertyDesc simulate_physics_property_desc    = UnrealUtils::findPropertyByName(static_mesh_component, "BodyInstance.bSimulatePhysics");
    SpPropertyDesc component_velocity_property_desc  = UnrealUtils::findPropertyByName(static_mesh_component, "ComponentVelocity"); // defined in base class

    // Get property value from SpPropertyDesc
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_property_desc_).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_x_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_y_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_z_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(body_instance_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(com_nudge_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(com_nudge_x_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(com_nudge_y_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(com_nudge_z_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(com_nudge_property_desc_).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(simulate_physics_property_desc).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(component_velocity_property_desc).value_);

    void* value_ptr = nullptr;
    UStruct* ustruct = nullptr;

    // Get property values from void* and UStruct*
    value_ptr = &(static_mesh_component->BodyInstance);
    ustruct = FBodyInstance::StaticStruct(); // Unreal::getStaticStruct<FBodyInstance>(); // could also use FBodyInstance::StaticStruct()
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(value_ptr, ustruct));
    SP_LOG();

    // Get property values from void* and UStruct*
    value_ptr = relative_location_property_desc.value_ptr_;
    ustruct = Unreal::getStaticStruct<FVector>();
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(value_ptr, ustruct));
    SP_LOG();

    static int i = 1;
    std::string str;
    FVector vec(1.23, 4.56, 7.89);

    // Set property value from void* and UStruct*
    str = Std::toString("{", "\"x\": ", 12.3*i, ", \"y\": ", 45.6*i, "}");
    value_ptr = &vec;
    ustruct = Unreal::getStaticStruct<FVector>();
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(value_ptr, ustruct));
    UnrealUtils::setObjectPropertiesFromString(value_ptr, ustruct, str);
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(value_ptr, ustruct));
    SP_LOG();

    // Set property value from SpPropertyDesc
    str = Std::toString("{", "\"x\": ", 1.1*i, ", \"y\": ", 2.2*i, ", \"z\": ", 3.3*i, "}");
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_property_desc).value_);
    UnrealUtils::setPropertyValueFromString(relative_location_property_desc, str);
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_property_desc).value_);
    SP_LOG();

    // Set property value from SpPropertyDesc
    str = "1.2345";
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_z_property_desc).value_);
    UnrealUtils::setPropertyValueFromString(relative_location_z_property_desc, str);
    SP_LOG(UnrealUtils::getPropertyValueAsString(relative_location_z_property_desc).value_);
    SP_LOG();

    // String properties can be get and set using our Unreal API
    MyString = Unreal::toFString("HELLO WORLD!");

    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MyString")).value_);
    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "MyString"), "hello world!");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MyString")).value_);

    SP_LOG(UnrealUtils::getObjectPropertiesAsString(this));

    // Arrays can be indexed when searching for properties
    ArrayOfInts.Add(10);
    ArrayOfInts.Add(20);
    ArrayOfInts.Add(30);
    ArrayOfVectors.Add(FVector(1.0, 2.0, 3.0));
    ArrayOfVectors.Add(FVector(4.0, 5.0, 6.0));
    ArrayOfStrings.Add(Unreal::toFString("Hello"));
    ArrayOfPointers.Add(this);
    ArrayOfPointers.Add(nullptr);
    ArrayOfPointers.Add(this);
    ArrayOfEnums.Add(EDebugManagerEnum::Hello);
    ArrayOfEnums.Add(EDebugManagerEnum::World);
    ArrayOfEnums.Add(EDebugManagerEnum::Hello);
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(this));

    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfInts[1]")).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfVectors[1]")).value_);

    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "PrimaryActorTick.TickGroup"), "TG_PostPhysics");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "PrimaryActorTick.TickGroup")).value_);
    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "PrimaryActorTick.TickGroup"), "TG_PrePhysics");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "PrimaryActorTick.TickGroup")).value_);

    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfInts")).value_);
    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "ArrayOfInts"), "[9, 8, 7, 6, 5, 4, 3, 2, 1, 0]");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfInts")).value_);

    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfVectors")).value_);
    str = Std::toString("{", "\"x\": ", 12.3*i, ", \"y\": ", 45.6*i, ", \"z\": ", 78.9*i, "}");
    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "ArrayOfVectors"), "[ " + str + ", " + str + ", " + str + "]");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfVectors")).value_);

    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfPointers")).value_);
    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "ArrayOfPointers"), "[\"0x0\", \"" + Std::toStringFromPtr(static_mesh_actor->GetStaticMeshComponent()) + "\"]");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "ArrayOfPointers")).value_);

    MapFromIntToInt.Add(1, 2);
    MapFromIntToInt.Add(3, 4);
    MapFromIntToInt.Add(5, 6);
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(this));

    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MapFromIntToInt")).value_);
    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "MapFromIntToInt"), "{\"10\": 20, \"30\": 40, \"3\": 100}");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MapFromIntToInt")).value_);

    MapFromStringToVector.Add(Unreal::toFString("Hello"), 1.0*vec);
    MapFromStringToVector.Add(Unreal::toFString("World"), 2.0*vec);
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(this));

    // Maps can also be indexed when searching for properties
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MapFromIntToInt")).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MapFromStringToVector")).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MapFromIntToInt[3]")).value_);
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "MapFromStringToVector[\"World\"]")).value_);

    SetOfStrings.Add("Hello");
    SetOfStrings.Add("1");
    SetOfStrings.Add("World");
    SetOfStrings.Add("2");
    SP_LOG(UnrealUtils::getObjectPropertiesAsString(this));
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "SetOfStrings")).value_);

    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "SetOfStrings")).value_);
    UnrealUtils::setPropertyValueFromString(UnrealUtils::findPropertyByName(this, "SetOfStrings"), "[\"10\", \"Hello\", \"30\"]");
    SP_LOG(UnrealUtils::getPropertyValueAsString(UnrealUtils::findPropertyByName(this, "SetOfStrings")).value_);

    //
    // We need to do this do see visual updates in the editor. But this interface is not ideal because
    // it requires passing in a position and rotation delta, and it doesn't take the UPROPERTIES we set
    // previously into account. Therefore, we must update the position of objects by calling UFUNCTIONS
    // rather than setting UPROPERTIES.
    // 
    // static_mesh_component->MoveComponentImpl(FVector(5.0, 5.0, 5.0), FQuat::Identity, false);
    //

    i++;
}

void ASpDebugManager::CallFunctions()
{
    SP_LOG_CURRENT_FUNCTION();

    static int i = 1;

    UFunction* ufunction = nullptr;
    std::map<std::string, std::string> args;
    std::map<std::string, SpPropertyValue> return_values;
    std::string vec_str = Std::toString("{", "\"x\": ", 1.1*i, ", \"y\": ", 2.2*i, ", \"z\": ", 3.3*i, "}");

    args = {{"Arg0", "Hello World"}, {"Arg1", "true"}, {"Arg2", "12345"}, {"Arg3", vec_str}};
    ufunction = UnrealUtils::findFunctionByName(this->GetClass(), "GetString");
    SP_ASSERT(ufunction);
    return_values = UnrealUtils::callFunction(GetWorld(), this, ufunction, args);
    SP_LOG(return_values.at("ReturnValue").value_);

    args = {{"Arg0", "Hello World"}, {"Arg1", "true"}, {"Arg2", "12345"}, {"Arg3", vec_str}};
    ufunction = UnrealUtils::findFunctionByName(this->GetClass(), "GetVector");
    SP_ASSERT(ufunction);
    return_values = UnrealUtils::callFunction(GetWorld(), this, ufunction, args);
    SP_LOG(return_values.at("Arg3").value_); // Arg3 is modified by GetVector(...)
    SP_LOG(return_values.at("ReturnValue").value_);

    // Pointers can be passed into functions by converting them to strings, and static functions can be
    // called by passing in the class' default object when calling callFunction(...).
    args = {{"Arg0", "Hello World"}, {"Arg1", "true"}};
    ufunction = UnrealUtils::findFunctionByName(this->GetClass(), "GetWorldContextObject");
    SP_ASSERT(ufunction);
    return_values = UnrealUtils::callFunction(GetWorld(), this->GetClass()->GetDefaultObject(), ufunction, args);
    SP_LOG(return_values.at("ReturnValue").value_);

    args = {};
    ufunction = UnrealUtils::findFunctionByName(this->GetClass(), "UpdateData");
    SP_ASSERT(ufunction);
    return_values = UnrealUtils::callFunction(GetWorld(), this->GetClass()->GetDefaultObject(), ufunction, args);
    SP_LOG(return_values.at("InMapFromStringToVector").value_);
    SP_LOG(return_values.at("InArrayOfVectors").value_);

    UWorld* world = GetWorld(); 
    SP_ASSERT(world);

    AStaticMeshActor* static_mesh_actor = UnrealUtils::findActorByName<AStaticMeshActor>(world, "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    UStaticMeshComponent* static_mesh_component = UnrealUtils::getComponentByType<UStaticMeshComponent>(static_mesh_actor);
    SP_ASSERT(static_mesh_component);

    //
    // Since partial updates are allowed throughout our setObjectPropertiesFromString(...) and
    // setPropertyValueFromString(...) interfaces, we follow the same convention in our callFunction(...)
    // interface. For each object that gets passed to a given target function, we attempt to initialize
    // it as follows. First, we set its entire memory region to 0, then we (partially) update it
    // according to its corresponding string in the args std::map provided by the caller. This approach
    // enables a caller to explicitly initialize a subset of data members when passing an object to a
    // target function, but always allows the caller to fully initialize the object if necessary. We also
    // allow entire arguments to be omitted from args, in which case the default-initialized (as
    // described above) object will be passed to the target function without further modification.
    // 
    // The signature for the target function below is as follows,
    //
    //     void SceneComponent::K2_AddRelativeLocation(
    //         FVector DeltaLocation, bool bSweep, FHitResult& SweepHitResult, bool bTeleport);
    // 
    // where SweepHitResult is an "out" parameter used to return additional data to the caller. Since
    // SweepHitResult will be filled in by the target function anyway, we simply omit it from args.
    // Regardless of whether or not it is included in args, SweepHitResult (and all other arguments), are
    // always included in the values returned by callFunction(...). So the caller can always access any
    // additional data that was filled in by the target function, as well as the target function's formal
    // return value.
    //

    args = {{"DeltaLocation", vec_str}, {"bSweep", "false"}, {"bTeleport", "false"}};
    ufunction = UnrealUtils::findFunctionByName(static_mesh_component->GetClass(), "K2_AddRelativeLocation");
    SP_ASSERT(ufunction);
    return_values = UnrealUtils::callFunction(GetWorld(), static_mesh_component, ufunction, args);
    SP_LOG(return_values.at("SweepHitResult").value_);

    UObject* uobject = ASpActorHitManager::StaticClass()->GetDefaultObject();
    SP_ASSERT(uobject);
    ufunction = UnrealUtils::findFunctionByName(uobject->GetClass(), "GetActorHitDescs");
    SP_ASSERT(ufunction);
    return_values = UnrealUtils::callFunction(GetWorld(), uobject, ufunction);
    SP_LOG(return_values.at("ReturnValue").value_);

    i++;
}

void ASpDebugManager::CallSpFunc() const
{
    SP_LOG_CURRENT_FUNCTION();

    if (shared_memory_region_) {
        USpFuncComponent* sp_func_component = UnrealUtils::getComponentByType<USpFuncComponent>(this);
        SP_ASSERT(sp_func_component);

        // define arg objects
        SpArray<double> action("action");
        SpArray<double> action_shared("action_shared");
        UnrealObj<FVector> in_location("in_location");
        UnrealObj<FRotator> in_rotation("in_rotation");

        // set arg objects
        action.setDataSource({0.0, 1.0, 2.0});
        action_shared.setDataSource(shared_memory_view_, {3});
        action_shared.setDataValues({3.0, 4.0, 5.0});
        in_location.setObj(FVector(6.0, 7.0, 8.0));
        in_rotation.setObj(FRotator(9.0, 10.0, 11.0));
        std::string info = "Hello world";

        SP_LOG("    action[0]:        ", Std::at(action.getView(), 0));
        SP_LOG("    action[1]:        ", Std::at(action.getView(), 1));
        SP_LOG("    action[2]:        ", Std::at(action.getView(), 2));
        SP_LOG("    action_shared[0]: ", Std::at(action_shared.getView(), 0));
        SP_LOG("    action_shared[1]: ", Std::at(action_shared.getView(), 1));
        SP_LOG("    action_shared[2]: ", Std::at(action_shared.getView(), 2));
        SP_LOG("    in_location:      ", in_location.getObj().X, " ", in_location.getObj().Y, " ", in_location.getObj().Z);
        SP_LOG("    in_rotation:      ", in_rotation.getObj().Pitch, " ", in_rotation.getObj().Yaw, " ", in_rotation.getObj().Roll);
        SP_LOG("    info:             ", info);

        // initialize data bundle from arg objects
        SpFuncDataBundle args;
        args.packed_arrays_ = SpArrayUtils::moveToPackedArrays({action.getPtr(), action_shared.getPtr()});
        args.unreal_obj_strings_ = UnrealObjUtils::getObjectPropertiesAsStrings({in_location.getPtr(), in_rotation.getPtr()});
        args.info_ = info;

        // call function
        SpFuncDataBundle return_values = sp_func_component->callFunc("hello_world", args);

        // define return value objects
        SpArrayView<double> observation("observation");
        SpArrayView<double> observation_shared("observation_shared");
        UnrealObj<FVector> out_location("out_location");
        UnrealObj<FRotator> out_rotation("out_rotation");

        // initialize return value objects from data bundle
        SpArrayUtils::setViews({observation.getPtr(), observation_shared.getPtr()}, return_values.packed_arrays_);
        UnrealObjUtils::setObjectPropertiesFromStrings({out_location.getPtr(), out_rotation.getPtr()}, return_values.unreal_obj_strings_);

        SP_LOG("    observation[0]:        ", Std::at(observation.getView(), 0));
        SP_LOG("    observation[1]:        ", Std::at(observation.getView(), 1));
        SP_LOG("    observation[2]:        ", Std::at(observation.getView(), 2));
        SP_LOG("    observation_shared[0]: ", Std::at(observation_shared.getView(), 0));
        SP_LOG("    observation_shared[1]: ", Std::at(observation_shared.getView(), 1));
        SP_LOG("    observation_shared[2]: ", Std::at(observation_shared.getView(), 2));
        SP_LOG("    out_location:          ", out_location.getObj().X, " ", out_location.getObj().Y, " ", out_location.getObj().Z);
        SP_LOG("    out_rotation:          ", out_rotation.getObj().Pitch, " ", out_rotation.getObj().Yaw, " ", out_rotation.getObj().Roll);
        SP_LOG("    info:                  ", return_values.info_);
    }
}

void ASpDebugManager::CreateObjects() const
{
    static int i = 1;

    std::string in_vec_string = Std::toString("{", "\"x\": ", 1.1*i, ", \"y\": ", 2.2*i, ", \"z\": ", 3.3*i, "}");
    std::map<std::string, std::string> in_named_arg_strings = {{"location", in_vec_string}, {"rotation", "{}"}};

    UnrealObj<FVector> location("location");
    UnrealObj<FRotator> rotation("rotation");
    std::vector<UnrealObjBase*> named_args = {location.getPtr(), rotation.getPtr()};

    // get object properties as strings for all of the UnrealObj objects in named_args
    std::map<std::string, std::string> out_named_arg_strings = UnrealObjUtils::getObjectPropertiesAsStrings(named_args);
    for (auto& [name, arg_string] : out_named_arg_strings) {
        SP_LOG(name);
        SP_LOG(arg_string);
    }

    // set object properties for all of the UnrealObj objects in named_args
    UnrealObjUtils::setObjectPropertiesFromStrings(named_args, in_named_arg_strings);

    // verify the UnrealObj objects have been updated
    out_named_arg_strings = UnrealObjUtils::getObjectPropertiesAsStrings(named_args);
    for (auto& [name, arg_string] : out_named_arg_strings) {
        SP_LOG(name);
        SP_LOG(arg_string);
    }

    SP_ASSERT(GetWorld());
    FActorSpawnParameters spawn_parameters;
    AActor* actor = GetWorld()->SpawnActor<AStaticMeshActor>(location.getObj(), rotation.getObj(), spawn_parameters);
    SP_ASSERT(actor);

    i++;
}

void ASpDebugManager::SubscribeToActorHitEvents() const
{
    SP_LOG_CURRENT_FUNCTION();

    AStaticMeshActor* static_mesh_actor = UnrealUtils::findActorByName<AStaticMeshActor>(GetWorld(), "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    UObject* sp_actor_hit_manager = ASpActorHitManager::StaticClass()->GetDefaultObject();
    SP_ASSERT(sp_actor_hit_manager);

    UFunction* ufunction = UnrealUtils::findFunctionByName(sp_actor_hit_manager->GetClass(), "SubscribeToActor");
    UnrealUtils::callFunction(GetWorld(), sp_actor_hit_manager, ufunction, {{"Actor", Std::toStringFromPtr(static_mesh_actor)}});
}

void ASpDebugManager::UnsubscribeFromActorHitEvents() const
{
    SP_LOG_CURRENT_FUNCTION();

    AStaticMeshActor* static_mesh_actor = UnrealUtils::findActorByName<AStaticMeshActor>(GetWorld(), "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    UObject* sp_actor_hit_manager = ASpActorHitManager::StaticClass()->GetDefaultObject();
    SP_ASSERT(sp_actor_hit_manager);

    UFunction* ufunction = UnrealUtils::findFunctionByName(sp_actor_hit_manager->GetClass(), "UnsubscribeFromActor");
    UnrealUtils::callFunction(GetWorld(), sp_actor_hit_manager, ufunction, {{"Actor", Std::toStringFromPtr(static_mesh_actor)}});
}

void ASpDebugManager::ReadPixels() const
{
    SP_LOG_CURRENT_FUNCTION();

    std::vector<AActor*> actors = UnrealUtils::findActorsByName<AActor>(GetWorld(), {"Debug/BP_Camera_Sensor"});
    AActor* actor = actors.at(0);
    if (!actor) {
        SP_LOG("    Couldn't find Debug/BP_Camera_Sensor, giving up...");
        return;
    }

    USpSceneCaptureComponent2D* sp_scene_capture_component_2d = UnrealUtils::getComponentByName<USpSceneCaptureComponent2D>(actor, "DefaultSceneRoot.final_tone_curve_hdr_");
    SP_ASSERT(sp_scene_capture_component_2d);
    if (!sp_scene_capture_component_2d->IsInitialized()) {
        SP_LOG("    Debug/BP_Camera_Sensor:DefaultSceneRoot.final_tone_curve_hdr isn't initialized, giving up...");
        return;
    }

    USpFuncComponent* sp_func_component = UnrealUtils::getComponentByName<USpFuncComponent>(actor, "DefaultSceneRoot.final_tone_curve_hdr_.sp_func_component");
    SP_ASSERT(sp_func_component);

    SpFuncDataBundle args;

    // call SpFunc
    SpArrayUtils::validate(args.packed_arrays_, SpArraySharedMemoryUsageFlags::Arg);
    SpFuncDataBundle return_values = sp_func_component->callFunc("read_pixels", args);
    SpArrayUtils::validate(return_values.packed_arrays_, SpArraySharedMemoryUsageFlags::ReturnValue);

    SP_LOG("    return_values.packed_arrays_.at(\"data\").data_:                      ", Std::toStringFromPtr(return_values.packed_arrays_.at("data").data_.data()));
    SP_LOG("    return_values.packed_arrays_.at(\"data\").view_:                      ", Std::toStringFromPtr(return_values.packed_arrays_.at("data").view_));
    SP_LOG("    return_values.packed_arrays_.at(\"data\").data_source_:               ", Unreal::getEnumValue(return_values.packed_arrays_.at("data").data_source_));
    SP_LOG("    return_values.packed_arrays_.at(\"data\").data_type_:                 ", Unreal::getEnumValue(return_values.packed_arrays_.at("data").data_type_));
    SP_LOG("    return_values.packed_arrays_.at(\"data\").shared_memory_name_:        ", return_values.packed_arrays_.at("data").shared_memory_name_);
    SP_LOG("    return_values.packed_arrays_.at(\"data\").shared_memory_usage_flags_: ", Unreal::getEnumValue(return_values.packed_arrays_.at("data").shared_memory_usage_flags_));

    for (int i = 0; i < return_values.packed_arrays_.at("data").shape_.size(); i++) {
        SP_LOG("    return_values.packed_arrays_.at(\"data\").shape_:                     ", return_values.packed_arrays_.at("data").shape_.at(i));
    }

    void* view_ptr = return_values.packed_arrays_.at("data").view_;

    SP_LOG("    view_ptr:    ", Std::toStringFromPtr(view_ptr));
    SP_LOG("    view_ptr[0]: ", (int)(((uint8_t*)view_ptr)[0]));
    SP_LOG("    view_ptr[1]: ", (int)(((uint8_t*)view_ptr)[1]));
    SP_LOG("    view_ptr[2]: ", (int)(((uint8_t*)view_ptr)[2]));
    SP_LOG("    view_ptr[3]: ", (int)(((uint8_t*)view_ptr)[3]));
}

void ASpDebugManager::PrintLevelDebugInfo() const
{
    SP_LOG_CURRENT_FUNCTION();

    UWorld* world = GetWorld();
    SP_ASSERT(world);

    SP_LOG("    Actors: ");
    std::map<std::string, AActor*> actors = UnrealUtils::findActorsAsMap(world);
    for (auto& [name, actor] : actors) {
        SP_LOG("        ", name);
    }
}

void ASpDebugManager::PrintActorDebugInfo() const
{
    SP_LOG_CURRENT_FUNCTION();

    AStaticMeshActor* static_mesh_actor = UnrealUtils::findActorByName<AStaticMeshActor>(GetWorld(), "Debug/SM_Prop_04");
    SP_LOG("    Printing debug info for actor: ", UnrealUtils::getStableName(static_mesh_actor));

    SP_LOG("    Non-scene components: ");
    std::map<std::string, UActorComponent*> components = UnrealUtils::getComponentsAsMap(static_mesh_actor);
    for (auto& [name, component] : components) {
        if (!component->IsA(USceneComponent::StaticClass())) {
            SP_LOG("        ", name, " (", Unreal::getTypeAsString(component->GetClass()), + ")");
        }
    }

    SP_LOG("    Scene components: ");
    USceneComponent* root_component = static_mesh_actor->GetRootComponent();
    if (root_component) {
        SP_LOG("        ", UnrealUtils::getStableName(root_component), " (", Unreal::getTypeAsString(root_component->GetClass()) + ")");
        std::map<std::string, USceneComponent*> scene_components = UnrealUtils::getChildrenComponentsAsMap(root_component);
        for (auto& [name, scene_component] : scene_components) {
            SP_LOG("        ", name, " (", Unreal::getTypeAsString(scene_component->GetClass()) + ")");
        }
    }

    UClass* uclass = static_mesh_actor->GetClass();
    SP_ASSERT(uclass);

    SP_LOG("    Type: ", Unreal::getTypeAsString(uclass));
    SP_LOG("    Meta type: ", Unreal::getTypeAsString(uclass->GetClass()));

    SP_LOG("    Type hierarchy: ");
    for (UClass* current_uclass = uclass; current_uclass; current_uclass = current_uclass->GetSuperClass()) {
        SP_LOG("        ", Unreal::getTypeAsString(current_uclass));
    }

    SP_LOG("    Functions: ");
    for (UClass* current_uclass = uclass; current_uclass; current_uclass = current_uclass->GetSuperClass()) {
        SP_LOG("        Functions for type: ", Unreal::getTypeAsString(current_uclass));
        std::map<std::string, UFunction*> ufunctions = UnrealUtils::findFunctionsAsMap(current_uclass, EFieldIterationFlags::IncludeDeprecated); // exclude base classes
        for (auto& [ufunction_name, ufunction] : ufunctions) {
            SP_LOG("            Function: ", ufunction_name);
            std::map<std::string, FProperty*> properties = UnrealUtils::findPropertiesAsMap(ufunction);
            for (auto& [property_name, property] : properties) {
                SP_ASSERT(property->HasAnyPropertyFlags(EPropertyFlags::CPF_Parm));
                if (!property->HasAnyPropertyFlags(EPropertyFlags::CPF_ReturnParm)) {
                    SP_LOG("                Argument: ", property_name, " (", Unreal::getCppTypeAsString(property), ")");
                }
            }
            for (auto& [property_name, property] : properties) {
                SP_ASSERT(property->HasAnyPropertyFlags(EPropertyFlags::CPF_Parm));
                if (property->HasAnyPropertyFlags(EPropertyFlags::CPF_ReturnParm)) {
                    SP_LOG("                Return value: ", property_name, " (", Unreal::getCppTypeAsString(property), ")");
                }
            }
        }
    }

    SP_LOG("    Properties: ");
    for (UClass* current_uclass = uclass; current_uclass; current_uclass = current_uclass->GetSuperClass()) {
        SP_LOG("        Properties for type: ", Unreal::getTypeAsString(current_uclass));
        std::map<std::string, FProperty*> properties = UnrealUtils::findPropertiesAsMap(current_uclass, EFieldIterationFlags::IncludeDeprecated); // exclude base classes
        for (auto& [name, property] : properties) {
            SP_LOG("            Property: ", name, " (", Unreal::getCppTypeAsString(property), ")");
        }
    }
}

void ASpDebugManager::PrintAllClassesDebugInfo() const
{
    SP_LOG_CURRENT_FUNCTION();

    UClass* uobject_static_class = nullptr;
    int num_functions = -1;
    int num_properties = -1;
    int total_num_functions = -1;
    int total_num_properties = -1;
    std::map<std::string, UClass*> uclasses;
    std::map<std::string, UScriptStruct*> ustructs;

    SP_LOG("Counting the number of functions and properties exposed to the Unreal Engine reflection system...");

    total_num_functions = 0;
    total_num_properties = 0;
    uclasses = {};

    uobject_static_class = UObject::StaticClass();
    SP_LOG("    Type: ", Unreal::getTypeAsString(uobject_static_class));
    SP_LOG("    Meta type: ", Unreal::getTypeAsString(uobject_static_class->GetClass()));

    num_functions = UnrealUtils::findFunctions(uobject_static_class, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
    num_properties = UnrealUtils::findProperties(uobject_static_class, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
    SP_LOG("        Class: UObject (", num_functions, " functions, ", num_properties, " properties)");
    total_num_functions += num_functions;
    total_num_properties += num_properties;

    uclasses = UnrealUtils::getDerivedClassesAsMap(uobject_static_class);
    SP_LOG("    Number of classes that derive from UObject: ", uclasses.size());

    for (auto& [uclass_name, uclass] : uclasses) {
        num_functions = UnrealUtils::findFunctions(uclass, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
        num_properties = UnrealUtils::findProperties(uclass, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
        SP_LOG("        Class: ", uclass_name, " (", num_functions, " functions, ", num_properties, " properties)");
        total_num_functions += num_functions;
        total_num_properties += num_properties;
    }

    ustructs = UnrealUtils::findStaticStructsByTypeAsMap<UScriptStruct>();
    SP_LOG("    Number of structs that are outside the UObject class hierarchy: ", ustructs.size());

    for (auto& [ustruct_name, ustruct] : ustructs) {
        num_properties = UnrealUtils::findProperties(ustruct, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
        SP_LOG("        Struct: ", ustruct_name, " (", num_properties, " properties)");
        total_num_properties += num_properties;
    }

    SP_LOG("    Total function count: ", total_num_functions);
    SP_LOG("    Total property count: ", total_num_properties);

    SP_LOG("Counting the number of functions and properties exposed to Unreal's editor-only Python library...");

    total_num_functions = 0;
    total_num_properties = 0;
    uclasses = {};

    uobject_static_class = UObject::StaticClass();
    SP_LOG("    Type: ", Unreal::getTypeAsString(uobject_static_class));
    SP_LOG("    Meta type: ", Unreal::getTypeAsString(uobject_static_class->GetClass()));

    EFunctionFlags function_flags = EFunctionFlags::FUNC_BlueprintCallable | EFunctionFlags::FUNC_BlueprintPure;
    EPropertyFlags property_flags = EPropertyFlags::CPF_BlueprintVisible | EPropertyFlags::CPF_BlueprintReadOnly | EPropertyFlags::CPF_BlueprintAssignable | EPropertyFlags::CPF_Edit;

    num_functions = UnrealUtils::findFunctionsByFlagsAny(uobject_static_class, function_flags, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
    num_properties = UnrealUtils::findPropertiesByFlagsAny(uobject_static_class, property_flags, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
    SP_LOG("        Class: UObject (", num_functions, " functions, ", num_properties, " properties)");
    total_num_functions += num_functions;
    total_num_properties += num_properties;

    uclasses = UnrealUtils::getDerivedClassesAsMap(uobject_static_class);
    SP_LOG("    Number of classes that derive from UObject: ", uclasses.size());

    for (auto& [uclass_name, uclass] : uclasses) {
        num_functions = UnrealUtils::findFunctionsByFlagsAny(uclass, function_flags, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
        num_properties = UnrealUtils::findPropertiesByFlagsAny(uclass, property_flags, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
        // SP_LOG("        Class: ", uclass_name, " (", num_functions, " functions, ", num_properties, " properties)");
        total_num_functions += num_functions;
        total_num_properties += num_properties;
    }

    ustructs = UnrealUtils::findStaticStructsByTypeAsMap<UScriptStruct>();
    SP_LOG("    Number of structs that are outside the UObject class hierarchy: ", ustructs.size());

    for (auto& [ustruct_name, ustruct] : ustructs) {
        num_properties = UnrealUtils::findPropertiesByFlagsAny(ustruct, property_flags, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
        // SP_LOG("        Struct: ", ustruct_name, " (", num_properties, " properties)");
        total_num_properties += num_properties;
    }

    SP_LOG("    Total function count: ", total_num_functions);
    SP_LOG("    Total property count: ", total_num_properties);

    SP_LOG("Counting the number of functions available through the AActor class...");

    total_num_functions = 0;
    uclasses = {};

    uobject_static_class = AActor::StaticClass();
    SP_LOG("    Type: ", Unreal::getTypeAsString(uobject_static_class));
    SP_LOG("    Meta type: ", Unreal::getTypeAsString(uobject_static_class->GetClass()));

    num_functions = UnrealUtils::findFunctions(uobject_static_class, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
    SP_LOG("        Class: AActor (", num_functions, " functions)");
    total_num_functions += num_functions;

    uclasses = UnrealUtils::getDerivedClassesAsMap(uobject_static_class);
    SP_LOG("    Number of classes that derive from AActor: ", uclasses.size());

    for (auto& [uclass_name, uclass] : uclasses) {
        num_functions = UnrealUtils::findFunctions(uclass, EFieldIterationFlags::IncludeDeprecated).size(); // exclude base classes
        SP_LOG("        Class: ", uclass_name, " (", num_functions, " functions, ", num_properties, " properties)");
        total_num_functions += num_functions;
    }

    SP_LOG("    Total function count: ", total_num_functions);
}

void ASpDebugManager::TestReinterpretAsVectorOf() const
{
    SP_LOG_CURRENT_FUNCTION();

    std::vector<uint8_t, SpAlignedAllocator<uint8_t, 32>> src;
    SP_LOG("src.size():     ", src.size());
    SP_LOG("src.capacity(): ", src.capacity());
    SP_LOG("src:");
    for (auto s : src) {
        SP_LOG(static_cast<int>(s));
    }
    SP_LOG();

    Std::resizeUninitialized(src, 8);

    SP_LOG("src.size():     ", src.size());
    SP_LOG("src.capacity(): ", src.capacity());
    SP_LOG("src:");
    for (auto s : src) {
        SP_LOG(static_cast<int>(s));
    }
    SP_LOG();

    src = {1, 0, 0, 0, 2, 0, 0, 0};
    src.reserve(12);

    SP_LOG("src.size():     ", src.size());
    SP_LOG("src.capacity(): ", src.capacity());
    SP_LOG("src:");
    for (auto s : src) {
        SP_LOG(static_cast<int>(s));
    }
    SP_LOG();

    // this fast variant requires dest to have the same allocator type (SpAlignedAllocator<..., 32>) as src
    std::vector<int, SpAlignedAllocator<int, 32>> src_as_int = Std::reinterpretAsVectorOf<int>(std::move(src));

    SP_LOG("src_as_int.size():     ", src_as_int.size());
    SP_LOG("src_as_int.capacity(): ", src_as_int.capacity());
    SP_LOG("src_as_int:");
    for (auto s : src_as_int) {
        SP_LOG(static_cast<int>(s));
    }
    SP_LOG();

    // in this case, we need to specify an allocator because otherwise std::vector<uint8_t> might be misaligned for storing int
    std::vector<uint8_t, SpAlignedAllocator<uint8_t, 32>> src_as_int_as_uint8 = Std::reinterpretAsVector<uint8_t, SpAlignedAllocator<uint8_t, 32>, int>(std::views::all(src_as_int));

    SP_LOG("src_as_int_as_uint8.size():     ", src_as_int_as_uint8.size());
    SP_LOG("src_as_int_as_uint8.capacity(): ", src_as_int_as_uint8.capacity());
    SP_LOG("src_as_int_as_uint8:");
    for (auto s : src_as_int_as_uint8) {
        SP_LOG(static_cast<int>(s));
    }
    SP_LOG();

    // in this case, we do not need to specify an allocator because std::vector<int> will always be correctly aligned for storing uint8_t
    std::vector<int> src_as_int_as_uint8_as_int = Std::reinterpretAsVector<int, uint8_t>(std::views::all(src_as_int_as_uint8));

    SP_LOG("src_as_int_as_uint8_as_int.size():     ", src_as_int_as_uint8_as_int.size());
    SP_LOG("src_as_int_as_uint8_as_int.capacity(): ", src_as_int_as_uint8_as_int.capacity());
    SP_LOG("src_as_int_as_uint8_as_int:");
    for (auto s : src_as_int_as_uint8_as_int) {
        SP_LOG(static_cast<int>(s));
    }
    SP_LOG();
}

FString ASpDebugManager::GetString(FString Arg0, bool Arg1, int Arg2, FVector Arg3) const
{
    SP_LOG_CURRENT_FUNCTION();
    return FString("    GetString return value.");
}

FVector ASpDebugManager::GetVector(FString Arg0, bool Arg1, int Arg2, FVector& Arg3) const
{
    SP_LOG_CURRENT_FUNCTION();
    Arg3 = FVector(1.11, 2.22, 3.33);
    return FVector(9.87, 6.54, 3.21);
}

UObject* ASpDebugManager::GetWorldContextObject(const UObject* WorldContextObject, FString Arg0, bool Arg1)
{
    SP_LOG_CURRENT_FUNCTION();
    return const_cast<UObject*>(WorldContextObject);
}

void ASpDebugManager::UpdateData(TMap<FString, FVector>& InMapFromStringToVector, TArray<FVector>& InArrayOfVectors)
{
    SP_LOG_CURRENT_FUNCTION();
    FVector vec(1.23, 4.56, 7.89);
    InMapFromStringToVector.Add(Unreal::toFString("Hello"), 1.0*vec);
    InMapFromStringToVector.Add(Unreal::toFString("World"), 2.0*vec);
    InArrayOfVectors.Add(FVector(1.0, 2.0, 3.0));
    InArrayOfVectors.Add(FVector(4.0, 5.0, 6.0));
}
