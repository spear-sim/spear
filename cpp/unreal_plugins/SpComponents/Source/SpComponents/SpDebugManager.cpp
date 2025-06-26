//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpDebugManager.h"

#include <stdint.h> // uint8_t

#include <memory> // std::make_unique
#include <ranges> // std::views::transform

#include <Components/ActorComponent.h>
#include <Components/PoseableMeshComponent.h>
#include <Components/SkinnedMeshComponent.h> // EBoneSpaces
#include <Components/StaticMeshComponent.h>
#include <Containers/Array.h>
#include <Containers/Map.h>
#include <Containers/UnrealString.h>         // FString
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>                    // FActorSpawnParameters
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <PhysicsEngine/BodyInstance.h>
#include <UObject/Class.h>                   // UClass
#include <UObject/Object.h>                  // UObject

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/SpFuncDataBundle.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealObj.h"
#include "SpCore/UnrealClassRegistrar.h"
#include "SpCore/Yaml.h"
#include "SpCore/YamlCpp.h"

#include "SpComponents/SpHitEventManager.h"
#include "SpComponents/SpSceneCaptureComponent2D.h"

// CALLED for the CDO
// CALLED for a newly added editor-world object, when adding the object to a map
// CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing play (SpFuncComponent is set to nullptr initially, and then is set to a new USpFuncComponent* not owned by any other actor)
ASpDebugManager::ASpDebugManager()
{
    SP_LOG_CURRENT_FUNCTION();

    SP_LOG("    this:            ", this);
    SP_LOG("    SpFuncComponent: ", SpFuncComponent);

    SpFuncComponent = Unreal::createSceneComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);

    SP_LOG("    this:            ", this);
    SP_LOG("    SpFuncComponent: ", SpFuncComponent);

    if (HasAnyFlags(RF_ClassDefaultObject)) {
        SP_LOG("    HasAnyFlags(RF_ClassDefaultObject)");
        initializeSpFunc();
    }
}

// CALLED for an existing PIE-world object, when pressing stop
// CALLED for an existing editor-world object, when unloading the map
// CALLED for a newly removed editor-world object, when removing the object from the map (but only called when the map is unloaded)
// CALLED for the CDO
ASpDebugManager::~ASpDebugManager()
{
    SP_LOG_CURRENT_FUNCTION();

    SP_LOG("    this:            ", this);
    SP_LOG("    SpFuncComponent: ", SpFuncComponent);
}

// CALLED for the CDO
// CALLED for a newly added editor-world object, when adding the object to a map
// CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing play (SpFuncComponent is set to the USpFuncComponent* owned by the CDO in this function)
void ASpDebugManager::PostInitProperties()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::PostInitProperties();

    SP_LOG("    this:            ", this);
    SP_LOG("    SpFuncComponent: ", SpFuncComponent);
}

// NOT CALLED for the CDO
// CALLED for a newly added editor-world object, when adding the object to a map
// NOT CALLED for an existing editor-world object, when loading the map
// NOT CALLED for an existing PIE-world object, when pressing play
void ASpDebugManager::PostActorCreated()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::PostActorCreated();

    SP_LOG("    this:            ", this);
    SP_LOG("    SpFuncComponent: ", SpFuncComponent);

    initializeSpFunc();
}

// NOT CALLED for the CDO
// NOT CALLED for a newly added editor-world object, when adding the object to a map
// CALLED for an existing editor-world object, when loading the map
// CALLED for an existing PIE-world object, when pressing play (SpFuncComponent is set to the new USpFuncComponent* from the constructor in this function)
void ASpDebugManager::PostLoad()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::PostLoad();

    SP_LOG("    this:            ", this);
    SP_LOG("    SpFuncComponent: ", SpFuncComponent);

    initializeSpFunc();
}

// CALLED for an existing PIE-world object, when pressing stop
// CALLED for an existing editor-world object, when unloading the map
// CALLED for a newly removed editor-world object, when removing the object from the map (but only called when the map is unloaded)
// CALLED for the CDO
void ASpDebugManager::BeginDestroy()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::BeginDestroy();

    SP_LOG("    this:            ", this);
    SP_LOG("    SpFuncComponent: ", SpFuncComponent);

    terminateSpFunc();
}

void ASpDebugManager::LoadConfig()
{
    AActor::LoadConfig();
}

void ASpDebugManager::SaveConfig()
{
    AActor::SaveConfig();
}

void ASpDebugManager::PrintDebugString()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_LOG("    DebugString: ", Unreal::toStdString(DebugString));
}

void ASpDebugManager::GetAndSetObjectProperties()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    std::map<std::string, AStaticMeshActor*> static_mesh_actors = Unreal::findActorsByNameAsMap<AStaticMeshActor>(world, {"Debug/SM_Prop_04", "null"});
    SP_ASSERT(Std::containsKey(static_mesh_actors, "Debug/SM_Prop_04"));
    SP_ASSERT(Std::containsKey(static_mesh_actors, "null"));
    SP_ASSERT(static_mesh_actors.at("Debug/SM_Prop_04"));
    SP_ASSERT(!static_mesh_actors.at("null"));

    AStaticMeshActor* static_mesh_actor = Unreal::findActorByName<AStaticMeshActor>(world, "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    // Get actor from registrar
    AActor* static_mesh_actor_from_registrar = UnrealClassRegistrar::findActorByName("AStaticMeshActor", world, "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor_from_registrar);
    SP_LOG(static_mesh_actor_from_registrar);

    // Get actor from class
    std::vector<AActor*> static_mesh_actors_from_class = Unreal::findActorsByClass(world, AStaticMeshActor::StaticClass());
    SP_ASSERT(!static_mesh_actors_from_class.empty());
    AActor* static_mesh_actor_from_class = static_mesh_actors_from_class.at(0);
    SP_LOG(Unreal::toStdString(static_mesh_actor_from_class->GetName()));

    // Get and set object properties from UObject*
    SP_LOG(Unreal::getObjectPropertiesAsString(static_mesh_actor));

    Unreal::setObjectPropertiesFromString(static_mesh_actor, "{\"bHidden\": true }"); // partial updates are allowed
    SP_LOG(Unreal::getObjectPropertiesAsString(static_mesh_actor));

    Unreal::setObjectPropertiesFromString(static_mesh_actor, "{\"bHidden\": false }");
    SP_LOG(Unreal::getObjectPropertiesAsString(static_mesh_actor));

    // Find properties by fully qualified name (pointers like RootComponent are handled correctly)
    Unreal::PropertyDesc root_component_property_desc = Unreal::findPropertyByName(static_mesh_actor, "RootComponent");
    Unreal::PropertyDesc relative_location_property_desc = Unreal::findPropertyByName(static_mesh_actor, "RootComponent.RelativeLocation");

    // Get property value from PropertyDesc
    SP_LOG(Std::toStringFromPtr(static_mesh_actor->GetStaticMeshComponent()));
    SP_LOG(Unreal::getPropertyValueAsString(root_component_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_property_desc));

    // Pointer properties can be set by converting the desired pointer value to a string
    Unreal::setPropertyValueFromString(root_component_property_desc, Std::toStringFromPtr(static_mesh_actor->GetStaticMeshComponent()));
    SP_LOG(Unreal::getPropertyValueAsString(root_component_property_desc));

    UStaticMeshComponent* static_mesh_component = Unreal::getComponentByType<UStaticMeshComponent>(static_mesh_actor);
    SP_ASSERT(static_mesh_component);
    SP_LOG(static_mesh_component);

    // Get component from class
    UActorComponent* actor_component = Unreal::getComponentByClass(static_mesh_actor, USceneComponent::StaticClass());
    SP_ASSERT(actor_component);
    SP_LOG(actor_component);

    USceneComponent* scene_component = Unreal::getChildComponentByClass(static_mesh_actor, USceneComponent::StaticClass());
    SP_ASSERT(scene_component);
    SP_LOG(scene_component);

    // Get component from registrar
    UActorComponent* static_mesh_component_from_registrar = UnrealClassRegistrar::getComponentByType("UStaticMeshComponent", static_mesh_actor);
    SP_ASSERT(static_mesh_component_from_registrar);

    Unreal::PropertyDesc relative_location_property_desc_  = Unreal::findPropertyByName(static_mesh_component, "RelativeLocation");
    Unreal::PropertyDesc relative_location_x_property_desc = Unreal::findPropertyByName(static_mesh_component, "RelativeLocation.X");
    Unreal::PropertyDesc relative_location_y_property_desc = Unreal::findPropertyByName(static_mesh_component, "RelativeLocation.Y");
    Unreal::PropertyDesc relative_location_z_property_desc = Unreal::findPropertyByName(static_mesh_component, "RelativeLocation.Z");
    Unreal::PropertyDesc body_instance_property_desc       = Unreal::findPropertyByName(static_mesh_component, "BodyInstance");
    Unreal::PropertyDesc com_nudge_property_desc           = Unreal::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge");
    Unreal::PropertyDesc com_nudge_x_property_desc         = Unreal::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge.X");
    Unreal::PropertyDesc com_nudge_y_property_desc         = Unreal::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge.Y");
    Unreal::PropertyDesc com_nudge_z_property_desc         = Unreal::findPropertyByName(static_mesh_component, "BodyInstance.COMNudge.Z");
    Unreal::PropertyDesc com_nudge_property_desc_          = Unreal::findPropertyByName(static_mesh_component, "bodyinstance.comnudge"); // not case-sensitive
    Unreal::PropertyDesc simulate_physics_property_desc    = Unreal::findPropertyByName(static_mesh_component, "BodyInstance.bSimulatePhysics");
    Unreal::PropertyDesc component_velocity_property_desc  = Unreal::findPropertyByName(static_mesh_component, "ComponentVelocity"); // defined in base class

    // Get property value from PropertyDesc
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_property_desc_));
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_x_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_y_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_z_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(body_instance_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(com_nudge_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(com_nudge_x_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(com_nudge_y_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(com_nudge_z_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(com_nudge_property_desc_));
    SP_LOG(Unreal::getPropertyValueAsString(simulate_physics_property_desc));
    SP_LOG(Unreal::getPropertyValueAsString(component_velocity_property_desc));

    void* value_ptr = nullptr;
    UStruct* ustruct = nullptr;

    // Get property values from void* and UStruct*
    value_ptr = &(static_mesh_component->BodyInstance);
    ustruct = UnrealClassRegistrar::getStaticStruct<FBodyInstance>();
    SP_LOG(Unreal::getObjectPropertiesAsString(value_ptr, ustruct));
    SP_LOG();

    // Get property values from void* and UStruct*
    value_ptr = relative_location_property_desc.value_ptr_;
    ustruct = UnrealClassRegistrar::getStaticStruct<FVector>();
    SP_LOG(Unreal::getObjectPropertiesAsString(value_ptr, ustruct));
    SP_LOG();

    static int i = 1;
    std::string str;
    FVector vec(1.23, 4.56, 7.89);

    // Set property value from void* and UStruct*
    str = Std::toString("{", "\"x\": ", 12.3*i, ", \"y\": ", 45.6*i, "}");
    value_ptr = &vec;
    ustruct = UnrealClassRegistrar::getStaticStruct<FVector>();
    SP_LOG(Unreal::getObjectPropertiesAsString(value_ptr, ustruct));
    Unreal::setObjectPropertiesFromString(value_ptr, ustruct, str);
    SP_LOG(Unreal::getObjectPropertiesAsString(value_ptr, ustruct));
    SP_LOG();

    // Set property value from PropertyDesc
    str = Std::toString("{", "\"x\": ", 1.1*i, ", \"y\": ", 2.2*i, ", \"z\": ", 3.3*i, "}");
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_property_desc));
    Unreal::setPropertyValueFromString(relative_location_property_desc, str);
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_property_desc));
    SP_LOG();

    // Set property value from PropertyDesc
    str = "1.2345";
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_z_property_desc));
    Unreal::setPropertyValueFromString(relative_location_z_property_desc, str);
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_z_property_desc));
    SP_LOG();

    // String properties can be get and set using our Unreal API
    MyString = Unreal::toFString("HELLO WORLD!");

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MyString")));
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "MyString"), "hello world!");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MyString")));

    SP_LOG(Unreal::getObjectPropertiesAsString(this));

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
    SP_LOG(Unreal::getObjectPropertiesAsString(this));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfInts[1]")));
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfVectors[1]")));

    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup"), "TG_PostPhysics");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup")));
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup"), "TG_PrePhysics");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup")));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfInts")));
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "ArrayOfInts"), "[9, 8, 7, 6, 5, 4, 3, 2, 1, 0]");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfInts")));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfVectors")));
    str = Std::toString("{", "\"x\": ", 12.3*i, ", \"y\": ", 45.6*i, ", \"z\": ", 78.9*i, "}");
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "ArrayOfVectors"), "[ " + str + ", " + str + ", " + str + "]");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfVectors")));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfPointers")));
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "ArrayOfPointers"), "[\"0x0\", \"" + Std::toStringFromPtr(static_mesh_actor->GetStaticMeshComponent()) + "\"]");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfPointers")));

    MapFromIntToInt.Add(1, 2);
    MapFromIntToInt.Add(3, 4);
    MapFromIntToInt.Add(5, 6);
    SP_LOG(Unreal::getObjectPropertiesAsString(this));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MapFromIntToInt")));
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "MapFromIntToInt"), "{\"10\": 20, \"30\": 40, \"3\": 100}");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MapFromIntToInt")));

    MapFromStringToVector.Add(Unreal::toFString("Hello"), 1.0*vec);
    MapFromStringToVector.Add(Unreal::toFString("World"), 2.0*vec);
    SP_LOG(Unreal::getObjectPropertiesAsString(this));

    // Maps can also be indexed when searching for properties
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MapFromIntToInt")));
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MapFromStringToVector")));
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MapFromIntToInt[3]")));
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "MapFromStringToVector[\"World\"]")));

    SetOfStrings.Add("Hello");
    SetOfStrings.Add("1");
    SetOfStrings.Add("World");
    SetOfStrings.Add("2");
    SP_LOG(Unreal::getObjectPropertiesAsString(this));
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "SetOfStrings")));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "SetOfStrings")));
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "SetOfStrings"), "[\"10\", \"Hello\", \"30\"]");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "SetOfStrings")));

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
    static int i = 1;

    UFunction* ufunction = nullptr;
    std::map<std::string, std::string> args;
    std::map<std::string, std::string> return_values;
    std::string vec_str = Std::toString("{", "\"x\": ", 1.1*i, ", \"y\": ", 2.2*i, ", \"z\": ", 3.3*i, "}");

    args = {{"arg_0", "Hello World"}, {"arg_1", "true"}, {"arg_2", "12345"}, {"arg_3", vec_str}};
    ufunction = Unreal::findFunctionByName(this->GetClass(), "GetString");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(GetWorld(), this, ufunction, args);
    SP_LOG(return_values.at("ReturnValue"));

    args = {{"arg_0", "Hello World"}, {"arg_1", "true"}, {"arg_2", "12345"}, {"arg_3", vec_str}};
    ufunction = Unreal::findFunctionByName(this->GetClass(), "GetVector");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(GetWorld(), this, ufunction, args);
    SP_LOG(return_values.at("arg_3")); // arg_3 is modified by GetVector(...)
    SP_LOG(return_values.at("ReturnValue"));

    // Pointers can be passed into functions by converting them to strings, and static functions can be
    // called by passing in the class' default object when calling callFunction(...).
    args = {{"world_context_object", Std::toStringFromPtr(GetWorld())}, {"arg_0", "Hello World"}, {"arg_1", "true"}};
    ufunction = Unreal::findFunctionByName(this->GetClass(), "GetWorldContextObject");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(GetWorld(), this->GetClass()->GetDefaultObject(), ufunction, args);
    SP_LOG(return_values.at("ReturnValue"));

    args = {};
    ufunction = Unreal::findFunctionByName(this->GetClass(), "UpdateData");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(GetWorld(), this->GetClass()->GetDefaultObject(), ufunction, args);
    SP_LOG(return_values.at("map_from_string_to_vector"));
    SP_LOG(return_values.at("array_of_vectors"));

    UWorld* world = GetWorld(); 
    SP_ASSERT(world);

    AStaticMeshActor* static_mesh_actor = Unreal::findActorByName<AStaticMeshActor>(world, "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    UStaticMeshComponent* static_mesh_component = Unreal::getComponentByType<UStaticMeshComponent>(static_mesh_actor);
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
    ufunction = Unreal::findFunctionByName(static_mesh_component->GetClass(), "K2_AddRelativeLocation");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(GetWorld(), static_mesh_component, ufunction, args);
    SP_LOG(return_values.at("SweepHitResult"));

    UObject* uobject = ASpHitEventManager::StaticClass()->GetDefaultObject();
    SP_ASSERT(uobject);
    ufunction = Unreal::findFunctionByName(uobject->GetClass(), "GetHitEventDescs");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(GetWorld(), uobject, ufunction);
    SP_LOG(return_values.at("ReturnValue"));

    i++;
}

void ASpDebugManager::CallSpFunc()
{
    USpFuncComponent* sp_func_component = Unreal::getComponentByType<USpFuncComponent>(this);
    SP_ASSERT(sp_func_component);

    // define arg objects
    SpArray<double> action("action");
    SpArray<double> action_shared("action_shared");
    UnrealObj<FVector> in_location("in_location");
    UnrealObj<FRotator> in_rotation("in_rotation");

    // set arg objects
    action.setDataSource({0.0, 1.0, 2.0});
    action_shared.setDataSource(shared_memory_view_, {3}, "hello_shared_memory");
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

void ASpDebugManager::CreateObjects()
{
    static int i = 1;

    UClass* uclass = UnrealClassRegistrar::getStaticClass("UGameplayStatics");
    SP_ASSERT(uclass);

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

    FActorSpawnParameters spawn_parameters;
    AActor* actor = UnrealClassRegistrar::spawnActor("AStaticMeshActor", GetWorld(), location.getObj(), rotation.getObj(), spawn_parameters);
    SP_ASSERT(actor);

    i++;
}

void ASpDebugManager::SubscribeToActorHitEvents()
{
    SP_LOG_CURRENT_FUNCTION();

    AStaticMeshActor* static_mesh_actor = Unreal::findActorByName<AStaticMeshActor>(GetWorld(), "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    UObject* hit_event_manager = ASpHitEventManager::StaticClass()->GetDefaultObject();
    SP_ASSERT(hit_event_manager);

    UFunction* ufunction = Unreal::findFunctionByName(hit_event_manager->GetClass(), "SubscribeToActor");
    Unreal::callFunction(GetWorld(), hit_event_manager, ufunction, {{"Actor", Std::toStringFromPtr(static_mesh_actor)}, {"bRecordDebugInfo", "true"}});
}

void ASpDebugManager::ReadPixels()
{
    SP_LOG_CURRENT_FUNCTION();

    std::vector<AActor*> actors = Unreal::findActorsByName<AActor>(GetWorld(), {"Debug/BP_Camera_Sensor"});
    AActor* actor = actors.at(0);
    if (!actor) {
        SP_LOG("    Couldn't find Debug/BP_Camera_Sensor, giving up...");
        return;
    }

    USpSceneCaptureComponent2D* sp_scene_capture_component_2d = Unreal::getComponentByName<USpSceneCaptureComponent2D>(actor, "DefaultSceneRoot.final_tone_curve_hdr_");
    SP_ASSERT(sp_scene_capture_component_2d);
    if (!sp_scene_capture_component_2d->IsInitialized()) {
        SP_LOG("    Debug/BP_Camera_Sensor:DefaultSceneRoot.final_tone_curve_hdr isn't initialized, giving up...");
        return;
    }

    USpFuncComponent* sp_func_component = Unreal::getComponentByName<USpFuncComponent>(actor, "DefaultSceneRoot.final_tone_curve_hdr_.sp_func_component");
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

FString ASpDebugManager::GetString(FString arg_0, bool arg_1, int arg_2, FVector arg_3)
{
    SP_LOG_CURRENT_FUNCTION();
    return FString("    GetString return value.");
}

FVector ASpDebugManager::GetVector(FString arg_0, bool arg_1, int arg_2, FVector& arg_3)
{
    SP_LOG_CURRENT_FUNCTION();
    arg_3 = FVector(1.11, 2.22, 3.33);
    return FVector(9.87, 6.54, 3.21);
}

UObject* ASpDebugManager::GetWorldContextObject(const UObject* world_context_object, FString arg_0, bool arg_1)
{
    SP_LOG_CURRENT_FUNCTION();
    return const_cast<UObject*>(world_context_object);
}

void ASpDebugManager::UpdateData(TMap<FString, FVector>& map_from_string_to_vector, TArray<FVector>& array_of_vectors)
{
    SP_LOG_CURRENT_FUNCTION();
    FVector vec(1.23, 4.56, 7.89);
    map_from_string_to_vector.Add(Unreal::toFString("Hello"), 1.0*vec);
    map_from_string_to_vector.Add(Unreal::toFString("World"), 2.0*vec);
    array_of_vectors.Add(FVector(1.0, 2.0, 3.0));
    array_of_vectors.Add(FVector(4.0, 5.0, 6.0));
}

void ASpDebugManager::initializeSpFunc()
{
    SP_ASSERT(SpFuncComponent);
    SP_ASSERT(!shared_memory_region_);

    int shared_memory_num_bytes = 1024;
    shared_memory_region_ = std::make_unique<SharedMemoryRegion>(shared_memory_num_bytes);
    SP_ASSERT(shared_memory_region_);

    SpFuncComponent->initialize();

    // The name chosen here for our shared memory region does not need to be globally unique. It only needs
    // unique within this SpFuncComponent. Normally we would choose a human-readable name for the shared
    // memory, e.g., "smem:observation", but in this case, we set it to Std::toStringFromPtr(this) as a
    // debugging tool.
    shared_memory_view_ = SpArraySharedMemoryView(shared_memory_region_->getView(), SpArraySharedMemoryUsageFlags::ReturnValue);
    SpFuncComponent->registerSharedMemoryView(Std::toStringFromPtr(this), shared_memory_view_);

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
        observation_shared.setDataSource(shared_memory_view_, {3}, Std::toStringFromPtr(this));
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

void ASpDebugManager::terminateSpFunc()
{
    // Terminate SpFuncComponent conditionally because BeginDestroy() might be called after SpFuncComponent
    // has already been garbage collected and set to nullptr, e.g., if an editor-world object has been
    // removed but BeginDestroy() is only called when the map is unloaded.
    if (SpFuncComponent) {
        SpFuncComponent->unregisterFunc("hello_world");
        SpFuncComponent->unregisterSharedMemoryView(Std::toStringFromPtr(this));
        SpFuncComponent->terminate();
    }

    shared_memory_region_ = nullptr;
}
