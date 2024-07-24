//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpDebugWidget.h"

#include <stdint.h> // uint8_t

#include <memory> // std::make_unique
#include <ranges> // std::views::transform

#include <Components/PoseableMeshComponent.h>
#include <Components/SkinnedMeshComponent.h> // EBoneSpaces::Type
#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>                    // FActorSpawnParameters
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <PhysicsEngine/BodyInstance.h>
#include <UObject/Object.h>

#include "SpCore/Assert.h"
#include "SpCore/CppFunc.h"
#include "SpCore/CppFuncComponent.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealObj.h"
#include "SpCore/UnrealClassRegistrar.h"
#include "SpCore/Yaml.h"
#include "SpCore/YamlCpp.h"

#include "SpComponents/SpHitEventActor.h"

ASpDebugWidget::ASpDebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();

    CppFuncComponent = Unreal::createComponentInsideOwnerConstructor<UCppFuncComponent>(this, "cpp_func");
    SP_ASSERT(CppFuncComponent);

    initializeCppFuncs();
}

ASpDebugWidget::~ASpDebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpDebugWidget::BeginDestroy()
{
    AActor::BeginDestroy();
    terminateCppFuncs();
}

void ASpDebugWidget::LoadConfig()
{
    AActor::LoadConfig();
}

void ASpDebugWidget::SaveConfig()
{
    AActor::SaveConfig();
}

void ASpDebugWidget::PrintDebugString()
{
    SP_LOG("DebugString: ", Unreal::toStdString(DebugString));
}

void ASpDebugWidget::GetAndSetObjectProperties()
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
    bool assert_if_not_found = true;
    bool assert_if_multiple_found = false;
    AActor* static_mesh_actor_from_class = Unreal::findActorByClass(world, AStaticMeshActor::StaticClass(), assert_if_not_found, assert_if_multiple_found);
    SP_ASSERT(static_mesh_actor_from_class);
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
    ArrayOfVectors.Add(FVector(1.0f, 2.0f, 3.0f));
    ArrayOfVectors.Add(FVector(4.0f, 5.0f, 6.0f));
    ArrayOfStrings.Add(Unreal::toFString("Hello"));
    SP_LOG(Unreal::getObjectPropertiesAsString(this));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfInts[1]")));
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfVectors[1]")));

    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup"), "TG_PostPhysics");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup")));
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup"), "TG_PrePhysics");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "PrimaryActorTick.TickGroup")));

    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfInts")));
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfVectors")));

    str = Std::toString("{", "\"x\": ", 12.3 * i, ", \"y\": ", 45.6 * i, ", \"z\": ", 78.9 * i, "}");
    Unreal::setPropertyValueFromString(Unreal::findPropertyByName(this, "ArrayOfVectors"), "[ " + str + ", " + str + ", " + str + "]");
    SP_LOG(Unreal::getPropertyValueAsString(Unreal::findPropertyByName(this, "ArrayOfVectors")));

    MapFromIntToInt.Add(1, 2);
    MapFromIntToInt.Add(3, 4);
    MapFromIntToInt.Add(5, 6);
    SP_LOG(Unreal::getObjectPropertiesAsString(this));

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

void ASpDebugWidget::CallFunctions()
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

    UObject* uobject = Unreal::findActorByName(world, "SpComponents/SpHitEventActor");
    SP_ASSERT(uobject);
    ufunction = Unreal::findFunctionByName(uobject->GetClass(), "GetHitEventDescs");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(GetWorld(), uobject, ufunction);
    SP_LOG(return_values.at("ReturnValue"));

    i++;
}

void ASpDebugWidget::CallCppFunctions()
{
    static int i = 1;

    UCppFuncComponent* cpp_func_component = Unreal::getComponentByType<UCppFuncComponent>(this);
    SP_ASSERT(cpp_func_component);

    CppFuncPackage args;

    // create new data objects
    CppFuncData<double> location("location");
    CppFuncData<double> rotation("rotation");
    location.setData({1.0, 2.0, 4.0});
    rotation.setData({3.0, 5.0, 7.0});

    // create new Unreal objects
    UnrealObj<FVector> vec("vec");
    UnrealObj<FRotator> rot("rot");
    vec.setObj(FVector(10.0, 20.0, 30.0));
    rot.setObj(FRotator(30.0, 50.0, 70.0));

    CppFuncData<double> shared_doubles("shared_doubles");
    shared_doubles.setData("my_shared_memory", shared_memory_region_->getView(), 3);
    shared_doubles.setValues({99.0, 98.0, 97.0});

    // set arg items from data objects
    args.items_ = CppFuncUtils::moveDataToItems({location.getPtr(), rotation.getPtr(), shared_doubles.getPtr()});

    // set arg strings from Unreal objects
    args.unreal_obj_strings_ = UnrealObjUtils::getObjectPropertiesAsStrings({vec.getPtr(), rot.getPtr()});

    // set arg info string as a YAML string
    args.info_ = "INITIALIZE_DATA_MODE: rvalue_reference_to_vector_of_uint8";

    // call function
    CppFuncPackage return_values = cpp_func_component->callFunc("my_func", args);

    // create view objects directly from return data
    CppFuncView<double> location_as_return_value("location");
    CppFuncView<double> new_location("new_location");
    CppFuncView<double> new_rotation("new_rotation");
    CppFuncView<double> new_shared_doubles("new_shared_doubles");
    CppFuncUtils::setViewsFromItems({location_as_return_value.getPtr(), new_location.getPtr(), new_rotation.getPtr(), new_shared_doubles.getPtr()}, return_values.items_);

    // create Unreal objects directly from return strings
    UnrealObj<FVector> new_vec("new_vec");
    UnrealObj<FRotator> new_rot("new_rot");
    UnrealObjUtils::setObjectPropertiesFromStrings({new_vec.getPtr(), new_rot.getPtr()}, return_values.unreal_obj_strings_);

    // get return info string as a YAML value
    bool success = Yaml::get<bool>(YAML::Load(return_values.info_), "SUCCESS");
    SP_ASSERT(success);

    i++;
}

void ASpDebugWidget::CreateObjects()
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

void ASpDebugWidget::SubscribeToActorHitEvents()
{
    AStaticMeshActor* static_mesh_actor = Unreal::findActorByName<AStaticMeshActor>(GetWorld(), "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    ASpHitEventActor* hit_event_actor = Unreal::findActorByName<ASpHitEventActor>(GetWorld(), "SpComponents/SpHitEventActor");
    SP_ASSERT(hit_event_actor);

    UFunction* ufunction = Unreal::findFunctionByName(hit_event_actor->GetClass(), "SubscribeToActorHitEvents");
    Unreal::callFunction(GetWorld(), hit_event_actor, ufunction, {{"Actor", Std::toStringFromPtr(static_mesh_actor)}});
}

FString ASpDebugWidget::GetString(FString arg_0, bool arg_1, int arg_2, FVector arg_3)
{
    SP_LOG_CURRENT_FUNCTION();
    return FString("GetString return value.");
}

FVector ASpDebugWidget::GetVector(FString arg_0, bool arg_1, int arg_2, FVector& arg_3)
{
    SP_LOG_CURRENT_FUNCTION();
    arg_3 = FVector(1.11, 2.22, 3.33);
    return FVector(9.87, 6.54, 3.21);
}

UObject* ASpDebugWidget::GetWorldContextObject(const UObject* world_context_object, FString arg_0, bool arg_1)
{
    SP_LOG_CURRENT_FUNCTION();
    return const_cast<UObject*>(world_context_object);
}

void ASpDebugWidget::UpdateData(TMap<FString, FVector>& map_from_string_to_vector, TArray<FVector>& array_of_vectors)
{
    SP_LOG_CURRENT_FUNCTION();
    FVector vec(1.23, 4.56, 7.89);
    map_from_string_to_vector.Add(Unreal::toFString("Hello"), 1.0*vec);
    map_from_string_to_vector.Add(Unreal::toFString("World"), 2.0*vec);
    array_of_vectors.Add(FVector(1.0f, 2.0f, 3.0f));
    array_of_vectors.Add(FVector(4.0f, 5.0f, 6.0f));
}

void ASpDebugWidget::GetPoseableMeshPoses()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor* actor = Unreal::findActorByName(GetWorld(), "Debug/Manny");
    SP_ASSERT(actor);

    UPoseableMeshComponent* poseable_mesh_component = Unreal::getComponentByType<UPoseableMeshComponent>(actor);
    SP_ASSERT(poseable_mesh_component);

    int num_bones = poseable_mesh_component->GetNumBones();
    for (int i = 0; i < num_bones; i++) {        
        FName bone_name = poseable_mesh_component->GetBoneName(i);
        FTransform transform = poseable_mesh_component->GetBoneTransformByName(bone_name, EBoneSpaces::Type::ComponentSpace);
        SP_LOG(i);
        SP_LOG(Unreal::toStdString(bone_name));

        void* value_ptr = &transform;
        UStruct* ustruct = UnrealClassRegistrar::getStaticStruct<FTransform>();
        SP_LOG(Unreal::getObjectPropertiesAsString(value_ptr, ustruct));
        SP_LOG();
    }
}

void ASpDebugWidget::SetPoseableMeshPoses()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor* actor = Unreal::findActorByName(GetWorld(), "Debug/Manny");
    SP_ASSERT(actor);

    UPoseableMeshComponent* poseable_mesh_component = Unreal::getComponentByType<UPoseableMeshComponent>(actor);
    SP_ASSERT(poseable_mesh_component);

    std::vector<std::string> bone_names = {"hand_l", "hand_r", "head"};
    for (auto& bone_name : bone_names) {
        FTransform transform;
        UStruct* ustruct = UnrealClassRegistrar::getStaticStruct<FTransform>();
        void* value_ptr = &transform;

        transform = poseable_mesh_component->GetBoneTransformByName(Unreal::toFName(bone_name), EBoneSpaces::Type::ComponentSpace);
        SP_LOG(Unreal::getObjectPropertiesAsString(value_ptr, ustruct));

        FVector scale = transform.GetScale3D();
        transform.SetScale3D(scale*PoseableMeshScaleFactor);        
        poseable_mesh_component->SetBoneTransformByName(Unreal::toFName(bone_name), transform, EBoneSpaces::Type::ComponentSpace);

        transform = poseable_mesh_component->GetBoneTransformByName(Unreal::toFName(bone_name), EBoneSpaces::Type::ComponentSpace);
        SP_LOG(Unreal::getObjectPropertiesAsString(value_ptr, ustruct));
    }

    // poseable_mesh_component->RefreshBoneTransforms();
}

void ASpDebugWidget::initializeCppFuncs()
{
    int shared_memory_num_bytes = 1024;
    shared_memory_region_ = std::make_unique<SharedMemoryRegion>(shared_memory_num_bytes);
    SP_ASSERT(shared_memory_region_);

    std::string shared_memory_name = "my_shared_memory";
    CppFuncSharedMemoryView shared_memory_view(shared_memory_region_->getView(), CppFuncSharedMemoryUsageFlags::Arg | CppFuncSharedMemoryUsageFlags::ReturnValue);
    CppFuncComponent->registerSharedMemoryView(shared_memory_name, shared_memory_view);

    CppFuncComponent->registerFunc("hello_world", [this](CppFuncPackage& args) -> CppFuncPackage {

        // CppFuncData objects are {named, strongly typed} arrays that can be efficiently passed
        // {to, from} Python.
        CppFuncData<uint8_t> hello("hello");
        hello.setData("Hello World!");

        // Get some some data from the Unreal Engine.
        AActor* actor = Unreal::findActorByName(GetWorld(), "SpComponents/SpHitEventActor");
        FVector location = actor->GetActorLocation();

        // Store the Unreal data in a CppFuncData object to efficiently return it to Python.
        // Here we use shared memory for the most efficient possible communication.
        CppFuncData<double> unreal_data("unreal_data");
        unreal_data.setData("my_shared_memory", shared_memory_region_->getView(), 3);
        unreal_data.setValues({location.X, location.Y, location.Z});

        // Return CppFuncData objects.
        CppFuncPackage return_values;
        return_values.items_ = CppFuncUtils::moveDataToItems({hello.getPtr(), unreal_data.getPtr()});
        return return_values;
    });

    CppFuncComponent->registerFunc("my_func_1", [this](CppFuncPackage& args) -> CppFuncPackage {

        // create view objects directly from arg data
        CppFuncView<double> location("location");
        CppFuncView<double> rotation("rotation");
        CppFuncView<double> shared_doubles("shared_doubles");
        CppFuncUtils::setViewsFromItems({location.getPtr(), rotation.getPtr(), shared_doubles.getPtr()}, args.items_);

        // create Unreal objects directly from arg strings
        UnrealObj<FVector> vec("vec");
        UnrealObj<FRotator> rot("rot");
        UnrealObjUtils::setObjectPropertiesFromStrings({vec.getPtr(), rot.getPtr()}, args.unreal_obj_strings_);

        // get arg info string as a YAML value
        std::string initialize_data_mode = Yaml::get<std::string>(YAML::Load(args.info_), "INITIALIZE_DATA_MODE");

        // create new data objects
        CppFuncData<double> hello("hello");
        if (initialize_data_mode == "initializer_list_of_double") {
            hello.setData({16.0, 32.0}); // initialize from initializer list of double
        } else if (initialize_data_mode == "vector_of_double") {
            std::vector<double> hello_vector = {64.0, 128.0};
            hello.setData(hello_vector); // initialize from vector of double
        } else if (initialize_data_mode == "rvalue_reference_to_vector_of_uint8") {
            std::vector<uint8_t> hello_vector_uint8 = {1, 2, 3, 4, 5, 6, 7, 8};
            hello.setData(std::move(hello_vector_uint8)); // initialize from rvalue reference to vector of uint8_t (avoids copy)
        } else {
            SP_ASSERT(false);
        }

        CppFuncData<uint8_t> hello_str("hello_str");
        hello_str.setData("Hello world!"); // initialize from string literal

        CppFuncData<double> new_location("new_location");
        CppFuncData<double> new_rotation("new_rotation");
        CppFuncData<double> new_shared_doubles("new_shared_doubles");
        new_location.setData(location.getView() | std::views::transform([](auto x) { return 2.0*x; })); // initialize from range of double
        new_rotation.setData(rotation.getView() | std::views::transform([](auto x) { return 3.0*x; })); // initialize from range of double
        new_shared_doubles.setData("my_shared_memory", shared_memory_region_->getView(), 3);
        new_shared_doubles.setValues(shared_doubles.getView() | std::views::transform([](auto x) { return 4.0*x; }));

        // create new data objects by moving arg data
        CppFuncData<double> location_as_return_value("location");
        CppFuncUtils::moveItemsToData({location_as_return_value.getPtr()}, args.items_);

        // create new Unreal objects
        UnrealObj<FVector> new_vec("new_vec");
        UnrealObj<FRotator> new_rot("new_rot");
        new_vec.setObj(4.0*vec.getObj());
        new_rot.setObj(5.0*rot.getObj());

        CppFuncPackage return_values;

        // set return data from data objects
        return_values.items_ = CppFuncUtils::moveDataToItems({new_location.getPtr(), new_rotation.getPtr(), new_shared_doubles.getPtr(), location_as_return_value.getPtr()});

        // set return strings from Unreal objects
        return_values.unreal_obj_strings_ = UnrealObjUtils::getObjectPropertiesAsStrings({new_vec.getPtr(), new_rot.getPtr()});

        // set return info string as a YAML string
        return_values.info_ = "SUCCESS: True";

        return return_values;
    });

    CppFuncComponent->registerFunc("my_func_2", [this](CppFuncPackage& args) -> CppFuncPackage {

        // create view objects directly from arg data
        CppFuncView<double> location("location");
        CppFuncView<double> rotation("rotation");
        CppFuncUtils::setViewsFromItems({location.getPtr(), rotation.getPtr()}, args.items_);

        // create new data objects
        CppFuncData<double> new_location("new_location");
        CppFuncData<double> new_rotation("new_rotation");
        new_location.setData(location.getView() | std::views::transform([](auto x) { return 2.0*x; })); // initialize from range of double
        new_rotation.setData(rotation.getView() | std::views::transform([](auto x) { return 3.0*x; })); // initialize from range of double

        // create new data objects backed by shared memory
        CppFuncData<float> my_floats("my_floats");
        my_floats.setData("my_shared_memory", shared_memory_region_->getView(), 3);
        my_floats.setValues({99.0, 98.0, 97.0});

        CppFuncPackage return_values;
        return_values.items_ = CppFuncUtils::moveDataToItems({new_location.getPtr(), new_rotation.getPtr(), my_floats.getPtr()});

        return return_values;
    });
}

void ASpDebugWidget::terminateCppFuncs()
{
    CppFuncComponent->unregisterFunc("my_func_2");
    CppFuncComponent->unregisterFunc("my_func_1");
    CppFuncComponent->unregisterFunc("hello_world");

    CppFuncComponent->unregisterSharedMemoryView("my_shared_memory");
    shared_memory_region_ = nullptr;
}
