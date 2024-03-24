//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSimEditor/DebugWidget.h"

#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMeshActor.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <PhysicsEngine/BodyInstance.h>

#include "SpCore/EngineActor.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "UrdfRobot/UrdfRobotPawn.h"
#include "Vehicle/VehiclePawn.h"

ADebugWidget::ADebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();
}

ADebugWidget::~ADebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();

    DebugString = Unreal::toFString("");
    UrdfFile = Unreal::toFString("");
}

void ADebugWidget::LoadConfig()
{
    AActor::LoadConfig();
}

void ADebugWidget::SaveConfig()
{
    AActor::SaveConfig();
}

void ADebugWidget::PrintDebugString()
{
    SP_LOG("DebugString: ", Unreal::toStdString(DebugString));
}

void ADebugWidget::SpawnVehiclePawn()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    std::vector<AVehiclePawn*> vehicle_pawns_list = Unreal::findActorsByType<AVehiclePawn>(world);
    std::string name_suffix = Std::toString(vehicle_pawns_list.size() + 1);
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName("vehicle_pawn_" + name_suffix);
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AVehiclePawn* vehicle_pawn = world->SpawnActor<AVehiclePawn>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(vehicle_pawn);
}

void ADebugWidget::SpawnUrdfRobotPawn()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    std::vector<AUrdfRobotPawn*> urdf_robot_pawns_list = Unreal::findActorsByType<AUrdfRobotPawn>(world);
    std::string name_suffix = Std::toString(urdf_robot_pawns_list.size() + 1);
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName("urdf_robot_pawn_" + name_suffix);
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AUrdfRobotPawn* urdf_robot_pawn = world->SpawnActor<AUrdfRobotPawn>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(urdf_robot_pawn);

    urdf_robot_pawn->UrdfFile = UrdfFile;
    urdf_robot_pawn->Initialize();
}

void ADebugWidget::SetObjectProperties()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    AStaticMeshActor* static_mesh_actor = Unreal::findActorByName<AStaticMeshActor>(world, "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    UStaticMeshComponent* static_mesh_component = Unreal::getComponentByType<UStaticMeshComponent>(static_mesh_actor);
    SP_ASSERT(static_mesh_component);

    Unreal::PropertyDesc relative_location_property_desc   = Unreal::findPropertyByName(static_mesh_component, "RelativeLocation");
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
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_property_desc));
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

    // Get property value from void* and UStruct*
    value_ptr = &(static_mesh_component->BodyInstance);
    ustruct = FBodyInstance::StaticStruct();
    SP_LOG(Unreal::getPropertyValueAsString(value_ptr, ustruct));
    SP_LOG();

    // Get property value from void* and UStruct*
    value_ptr = relative_location_property_desc.value_ptr_;
    ustruct = Unreal::findStructByName(world, "FVector"); // useful for when a class or struct doesn't define a StaticStruct() method
    SP_LOG(Unreal::getPropertyValueAsString(value_ptr, ustruct));
    SP_LOG();

    static int i = 0;
    std::string str;

    // Set property value from void* and UStruct*
    FVector vec(1.23, 4.56, 7.89);
    str = Std::toString("{", "\"x\": ", 12.3*i, ", \"y\": ", 45.6*i, "}"); // partial updates are allowed, only the members specified here are updated
    value_ptr = &vec;
    ustruct = Unreal::findStructByName(world, "FVector"); // useful for when a class or struct doesn't define a StaticStruct() method
    SP_LOG(Unreal::getPropertyValueAsString(value_ptr, ustruct));
    Unreal::setPropertyValueFromString(value_ptr, ustruct, str);
    SP_LOG(Unreal::getPropertyValueAsString(value_ptr, ustruct));
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

void ADebugWidget::CallFunctions()
{
    static int i = 0;

    UFunction* ufunction = nullptr;
    std::map<std::string, std::string> args;
    std::map<std::string, std::string> return_values;
    std::string vector_str = Std::toString("{", "\"x\": ", 1.1*i, ", \"y\": ", 2.2*i, ", \"z\": ", 3.3*i, "}");

    args = {{"arg_0", "\"Hello World\""}, {"arg_1", "true"}, {"arg_2", "12345"}, {"arg_3", vector_str}};
    ufunction = Unreal::findFunctionByName(this->GetClass(), "GetString");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(this, ufunction, args);
    SP_LOG(return_values.at("ReturnValue"));

    args = {{"arg_0", "\"Hello World\""}, {"arg_1", "true"}, {"arg_2", "12345"}, {"arg_3", vector_str}};
    ufunction = Unreal::findFunctionByName(this->GetClass(), "GetVector");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(this, ufunction, args);
    SP_LOG(return_values.at("arg_3")); // arg_3 is modified by GetVector(...)
    SP_LOG(return_values.at("ReturnValue"));

    UWorld* world = GetWorld();
    SP_ASSERT(world);

    AStaticMeshActor* static_mesh_actor = Unreal::findActorByName<AStaticMeshActor>(world, "Debug/SM_Prop_04");
    SP_ASSERT(static_mesh_actor);

    UStaticMeshComponent* static_mesh_component = Unreal::getComponentByType<UStaticMeshComponent>(static_mesh_actor);
    SP_ASSERT(static_mesh_component);

    //
    // Since partial updates are allowed throughout our setPropertyValueFromString(...) interface, we
    // follow the same convention in our callFunction(...) interface. For each object that gets passed to
    // a given target function, we attempt to initialize it as follows. First, we set its entire memory
    // region to 0, then we initialize it using a reasonable default string based on its type (e.g.,
    // "false" for a bool, "0" for an int, "{}" for a struct, etc), then we update it according to its
    // corresponding string in the args std::map provided by the caller. This approach enables a caller
    // to specify a sparse subset of data members that are relevant for a given function invocation, but
    // always allows the caller to fully initialize an object if desired. We also allow entire arguments
    // to be omitted from args, in which case the default-initialized object will be passed to the target
    // function without further modification.
    // 
    // The signature for the target function below is as follows,
    //
    //     void SceneComponent::K2_AddRelativeLocation(FVector DeltaLocation, bool bSweep, FHitResult& SweepHitResult, bool bTeleport)
    // 
    // where SweepHitResult is an "out" parameter used to return additional data to the caller. Since
    // SweepHitResult will be filled in by the target function anyway, we simply omit it from args.
    // Regardless of whether or not it is included in args, SweepHitResult (and all other arguments), are
    // always included in the values returned by callFunction(...). So the caller can always access data
    // that was filled in by the target function, as well as the target function's formal return value.
    //

    args = {{"DeltaLocation", vector_str}, {"bSweep", "false"}, {"bTeleport", "false"}};
    ufunction = Unreal::findFunctionByName(static_mesh_component->GetClass(), "K2_AddRelativeLocation");
    SP_ASSERT(ufunction);
    return_values = Unreal::callFunction(static_mesh_component, ufunction, args);
    SP_LOG(return_values.at("SweepHitResult"));

    i++;
}

FString ADebugWidget::GetString(FString arg_0, bool arg_1, int arg_2, FVector arg_3)
{
    SP_LOG_CURRENT_FUNCTION();
    return FString("GetString return value.");
}

FVector ADebugWidget::GetVector(FString arg_0, bool arg_1, int arg_2, FVector& arg_3)
{
    SP_LOG_CURRENT_FUNCTION();
    arg_3 = FVector(1.11, 2.22, 3.33);
    return FVector(9.87, 6.54, 3.21);
}
