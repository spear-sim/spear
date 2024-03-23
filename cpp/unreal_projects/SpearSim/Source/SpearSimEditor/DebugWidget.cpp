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

    // Get property value from PropertyDesc.
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

    // Get property value from void* pointer and StaticStruct().
    void* body_instance = &(static_mesh_component->BodyInstance);
    SP_LOG(Unreal::getPropertyValueAsString(body_instance, FBodyInstance::StaticStruct()));
    SP_LOG();

    // Get property value from void* pointer and findStaticStructByName(...), useful for when a class
    // doesn't define a StaticStruct() method, e.g., FVector.
    AEngineActor* engine_actor = Unreal::findActorByType<AEngineActor>(world);
    SP_ASSERT(engine_actor);
    void* v1 = relative_location_property_desc.value_ptr_;
    SP_LOG(Unreal::getPropertyValueAsString(v1, engine_actor->findStaticStructByName("FVector")));
    SP_LOG();

    static int i = 0;
    std::string str;

    // Set property value from void* and findStaticStructByName(...), useful for when a class doesn't
    // define a StaticStruct() method, e.g., FVector.
    FVector v2(1.23, 4.56, 7.89);
    str = Std::toString("{", "\"x\": ", 12.3*i, ", \"y\": ", 45.6*i, "}");
    SP_LOG(Unreal::getPropertyValueAsString(&v2, engine_actor->findStaticStructByName("FVector")));
    Unreal::setPropertyValueFromString(&v2, engine_actor->findStaticStructByName("FVector"), str);
    SP_LOG(Unreal::getPropertyValueAsString(&v2, engine_actor->findStaticStructByName("FVector")));
    SP_LOG();

    // Set property value from PropertyDesc.
    str = Std::toString("{", "\"x\": ", 1.1*i, ", \"y\": ", 2.2*i, ", \"z\": ", 3.3*i, "}");
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_property_desc));
    Unreal::setPropertyValueFromString(relative_location_property_desc, str);
    SP_LOG(Unreal::getPropertyValueAsString(relative_location_property_desc));
    SP_LOG();

    // Set property value from PropertyDesc.
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
