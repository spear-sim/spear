//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfLinkComponent.h"

#include <map>
#include <string>
#include <vector>

#include <Components/StaticMeshComponent.h>
#include <Containers/Array.h>
#include <Engine/StaticMesh.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <Math/Rotator.h>
#include <Math/UnrealMathUtility.h> // FMath::RadiansToDegrees
#include <Math/Vector.h>
#include <PhysicsEngine/BodyInstance.h>
#include <PhysicalMaterials/PhysicalMaterial.h>

#include "SpCore/ArrayDesc.h"
#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"
#include "SpCore/Std.h"
#include "UrdfRobot/UrdfParser.h"

UUrdfLinkComponent::UUrdfLinkComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

UUrdfLinkComponent::~UUrdfLinkComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // Objects created with CreateDefaultSubobject, DuplicateObject, LoadObject, NewObject don't need to be cleaned up explicitly.

    StaticMeshComponents.Empty();
}

void UUrdfLinkComponent::initialize(const UrdfLinkDesc* link_desc)
{
    bool promote_to_children = true;
    for (auto static_mesh_component : StaticMeshComponents) {
        static_mesh_component->DestroyComponent(promote_to_children);
    }
    StaticMeshComponents.Empty();

    double m_to_cm = 100.0;

    // set our own reference frame using the top-level UrdfLinkDesc
    FVector location = FVector(link_desc->xyz_.at(0), link_desc->xyz_.at(1), link_desc->xyz_.at(2)) * m_to_cm; // m to cm
    FRotator rotation = FMath::RadiansToDegrees(FRotator({link_desc->rpy_.at(1), link_desc->rpy_.at(2), link_desc->rpy_.at(0)})); // rpy to pyr, rad to deg

    // assign location, rotation, scale, for the top-level static mesh
    auto link_static_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString("/UrdfRobot/Common/Meshes/SM_Dummy.SM_Dummy"));

    SetRelativeLocation(location);
    SetRelativeRotation(rotation);
    SetStaticMesh(link_static_mesh);

    // set physical properties
    SetNotifyRigidBodyCollision(true);
    SetSimulatePhysics(link_desc->simulate_physics_);
    bUseDefaultCollision = true;

    // inertial
    SetMassOverrideInKg(NAME_None, link_desc->inertial_desc_.mass_, true);

    // visual
    for (auto& visual_desc : link_desc->visual_descs_) {

        // needs to be const because link_desc is const
        const UrdfGeometryDesc& geometry_desc = visual_desc.geometry_desc_;

        // create child static mesh component for each UrdfSpearLinkDesc
        auto static_mesh_component = Unreal::createComponentOutsideOwnerConstructor<UStaticMeshComponent>(this, "static_mesh_component");
        SP_ASSERT(static_mesh_component);

        // each UrdfSpearLinkDesc has its own reference frame
        FVector static_mesh_location = FVector(
            visual_desc.xyz_.at(0), visual_desc.xyz_.at(1), visual_desc.xyz_.at(2)) * m_to_cm; // m to cm
        FRotator static_mesh_rotation = FMath::RadiansToDegrees(FRotator(
            visual_desc.rpy_.at(1), visual_desc.rpy_.at(2), visual_desc.rpy_.at(0))); // rpy to pyr, rad to deg
        FVector static_mesh_scale = FVector::OneVector;

        UStaticMesh* static_mesh = nullptr;
        switch (geometry_desc.type_) {
            case UrdfGeometryType::Box:
                static_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString("/Engine/BasicShapes/Cube.Cube"));
                static_mesh_scale = {geometry_desc.size_.at(0), geometry_desc.size_.at(1), geometry_desc.size_.at(2)};
                break;
            case UrdfGeometryType::Cylinder:
                static_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString("/Engine/BasicShapes/Cylinder.Cylinder"));
                static_mesh_scale = {geometry_desc.radius_ * 2.0, geometry_desc.radius_ * 2.0, geometry_desc.length_};
                break;
            case UrdfGeometryType::Sphere:
                static_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString("/Engine/BasicShapes/Sphere.Sphere"));
                static_mesh_scale = {geometry_desc.radius_ * 2.0, geometry_desc.radius_ * 2.0, geometry_desc.radius_ * 2.0};
                break;
            case UrdfGeometryType::Mesh:
                static_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString(geometry_desc.unreal_static_mesh_));
                static_mesh_scale = {geometry_desc.unreal_static_mesh_scale_, geometry_desc.unreal_static_mesh_scale_, geometry_desc.unreal_static_mesh_scale_};
                break;
            default:
                SP_ASSERT(false);
        }
        SP_ASSERT(static_mesh);

        // assign location, rotation, scale, static mesh
        static_mesh_component->SetRelativeLocation(static_mesh_location);
        static_mesh_component->SetRelativeRotation(static_mesh_rotation);
        static_mesh_component->SetRelativeScale3D(static_mesh_scale);
        static_mesh_component->SetStaticMesh(static_mesh);
        static_mesh_component->SetNotifyRigidBodyCollision(true);
        static_mesh_component->bUseDefaultCollision = true;

        // HACK(MR): When two sibling links are in collision, the physics engine will generate restitution forces to push the links out of collision.
        // Most of this time, this is the correct behavior. But if the two siblings are connected to their common parent via fixed joints, then
        // arguably the correct behavior would be to ignore the collision entirely, because the user has expressed their intent that each sibling
        // should be rigidly attached to the parent. However, in this case, Chaos will generate large restitution forces for the siblings, which will
        // cause excessive jittering. We include a custom flag to completely disable collisions for a link as a lightweight workaround in these
        // situations.
        if (link_desc->ignore_collisions_) {
            static_mesh_component->bUseDefaultCollision = false;
            static_mesh_component->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
        }

        // HACK (MR): Under normal circumstances, neighboring links won't collide with each other because the "Disable Collision" flag is enabled
        // by default on joints. But this flag has no effect when the "Simulate Physics" option is disabled for a particular link. So, when
        // "Simulate Physics" is disabled, we need to set the object type to be a special type (e.g., ECollisionChannel::ECC_Vehicle), and then set
        // all child links to ignore collisions for that type.
        if (!link_desc->simulate_physics_) {
            static_mesh_component->bUseDefaultCollision = false;
            static_mesh_component->SetCollisionObjectType(ECollisionChannel::ECC_Vehicle);
        }

        if (link_desc->has_parent_ && !link_desc->parent_simulate_physics_) {
            static_mesh_component->bUseDefaultCollision = false;
            static_mesh_component->SetCollisionResponseToChannel(ECollisionChannel::ECC_Vehicle, ECollisionResponse::ECR_Ignore);
        }

        // assign material
        if (visual_desc.has_material_) {

            // if the material desc is a reference, then follow the pointer to obtain the underlying material desc
            const UrdfMaterialDesc* material_desc = &(visual_desc.material_desc_); // needs to be const because link_desc is const
            if (material_desc->is_reference_) {
                material_desc = material_desc->material_desc_;
            }
            SP_ASSERT(material_desc);

            if (material_desc->unreal_material_ != "") {
                UMaterialInterface* material_interface = LoadObject<UMaterialInterface>(
                    nullptr, *Unreal::toFString(material_desc->unreal_material_));
                SP_ASSERT(material_interface);
                static_mesh_component->SetMaterial(0, material_interface);
            } else {
                UMaterialInterface* material_interface = LoadObject<UMaterialInterface>(
                    nullptr, *Unreal::toFString("/UrdfRobot/Common/Materials/M_PureColor.M_PureColor"));
                UMaterialInstanceDynamic* material_instance_dynamic = UMaterialInstanceDynamic::Create(material_interface, static_mesh_component);
                FLinearColor color = {
                    static_cast<float>(material_desc->color_.at(0)),
                    static_cast<float>(material_desc->color_.at(1)),
                    static_cast<float>(material_desc->color_.at(2)),
                    static_cast<float>(material_desc->color_.at(3))};
                material_instance_dynamic->SetVectorParameterValue("BaseColor_Color", color);
                static_mesh_component->SetMaterial(0, material_instance_dynamic);
            }
        }

        StaticMeshComponents.Add(static_mesh_component);
    }
}

std::map<std::string, ArrayDesc> UUrdfLinkComponent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;

    std::string name = Unreal::toStdString(GetName()) + ".location";
    ArrayDesc array_desc;
    array_desc.low_ = std::numeric_limits<double>::lowest();
    array_desc.high_ = std::numeric_limits<double>::max();
    array_desc.shape_ = {3}; // x, y, z in [cm] of each link relative to it's parent
    array_desc.datatype_ = DataType::Float64;
    Std::insert(observation_space, name, std::move(array_desc));

    name = Unreal::toStdString(GetName()) + ".rotation";
    array_desc.low_ = std::numeric_limits<double>::lowest();
    array_desc.high_ = std::numeric_limits<double>::max();
    array_desc.shape_ = {3}; // pitch, yaw, roll in [deg] of each link relative to it's parent
    array_desc.datatype_ = DataType::Float64;
    Std::insert(observation_space, name, std::move(array_desc));

    return observation_space;
}

std::map<std::string, std::vector<uint8_t>> UUrdfLinkComponent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    FVector location = GetRelativeLocation();
    std::string name = Unreal::toStdString(GetName()) + ".location";
    Std::insert(observation, name, Std::reinterpretAsVector<uint8_t, double>({location.X, location.Y, location.Z}));

    FRotator rotation = GetRelativeRotation();
    name = Unreal::toStdString(GetName()) + ".rotation";
    Std::insert(observation, name, Std::reinterpretAsVector<uint8_t, double>({rotation.Pitch, rotation.Yaw, rotation.Roll}));

    return observation;
}

void UUrdfLinkComponent::reset()
{
    FBodyInstance* body_instance = GetBodyInstance();
    SP_ASSERT(body_instance);
    if (body_instance->ShouldInstanceSimulatingPhysics()) {
        body_instance->SetLinearVelocity(FVector::ZeroVector, false);
        body_instance->SetAngularVelocityInRadians(FVector::ZeroVector, false);
        body_instance->ClearForces();
        body_instance->ClearTorques();
    }
}
