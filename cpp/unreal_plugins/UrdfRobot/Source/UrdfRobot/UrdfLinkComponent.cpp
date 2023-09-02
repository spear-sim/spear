//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfLinkComponent.h"

#include <map>
#include <string>

#include <CoreMinimal.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/StaticMesh.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Materials/MaterialInterface.h>
#include <Math/Color.h>
#include <Math/Rotator.h>
#include <Math/UnrealMathUtility.h>
#include <Math/Vector.h>
#include <PhysicalMaterials/PhysicalMaterial.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfParser.h"

UUrdfLinkComponent::UUrdfLinkComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

UUrdfLinkComponent::~UUrdfLinkComponent()
{
    SP_LOG_CURRENT_FUNCTION();

    // Objects created with LoadObject and NewObject don't need to be cleaned up explicitly.

    StaticMeshComponents.Empty();
}

void UUrdfLinkComponent::BeginPlay()
{
    UStaticMeshComponent::BeginPlay();

    // Prevent the link from sleeping, otherwise it won't wake up when we apply position-or-velocity-based control actions on joints
    FBodyInstance* body_instance = GetBodyInstance();
    SP_ASSERT(body_instance);
    UPhysicalMaterial* physical_material = body_instance->GetSimplePhysicalMaterial();
    SP_ASSERT(physical_material);
    UPhysicalMaterial* physical_material_override = DuplicateObject<UPhysicalMaterial>(physical_material, this, "physical_material_override");
    physical_material_override->SleepAngularVelocityThreshold = 0.0f;
    physical_material_override->SleepLinearVelocityThreshold = 0.0f;
    body_instance->SetPhysMaterialOverride(physical_material_override);
}

void UUrdfLinkComponent::initialize(const UrdfLinkDesc* link_desc)
{
    double m_to_cm = 100.0;

    // set our own reference frame using the top-level UrdfLinkDesc
    FVector location = FVector({link_desc->xyz_.at(0), link_desc->xyz_.at(1), link_desc->xyz_.at(2)}) * m_to_cm; // m to cm
    FRotator rotation = FMath::RadiansToDegrees(FRotator({link_desc->rpy_.at(1), link_desc->rpy_.at(2), link_desc->rpy_.at(0)})); // rpy to pyr, rad to deg

    // assign location, rotation, scale, for the top-level static mesh
    auto link_static_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString("/UrdfRobot/Common/Meshes/SM_Dummy.SM_Dummy"));

    SetRelativeLocation(location);
    SetRelativeRotation(rotation);
    SetStaticMesh(link_static_mesh);

    // set physical properties
    SetSimulatePhysics(link_desc->simulate_physics_);
    SetNotifyRigidBodyCollision(true);
    bUseDefaultCollision = true;

    // inertial
    SetMassOverrideInKg(NAME_None, link_desc->inertial_desc_.mass_, true);

    // visual
    for (auto& visual_desc : link_desc->visual_descs_) {

        // needs to be const because spear_link_desc is const
        const UrdfGeometryDesc& geometry_desc = visual_desc.geometry_desc_;

        // create child static mesh component for each UrdfSpearLinkDesc
        auto static_mesh_component = NewObject<UStaticMeshComponent>(this, Unreal::toFName("link." + link_desc->name_ + ".static_mesh"));
        SP_ASSERT(static_mesh_component);

        // each UrdfSpearLinkDesc has its own reference frame
        FVector static_mesh_location = FVector(
            {visual_desc.xyz_.at(0), visual_desc.xyz_.at(1), visual_desc.xyz_.at(2)}) * m_to_cm; // m to cm
        FRotator static_mesh_rotation = FMath::RadiansToDegrees(FRotator(
            {visual_desc.rpy_.at(1), visual_desc.rpy_.at(2), visual_desc.rpy_.at(0) })); // rpy to pyr, rad to deg
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

        // assign material
        if (visual_desc.has_material_) {

            // if the material desc is a reference, then follow the pointer to obtain the reference
            const UrdfMaterialDesc* material_desc = &(visual_desc.material_desc_); // needs to be const because spear_link_desc is const
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
                    nullptr, *Unreal::toFString("Material'/UrdfRobot/Common/Materials/M_PureColor.M_PureColor'"));
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

        // connect static mesh component to the top-level link
        static_mesh_component->SetupAttachment(this);
        static_mesh_component->RegisterComponent();
        StaticMeshComponents.Add(static_mesh_component);
    }
}
