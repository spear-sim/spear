//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfLinkComponent.h"

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "UrdfParser.h"

void UUrdfLinkComponent::initialize(UrdfLinkDesc* link_desc)
{
    // for now link with no visual node or multiple visual node is not supported
    ASSERT(link_desc->visual_descs_.size() == 1);

    const UrdfVisualDesc& visual_desc = link_desc->visual_descs_[0];
    const UrdfGeometryDesc& geometry_desc = visual_desc.geometry_desc_;

    UStaticMesh* static_mesh = nullptr;
    FVector type_scale;
    if (geometry_desc.type_ == UrdfGeometryType::Box) {
        static_mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
        type_scale = geometry_desc.size_;
    }
    else if (geometry_desc.type_ == UrdfGeometryType::Cylinder) {
        static_mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
        type_scale = FVector(geometry_desc.radius_ * 2.0f, geometry_desc.radius_ * 2.0f, geometry_desc.length_);
    }
    else if (geometry_desc.type_ == UrdfGeometryType::Sphere) {
        static_mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
        type_scale = FVector(geometry_desc.radius_ * 2.0f, geometry_desc.radius_ * 2.0f, geometry_desc.radius_ * 2.0f);
    }
    else if (geometry_desc.type_ == UrdfGeometryType::Mesh) {
        static_mesh = LoadObject<UStaticMesh>(nullptr, *FString(geometry_desc.filename_.c_str()));
        type_scale = FVector(geometry_desc.scale_);
    }
    else {
        ASSERT(false);
    }
    ASSERT(static_mesh);

    this->SetStaticMesh(static_mesh);
    this->SetRelativeScale3D(type_scale);

    // set physical property
    this->SetSimulatePhysics(true);
    this->SetMobility(EComponentMobility::Movable);
    this->SetCollisionObjectType(ECollisionChannel::ECC_Vehicle);
    this->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
    this->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
    this->SetCollisionResponseToChannel(ECC_Vehicle, ECR_Overlap); // ignore collision between robot links
    this->SetNotifyRigidBodyCollision(true);

    // set never sleep
    this->GetBodyInstance()->SleepFamily = ESleepFamily::Custom;
    this->GetBodyInstance()->CustomSleepThresholdMultiplier = 0.f;

    // set physics body's solver iteration count to be better stabalized with more CPU intensive
    this->GetBodyInstance()->PositionSolverIterationCount = Config::get<float>("URDFBOT.URDFBOT_PAWN.POSITION_IITERATION_COUNT");
    this->GetBodyInstance()->VelocitySolverIterationCount = Config::get<float>("URDFBOT.URDFBOT_PAWN.VELOCITY_IITERATION_COUNT");

    // set mass
    if (link_desc->inertial_desc_.mass_ > 0) {
        this->SetMassOverrideInKg(NAME_None, link_desc->inertial_desc_.mass_, true);
    }

    // set rendering material
    if (visual_desc.has_material_) {
        const UrdfMaterialDesc* material_desc = &(visual_desc.material_desc_);
        if (material_desc->is_reference_) {
            material_desc = material_desc->material_desc_;
        }
        ASSERT(material_desc);

        UMaterialInterface* material = LoadObject<UMaterialInterface>(nullptr, *FString(material_desc->texture_.c_str()));
        ASSERT(material);

        this->SetMaterial(0, material);
    }
}
