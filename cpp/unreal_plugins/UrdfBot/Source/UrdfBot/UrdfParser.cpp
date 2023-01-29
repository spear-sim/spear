//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfParser.h"

#include <XmlFile.h>
#include <XmlNode.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"

UrdfRobotDesc UrdfParser::parse(const std::string& file_name)
{
    FXmlFile file;
    bool is_file_loaded = file.LoadFile(Unreal::toFString(file_name));
    ASSERT(is_file_loaded);

    FXmlNode* robot_node = file.GetRootNode();
    ASSERT(robot_node->GetTag().Equals(TEXT("robot")));

    return parseRobotNode(robot_node);
}

UrdfGeometryDesc UrdfParser::parseGeometryNode(FXmlNode* geometry_node)
{
    UrdfGeometryDesc geometry;

    FXmlNode* box_node = geometry_node->FindChildNode(TEXT("box"));
    if (box_node) {
        geometry.type_ = UrdfGeometryType::Box;
        geometry.size_ = parseVector(box_node->GetAttribute(TEXT("size")));
    }

    FXmlNode* cylinder_node = geometry_node->FindChildNode(TEXT("cylinder"));
    if (cylinder_node) {
        geometry.type_   = UrdfGeometryType::Cylinder;
        geometry.length_ = FCString::Atof(*cylinder_node->GetAttribute(TEXT("length")));
        geometry.radius_ = FCString::Atof(*cylinder_node->GetAttribute(TEXT("radius")));
    }

    FXmlNode* sphere_node = geometry_node->FindChildNode(TEXT("sphere"));
    if (sphere_node) {
        geometry.type_   = UrdfGeometryType::Sphere;
        geometry.radius_ = FCString::Atof(*sphere_node->GetAttribute(TEXT("radius")));
    }

    FXmlNode* mesh_node = geometry_node->FindChildNode(TEXT("mesh"));
    if (mesh_node) {
        geometry.type_     = UrdfGeometryType::Mesh;
        geometry.filename_ = Unreal::toString(mesh_node->GetAttribute(TEXT("filename")));
        FString scale      = mesh_node->GetAttribute(TEXT("scale"));
        if (!scale.IsEmpty()) {
            geometry.scale_ = FCString::Atof(*scale);
        }
    }

    ASSERT(geometry.type_ != UrdfGeometryType::Invalid);

    return geometry;
}

UrdfMaterialDesc UrdfParser::parseMaterialNode(FXmlNode* material_node)
{
    UrdfMaterialDesc material_desc;

    material_desc.name_         = Unreal::toString(material_node->GetAttribute(TEXT("name")));
    material_desc.is_reference_ = true;

    FXmlNode* color_node = material_node->FindChildNode(TEXT("color"));
    if (color_node) {
        material_desc.color_        = parseVector4(color_node->GetAttribute(TEXT("rgba")));
        material_desc.is_reference_ = false;
    }

    FXmlNode* texture_node = material_node->FindChildNode(TEXT("texture"));
    if (texture_node) {
        material_desc.texture_      = Unreal::toString(texture_node->GetAttribute(TEXT("filename")));
        material_desc.is_reference_ = false;
    }
    
    // material node must be either have non-empty name_ or valid color or texture value
    ASSERT(material_desc.name_ != "" || !material_desc.is_reference_);

    return material_desc;
}

UrdfInertialDesc UrdfParser::parseInertialNode(FXmlNode* inertial_node)
{
    UrdfInertialDesc inertial_desc_;
    
    FXmlNode* origin_node = inertial_node->FindChildNode(TEXT("origin"));
    if (origin_node) {
        inertial_desc_.origin_.SetLocation(parseVector(origin_node->GetAttribute(TEXT("xyz"))));
        inertial_desc_.origin_.SetRotation(parseRotation(origin_node->GetAttribute(TEXT("rpy"))));
    }

    FXmlNode* mass_node = inertial_node->FindChildNode(TEXT("mass"));
    ASSERT(mass_node);
    inertial_desc_.mass_ = FCString::Atof(*mass_node->GetAttribute(TEXT("value")));

    FXmlNode* inertia_node = inertial_node->FindChildNode(TEXT("inertia"));
    ASSERT(inertia_node);
    inertial_desc_.inertia_.M[0][0] = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixx")));
    inertial_desc_.inertia_.M[0][1] = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixy")));
    inertial_desc_.inertia_.M[0][2] = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixz")));
    inertial_desc_.inertia_.M[1][0] = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixy")));
    inertial_desc_.inertia_.M[1][1] = FCString::Atof(*inertia_node->GetAttribute(TEXT("iyy")));
    inertial_desc_.inertia_.M[1][2] = FCString::Atof(*inertia_node->GetAttribute(TEXT("iyz")));
    inertial_desc_.inertia_.M[2][0] = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixz")));
    inertial_desc_.inertia_.M[2][1] = FCString::Atof(*inertia_node->GetAttribute(TEXT("iyz")));
    inertial_desc_.inertia_.M[2][2] = FCString::Atof(*inertia_node->GetAttribute(TEXT("izz")));

    return inertial_desc_;
}

UrdfVisualDesc UrdfParser::parseVisualNode(FXmlNode* visual_node)
{
    UrdfVisualDesc visual_desc_;

    visual_desc_.name_ = Unreal::toString(visual_node->GetAttribute(TEXT("name")));

    FXmlNode* origin_node = visual_node->FindChildNode(TEXT("origin"));
    if (origin_node) {
        visual_desc_.origin_.SetLocation(parseVector(origin_node->GetAttribute(TEXT("xyz"))));
        visual_desc_.origin_.SetRotation(parseRotation(origin_node->GetAttribute(TEXT("rpy"))));
    }

    FXmlNode* geometry_node = visual_node->FindChildNode(TEXT("geometry"));
    ASSERT(geometry_node);
    visual_desc_.geometry_desc_ = parseGeometryNode(geometry_node);

    FXmlNode* material_node = visual_node->FindChildNode(TEXT("material"));
    if (material_node) {
        visual_desc_.has_material_  = true;
        visual_desc_.material_desc_ = parseMaterialNode(material_node);
    }

    return visual_desc_;
}

UrdfCollisionDesc UrdfParser::parseCollisionNode(FXmlNode* collision_node)
{
    UrdfCollisionDesc collision_desc;

    collision_desc.name_ = Unreal::toString(collision_node->GetAttribute(TEXT("name")));

    FXmlNode* origin_node = collision_node->FindChildNode(TEXT("origin"));
    if (origin_node) {
        collision_desc.origin_.SetLocation(parseVector(origin_node->GetAttribute(TEXT("xyz"))));
        collision_desc.origin_.SetRotation(parseRotation(origin_node->GetAttribute(TEXT("rpy"))));
    }

    FXmlNode* geometry_node = collision_node->FindChildNode(TEXT("geometry"));
    ASSERT(geometry_node);
    collision_desc.geometry_desc_ = parseGeometryNode(geometry_node);

    return collision_desc;
}

UrdfLinkDesc UrdfParser::parseLinkNode(FXmlNode* link_node)
{
    UrdfLinkDesc link_desc;

    link_desc.name_ = Unreal::toString(link_node->GetAttribute(TEXT("name")));

    FXmlNode* inertial_node = link_node->FindChildNode(TEXT("inertial"));
    if (inertial_node) {
        link_desc.inertial_desc_ = parseInertialNode(inertial_node);
    }

    for (auto& child_node : link_node->GetChildrenNodes()) {
        const FString& tag = child_node->GetTag();
        if (tag.Equals(TEXT("visual"))) {
            link_desc.visual_descs_.push_back(parseVisualNode(child_node));
        } else if (tag.Equals(TEXT("collision"))) {
            link_desc.collision_descs_.push_back(parseCollisionNode(child_node));
        }
    }

    return link_desc;
}

UrdfJointDesc UrdfParser::parseJointNode(FXmlNode* joint_node)
{
    UrdfJointDesc joint_desc;

    joint_desc.name_ = Unreal::toString(joint_node->GetAttribute(TEXT("name")));

    FString type = joint_node->GetAttribute(TEXT("type"));
    if (type.Equals(TEXT("revolute"))) {
        joint_desc.type_ = UrdfJointType::Revolute;
    } else if (type.Equals(TEXT("continuous"))) {
        joint_desc.type_ = UrdfJointType::Continuous;
    } else if (type.Equals(TEXT("prismatic"))) {
        joint_desc.type_ = UrdfJointType::Prismatic;
    } else if (type.Equals(TEXT("fixed"))) {
        joint_desc.type_ = UrdfJointType::Fixed;
    } else if (type.Equals(TEXT("floating"))) {
        joint_desc.type_ = UrdfJointType::Floating;
    } else if (type.Equals(TEXT("planar"))) {
        joint_desc.type_ = UrdfJointType::Planar;
    } else {
        ASSERT(false);
    }

    FXmlNode* origin_node = joint_node->FindChildNode(TEXT("origin"));
    if (origin_node) {
        joint_desc.origin_.SetLocation(parseVector(origin_node->GetAttribute(TEXT("xyz"))));
        joint_desc.origin_.SetRotation(parseRotation(origin_node->GetAttribute(TEXT("rpy"))));
    }

    FXmlNode* parent_node = joint_node->FindChildNode(TEXT("parent"));
    ASSERT(parent_node);
    joint_desc.parent_ = Unreal::toString(parent_node->GetAttribute(TEXT("link")));

    FXmlNode* child_node = joint_node->FindChildNode(TEXT("child"));
    ASSERT(child_node);
    joint_desc.child_ = Unreal::toString(child_node->GetAttribute(TEXT("link")));

    FXmlNode* axis_node = joint_node->FindChildNode(TEXT("axis"));
    if (axis_node) {
        joint_desc.axis_ = parseVector(axis_node->GetAttribute(TEXT("xyz")));
    }

    FXmlNode* calibration_node = joint_node->FindChildNode(TEXT("calibration"));
    if (calibration_node) {
        FString rising = calibration_node->GetAttribute(TEXT("rising"));
        if (!rising.IsEmpty()) {
            ASSERT(joint_desc.calibration_type_ == CalibrationType::Invalid);
            joint_desc.calibration_type_ = CalibrationType::Rising;
            joint_desc.rising_           = FCString::Atof(*rising);
        }

        FString falling = calibration_node->GetAttribute(TEXT("falling"));
        if (!falling.IsEmpty()) {
            ASSERT(joint_desc.calibration_type_ == CalibrationType::Invalid);
            joint_desc.calibration_type_ = CalibrationType::Falling;
            joint_desc.falling_          = FCString::Atof(*falling);
        }
    }

    FXmlNode* dynamics_node = joint_node->FindChildNode(TEXT("dynamics"));
    if (dynamics_node) {
        joint_desc.damping_  = FCString::Atof(*dynamics_node->GetAttribute(TEXT("damping")));
        joint_desc.friction_ = FCString::Atof(*dynamics_node->GetAttribute(TEXT("friction")));
    }

    FXmlNode* limit_node = joint_node->FindChildNode(TEXT("limit"));
    if (joint_desc.type_ == UrdfJointType::Revolute || joint_desc.type_ == UrdfJointType::Prismatic) {
        ASSERT(limit_node);
    }
    if (limit_node) {
        joint_desc.lower_    = FCString::Atof(*limit_node->GetAttribute(TEXT("lower")));
        joint_desc.upper_    = FCString::Atof(*limit_node->GetAttribute(TEXT("upper")));
        joint_desc.effort_   = FCString::Atof(*limit_node->GetAttribute(TEXT("effort")));
        joint_desc.velocity_ = FCString::Atof(*limit_node->GetAttribute(TEXT("velocity")));
    }

    FXmlNode* mimic_node = joint_node->FindChildNode(TEXT("mimic"));
    if (mimic_node) {
        joint_desc.joint_ = Unreal::toString(mimic_node->GetAttribute(TEXT("joint")));

        FString multiplier = mimic_node->GetAttribute(TEXT("multiplier"));
        if (!multiplier.IsEmpty()) {
            joint_desc.multiplier_ = FCString::Atof(*multiplier);
        }

        joint_desc.offset_ = FCString::Atof(*mimic_node->GetAttribute(TEXT("offset")));
    }

    FXmlNode* safety_controller_node = joint_node->FindChildNode(TEXT("safety_controller"));
    if (safety_controller_node) {
        joint_desc.soft_lower_limit_ = FCString::Atof(*safety_controller_node->GetAttribute(TEXT("soft_lower_limit")));
        joint_desc.soft_upper_limit_ = FCString::Atof(*safety_controller_node->GetAttribute(TEXT("soft_upper_limit")));
        joint_desc.k_position_       = FCString::Atof(*safety_controller_node->GetAttribute(TEXT("k_position")));
        joint_desc.k_velocity_       = FCString::Atof(*safety_controller_node->GetAttribute(TEXT("k_velocity")));
    }

    return joint_desc;
}

UrdfRobotDesc UrdfParser::parseRobotNode(FXmlNode* robot_node)
{
    UrdfRobotDesc robot_desc;
    
    robot_desc.name_ = Unreal::toString(robot_node->GetAttribute(TEXT("name")));

    // parse all top-level URDF nodes into their own dictionaries
    for (auto& child_node : robot_node->GetChildrenNodes()) {
        const FString& tag = child_node->GetTag();

        if (tag.Equals(TEXT("link"))) {
            UrdfLinkDesc link_desc = parseLinkNode(child_node);
            ASSERT(!Std::containsKey(robot_desc.link_descs_, link_desc.name_));
            robot_desc.link_descs_[link_desc.name_] = std::move(link_desc);

        } else if (tag.Equals(TEXT("joint"))) {
            UrdfJointDesc joint_desc = parseJointNode(child_node);
            ASSERT(!Std::containsKey(robot_desc.joint_descs_, joint_desc.name_));
            robot_desc.joint_descs_[joint_desc.name_] = std::move(joint_desc);

        } else if (tag.Equals(TEXT("material"))) {
            UrdfMaterialDesc material_desc = parseMaterialNode(child_node);
            ASSERT(material_desc.name_ != "");
            ASSERT(!material_desc.is_reference_);
            ASSERT(!Std::containsKey(robot_desc.material_descs_, material_desc.name_));
            robot_desc.material_descs_[material_desc.name_] = std::move(material_desc);

        } else {
            ASSERT(false);
        }
    }

    // for a robot with N links, there must be N-1 joints.
    ASSERT(robot_desc.joint_descs_.size() + 1 == robot_desc.link_descs_.size());

    // for each joint, update its parent and child links
    for (auto& joint_desc_pair : robot_desc.joint_descs_) {
        UrdfJointDesc* joint_desc = &(joint_desc_pair.second);
        ASSERT(joint_desc);

        UrdfLinkDesc* parent_link_desc = &(robot_desc.link_descs_.at(joint_desc->parent_));
        UrdfLinkDesc* child_link_desc = &(robot_desc.link_descs_.at(joint_desc->child_));
        ASSERT(parent_link_desc);
        ASSERT(child_link_desc);

        parent_link_desc->child_link_descs_.push_back(child_link_desc);
        parent_link_desc->child_joint_descs_.push_back(joint_desc);

        // each link should only be visited as a child link at most once
        ASSERT(!child_link_desc->has_parent_);
        child_link_desc->has_parent_ = true;
        child_link_desc->parent_joint_desc_ = joint_desc;
    }

    // update pointers for root node and material node
    for (auto& link_desc_pair : robot_desc.link_descs_) {
        UrdfLinkDesc& link_desc = link_desc_pair.second;

        // for each material, update its pointer
        for (auto& visual_desc_ : link_desc.visual_descs_) {
            UrdfMaterialDesc& material_desc = visual_desc_.material_desc_;

            if (visual_desc_.has_material_ && material_desc.is_reference_) {
                UrdfMaterialDesc* target_material_desc = &(robot_desc.material_descs_.at(material_desc.name_));
                ASSERT(target_material_desc);
                material_desc.material_desc_ = target_material_desc;
            }
        }
        
        // find the root link
        if (!link_desc.has_parent_) {
            ASSERT(!robot_desc.root_link_desc_);
            robot_desc.root_link_desc_ = &(link_desc);
        }
    }

    // a robot must have only one root link
    ASSERT(robot_desc.root_link_desc_);

    return robot_desc;
}

FVector UrdfParser::parseVector(const FString& input_string)
{
    TArray<FString> split_string_array;
    input_string.ParseIntoArray(split_string_array, TEXT(" "), true);
    ASSERT(split_string_array.Num() == 3);
    return FVector(FCString::Atof(*split_string_array[0]), FCString::Atof(*split_string_array[1]), FCString::Atof(*split_string_array[2]));
}

FVector4 UrdfParser::parseVector4(const FString& input_string)
{
    TArray<FString> split_string_array;
    input_string.ParseIntoArray(split_string_array, TEXT(" "), true);
    ASSERT(split_string_array.Num() == 4);
    return FVector4(FCString::Atof(*split_string_array[0]), FCString::Atof(*split_string_array[1]), FCString::Atof(*split_string_array[2]), FCString::Atof(*split_string_array[3]));
}

FQuat UrdfParser::parseRotation(const FString& input_string)
{
    if (input_string.IsEmpty()) {
        return FRotator::ZeroRotator.Quaternion();
    }

    TArray<FString> split_string_array;
    input_string.ParseIntoArray(split_string_array, TEXT(" "), true);
    ASSERT(split_string_array.Num() == 3);
    return FMath::RadiansToDegrees(FRotator(FCString::Atof(*split_string_array[1]), FCString::Atof(*split_string_array[2]), FCString::Atof(*split_string_array[0]))).Quaternion();
}
