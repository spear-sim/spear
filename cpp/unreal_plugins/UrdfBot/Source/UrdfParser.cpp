//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfParser.h"

#include <XmlFile.h>
#include <XmlNode.h>
#include <XmlParser.h>

#include "Assert/Assert.h"

UrdfRobotDesc UrdfParser::parse(const std::string& file_name)
{
    FXmlFile file;
    file.LoadFile(UTF8_TO_TCHAR(file_name.c_str()));

    FXmlNode* robot_node = file.GetRootNode();
    ASSERT(robot_node->GetTag().Equals(TEXT("robot")));

    return parseRobotNode(robot_node);
}

UrdfGeometryDesc UrdfParser::parseGeometryNode(FXmlNode* geometry_node)
{
    UrdfGeometryDesc geometry;
    FXmlNode* child_node = nullptr;

    child_node = geometry_node->FindChildNode("box");
    if (child_node) {
        geometry.type_ = UrdfGeometryType::Box;
        geometry.size_ = parseVector(child_node->GetAttribute("size"));
    }

    child_node = geometry_node->FindChildNode("cylinder");
    if (child_node) {
        geometry.type_   = UrdfGeometryType::Cylinder;
        geometry.length_ = FCString::Atof(*child_node->GetAttribute("length"));
        geometry.radius_ = FCString::Atof(*child_node->GetAttribute("radius"));
    }

    child_node = geometry_node->FindChildNode("sphere");
    if (child_node) {
        geometry.type_   = UrdfGeometryType::Sphere;
        geometry.radius_ = FCString::Atof(*child_node->GetAttribute("radius"));
    }

    child_node = geometry_node->FindChildNode("mesh");
    if (child_node) {
        geometry.type_     = UrdfGeometryType::Mesh;
        geometry.filename_ = TCHAR_TO_UTF8(*child_node->GetAttribute("filename"));
        FString scale      = child_node->GetAttribute("scale");
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
    FXmlNode* child_node = nullptr;

    material_desc.name_         = TCHAR_TO_UTF8(*material_node->GetAttribute(TEXT("name")));
    material_desc.is_reference_ = true;

    child_node = material_node->FindChildNode("color");
    if (child_node) {
        material_desc.color_        = parseVector4(child_node->GetAttribute(TEXT("rgba")));
        material_desc.is_reference_ = false;
    }

    child_node = material_node->FindChildNode("texture");
    if (child_node) {
        material_desc.texture_      = TCHAR_TO_UTF8(*child_node->GetAttribute(TEXT("filename")));
        material_desc.is_reference_ = false;
    }
    
    // material node must be either have non-empty name_ or valid color or texture value
    ASSERT(material_desc.name_ != "" || !material_desc.is_reference_);

    return material_desc;
}

UrdfInertialDesc UrdfParser::parseInertialNode(FXmlNode* inertial_node)
{
    UrdfInertialDesc inertial_desc_;
    FXmlNode* child_node = nullptr;
    
    child_node = inertial_node->FindChildNode("origin");
    if (child_node) {
        inertial_desc_.origin_.SetLocation(parseVector(child_node->GetAttribute("xyz")));
        inertial_desc_.origin_.SetRotation(parseRotation(child_node->GetAttribute("rpy")));
    }

    child_node = inertial_node->FindChildNode("mass");
    ASSERT(child_node);
    inertial_desc_.mass_ = FCString::Atof(*child_node->GetAttribute(TEXT("value")));

    child_node = inertial_node->FindChildNode("inertia");
    ASSERT(child_node);
    inertial_desc_.inertia_.M[0][0] = FCString::Atof(*child_node->GetAttribute(TEXT("ixx")));
    inertial_desc_.inertia_.M[0][1] = FCString::Atof(*child_node->GetAttribute(TEXT("ixy")));
    inertial_desc_.inertia_.M[0][2] = FCString::Atof(*child_node->GetAttribute(TEXT("ixz")));
    inertial_desc_.inertia_.M[1][0] = FCString::Atof(*child_node->GetAttribute(TEXT("ixy")));
    inertial_desc_.inertia_.M[1][1] = FCString::Atof(*child_node->GetAttribute(TEXT("iyy")));
    inertial_desc_.inertia_.M[1][2] = FCString::Atof(*child_node->GetAttribute(TEXT("iyz")));
    inertial_desc_.inertia_.M[2][0] = FCString::Atof(*child_node->GetAttribute(TEXT("ixz")));
    inertial_desc_.inertia_.M[2][1] = FCString::Atof(*child_node->GetAttribute(TEXT("iyz")));
    inertial_desc_.inertia_.M[2][2] = FCString::Atof(*child_node->GetAttribute(TEXT("izz")));

    return inertial_desc_;
}

UrdfVisualDesc UrdfParser::parseVisualNode(FXmlNode* visual_node)
{
    UrdfVisualDesc visual_desc_;
    FXmlNode* child_node = nullptr;

    visual_desc_.name_ = TCHAR_TO_UTF8(*visual_node->GetAttribute(TEXT("name")));

    child_node = visual_node->FindChildNode("origin");
    if (child_node) {
        visual_desc_.origin_.SetLocation(parseVector(child_node->GetAttribute("xyz")));
        visual_desc_.origin_.SetRotation(parseRotation(child_node->GetAttribute("rpy")));
    }

    child_node = visual_node->FindChildNode("geometry");
    ASSERT(child_node);
    visual_desc_.geometry_desc_ = parseGeometryNode(child_node);

    child_node = visual_node->FindChildNode("material");
    if (child_node) {
        visual_desc_.has_material_  = true;
        visual_desc_.material_desc_ = parseMaterialNode(child_node);
    }

    return visual_desc_;
}

UrdfCollisionDesc UrdfParser::parseCollisionNode(FXmlNode* collision_node)
{
    UrdfCollisionDesc collision_desc;
    FXmlNode* child_node = nullptr;

    collision_desc.name_ = TCHAR_TO_UTF8(*collision_node->GetAttribute(TEXT("name")));

    child_node = collision_node->FindChildNode("origin");
    if (child_node) {
        collision_desc.origin_.SetLocation(parseVector(child_node->GetAttribute("xyz")));
        collision_desc.origin_.SetRotation(parseRotation(child_node->GetAttribute("rpy")));
    }

    child_node = collision_node->FindChildNode("geometry");
    ASSERT(child_node);
    collision_desc.geometry_desc_ = parseGeometryNode(child_node);

    return collision_desc;
}

UrdfLinkDesc UrdfParser::parseLinkNode(FXmlNode* link_node)
{
    UrdfLinkDesc link_desc;
    FXmlNode* child_node = nullptr;

    link_desc.name_ = TCHAR_TO_UTF8(*link_node->GetAttribute(TEXT("name")));

    child_node = link_node->FindChildNode("inertial");
    if (child_node) {
        link_desc.inertial_desc_ = parseInertialNode(child_node);
    }

    for (auto& node : link_node->GetChildrenNodes()) {
        const FString& tag = node->GetTag();

        if (tag.Equals(TEXT("visual"))) {
            UrdfVisualDesc visual_desc = parseVisualNode(node);
            link_desc.visual_descs_.push_back(visual_desc);

        } else if (tag.Equals(TEXT("collision"))) {
            UrdfCollisionDesc collision_desc = parseCollisionNode(node);
            link_desc.collision_descs_.push_back(collision_desc);
        }
    }

    return link_desc;
}

UrdfJointDesc UrdfParser::parseJointNode(FXmlNode* joint_node)
{
    UrdfJointDesc joint_desc;
    FXmlNode* child_node = nullptr;

    joint_desc.name_ = TCHAR_TO_UTF8(*joint_node->GetAttribute(TEXT("name")));

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

    child_node = joint_node->FindChildNode("origin");
    if (child_node) {
        joint_desc.origin_.SetLocation(parseVector(child_node->GetAttribute("xyz")));
        joint_desc.origin_.SetRotation(parseRotation(child_node->GetAttribute("rpy")));
    }

    child_node = joint_node->FindChildNode("parent");
    ASSERT(child_node);
    joint_desc.parent_ = TCHAR_TO_UTF8(*child_node->GetAttribute(TEXT("link")));

    child_node = joint_node->FindChildNode("child");
    ASSERT(child_node);
    joint_desc.child_ = TCHAR_TO_UTF8(*child_node->GetAttribute(TEXT("link")));

    child_node = joint_node->FindChildNode("axis");
    if (child_node) {
        joint_desc.axis_ = parseVector(child_node->GetAttribute(TEXT("xyz")));
    }

    child_node = joint_node->FindChildNode("calibration");
    if (child_node) {
        FString rising = child_node->GetAttribute(TEXT("rising"));
        if (!rising.IsEmpty()) {
            ASSERT(joint_desc.calibration_type_ == CalibrationType::Invalid);
            joint_desc.calibration_type_ = CalibrationType::Rising;
            joint_desc.rising_           = FCString::Atof(*rising);
        }

        FString falling = child_node->GetAttribute(TEXT("falling"));
        if (!falling.IsEmpty()) {
            ASSERT(joint_desc.calibration_type_ == CalibrationType::Invalid);
            joint_desc.calibration_type_ = CalibrationType::Falling;
            joint_desc.falling_          = FCString::Atof(*falling);
        }
    }

    child_node = joint_node->FindChildNode("dynamics");
    if (child_node) {
        joint_desc.damping_  = FCString::Atof(*child_node->GetAttribute(TEXT("damping")));
        joint_desc.friction_ = FCString::Atof(*child_node->GetAttribute(TEXT("friction")));
    }

    child_node = joint_node->FindChildNode("limit");
    if (joint_desc.type_ == UrdfJointType::Revolute || joint_desc.type_ == UrdfJointType::Prismatic) {
        ASSERT(child_node);
    }
    if (child_node) {
        joint_desc.lower_    = FCString::Atof(*child_node->GetAttribute(TEXT("lower")));
        joint_desc.upper_    = FCString::Atof(*child_node->GetAttribute(TEXT("upper")));
        joint_desc.effort_   = FCString::Atof(*child_node->GetAttribute(TEXT("effort")));
        joint_desc.velocity_ = FCString::Atof(*child_node->GetAttribute(TEXT("velocity")));
    }

    child_node = joint_node->FindChildNode("mimic");
    if (child_node) {
        joint_desc.joint_  = TCHAR_TO_UTF8(*child_node->GetAttribute(TEXT("joint")));

        FString multiplier = child_node->GetAttribute(TEXT("multiplier"));
        if (!multiplier.IsEmpty()) {
            joint_desc.multiplier_ = FCString::Atof(*multiplier);
        }

        joint_desc.offset_ = FCString::Atof(*child_node->GetAttribute(TEXT("offset")));
    }

    child_node = joint_node->FindChildNode("safety_controller");
    if (child_node) {
        joint_desc.soft_lower_limit_ = FCString::Atof(*child_node->GetAttribute(TEXT("soft_lower_limit")));
        joint_desc.soft_upper_limit_ = FCString::Atof(*child_node->GetAttribute(TEXT("soft_upper_limit")));
        joint_desc.k_position_       = FCString::Atof(*child_node->GetAttribute(TEXT("k_position")));
        joint_desc.k_velocity_       = FCString::Atof(*child_node->GetAttribute(TEXT("k_velocity")));
    }

    return joint_desc;
}

UrdfRobotDesc UrdfParser::parseRobotNode(FXmlNode* robot_node)
{
    UrdfRobotDesc robot_desc;
    
    robot_desc.name_ = TCHAR_TO_UTF8(*robot_node->GetAttribute(TEXT("name")));

    // parse all top-level URDF nodes into their own dictionaries
    for (auto& node : robot_node->GetChildrenNodes()) {
        const FString& tag = node->GetTag();

        if (tag.Equals(TEXT("link"))) {
            UrdfLinkDesc link_desc = parseLinkNode(node);
            ASSERT(!robot_desc.link_descs_.count(link_desc.name_));
            robot_desc.link_descs_[link_desc.name_] = std::move(link_desc);

        } else if (tag.Equals(TEXT("joint"))) {
            UrdfJointDesc joint_desc = parseJointNode(node);
            ASSERT(!robot_desc.joint_descs_.count(joint_desc.name_));
            robot_desc.joint_descs_[joint_desc.name_] = std::move(joint_desc);

        } else if (tag.Equals(TEXT("material"))) {
            UrdfMaterialDesc material_desc = parseMaterialNode(node);
            ASSERT(material_desc.name_ != "" && !material_desc.is_reference_);
            ASSERT(!robot_desc.material_descs_.count(material_desc.name_));
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
