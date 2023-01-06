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

UrdfRobotDesc UrdfParser::parseRobotNode(FXmlNode* robot_node)
{
    UrdfRobotDesc robot_desc;
    // parse all nodes
    for (auto& child_node : robot_node->GetChildrenNodes()) {
        const FString& tag = child_node->GetTag();

        if (tag.Equals(TEXT("link"))) {
            UrdfLinkDesc link_desc = parseLinkNode(child_node);
            ASSERT(!robot_desc.link_descs_.count(link_desc.name_));
            robot_desc.link_descs_[link_desc.name_] = std::move(link_desc);
        } else if (tag.Equals(TEXT("joint"))) {
            UrdfJointDesc joint_desc = parseJointNode(child_node);
            ASSERT(!robot_desc.joint_descs_.count(joint_desc.name_));
            robot_desc.joint_descs_[joint_desc.name_] = std::move(joint_desc);
        } else if (tag.Equals(TEXT("material"))) {
            UrdfMaterialDesc material_desc = parseMaterialNode(child_node);
            ASSERT(!robot_desc.material_descs_.count(material_desc.name_));
            robot_desc.material_descs_[material_desc.name_] = std::move(material_desc);
        }
    }

    // for a robot with N links, there must be N-1 joints.
    ASSERT(robot_desc.joint_descs_.size() + 1 == robot_desc.link_descs_.size());

    // build link tree based on joints
    for (auto& joint_desc_pair : robot_desc.joint_descs_) {
        UrdfJointDesc* joint_desc = &(joint_desc_pair.second);
        ASSERT(joint_desc);

        UrdfLinkDesc* parent_link_desc = &robot_desc.link_descs_.at(joint_desc->parent_link_name_);
        UrdfLinkDesc* child_link_desc = &robot_desc.link_descs_.at(joint_desc->child_link_name_);

        ASSERT(parent_link_desc);
        ASSERT(child_link_desc);

        // update the joint's pointers
        ASSERT(!joint_desc->parent_link_desc_);
        ASSERT(!joint_desc->parent_link_desc_);
        joint_desc->parent_link_desc_ = parent_link_desc;
        joint_desc->child_link_desc_ = child_link_desc;

        // update the child's pointers (a child must have one parent)
        ASSERT(!child_link_desc->parent_link_desc_);
        ASSERT(!child_link_desc->parent_joint_desc_);
        child_link_desc->parent_link_desc_ = parent_link_desc;
        child_link_desc->parent_joint_desc_ = joint_desc;

        // update the parent's pointers (a parent can have multiple children)
        parent_link_desc->child_link_descs_.push_back(child_link_desc);
        parent_link_desc->child_joint_descs_.push_back(joint_desc);
    }

    // for each link, update its material pointer, and find the root link
    for (auto& link_desc_pair : robot_desc.link_descs_) {
        UrdfLinkDesc* link_desc = &(link_desc_pair.second);
        ASSERT(link_desc);

        //bind material description
        if (link_desc->material_name_ != "") {
            UrdfMaterialDesc* material_desc = &robot_desc.material_descs_.at(link_desc->material_name_);
            ASSERT(material_desc);

            // update the link's material pointer (a link can have at most one material pointer)
            ASSERT(!link_desc->material_desc_);
            link_desc->material_desc_ = material_desc;
        }
        //find the root link
        if (!link_desc->parent_link_desc_) {
            ASSERT(!robot_desc.root_link_desc_);
            robot_desc.root_link_desc_ = link_desc;
        }
    }

    // a robot must have only one root link
    ASSERT(robot_desc.root_link_desc_);

    return robot_desc;
}

// parse urdf nodes
UrdfLinkDesc UrdfParser::parseLinkNode(FXmlNode* link_node)
{
    UrdfLinkDesc link_desc;
    link_desc.name_ = TCHAR_TO_UTF8(*link_node->GetAttribute(TEXT("name")));

    bool has_inertia_node = false;
    bool has_visual_node = false;
    bool has_collision_node = false;

    for (auto& node : link_node->GetChildrenNodes()) {
        FString tag = node->GetTag();
        if (tag.Equals(TEXT("inertial"))) {
            ASSERT(!has_inertia_node);
            has_inertia_node = true;
            parseLinkInertialNode(node, link_desc);
        } else if (tag.Equals(TEXT("visual"))) {
            ASSERT(!has_visual_node);
            has_visual_node = true;
            parseLinkVisualNode(node, link_desc);
        } else if (tag.Equals(TEXT("collision"))) {
            ASSERT(!has_collision_node);
            has_collision_node = true;
            parseLinkCollisionNode(node, link_desc);
        }
    }

    return link_desc;
}

UrdfJointDesc UrdfParser::parseJointNode(FXmlNode* joint_node)
{
    UrdfJointDesc joint_desc;

    joint_desc.name_ = TCHAR_TO_UTF8(*joint_node->GetAttribute(TEXT("name")));
    joint_desc.type_ = parseJointType(joint_node->GetAttribute(TEXT("type")));

    bool has_origin_node = false;
    bool has_parent_node = false;
    bool has_child_node = false;
    bool has_axis_node = false;
    bool has_dynamics_node = false;
    bool has_limit_node = false;

    for (auto& node : joint_node->GetChildrenNodes()) {
        FString tag = node->GetTag();
        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            joint_desc.origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            joint_desc.origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("parent"))) {
            ASSERT(!has_parent_node);
            has_parent_node = true;
            joint_desc.parent_link_name_ = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("link")));
        } else if (tag.Equals(TEXT("child"))) {
            ASSERT(!has_child_node);
            has_child_node = true;
            joint_desc.child_link_name_ = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("link")));
        } else if (tag.Equals(TEXT("axis"))) {
            ASSERT(!has_axis_node);
            has_axis_node = true;
            joint_desc.axis_ = parseVector(node->GetAttribute(TEXT("xyz")));
        } else if (tag.Equals(TEXT("dynamics"))) {
            ASSERT(!has_dynamics_node);
            has_dynamics_node = true;
            joint_desc.damping_ = FCString::Atof(*node->GetAttribute(TEXT("damping")));
            joint_desc.friction_ = FCString::Atof(*node->GetAttribute(TEXT("friction")));
        } else if (tag.Equals(TEXT("limit"))) {
            ASSERT(!has_limit_node);
            has_limit_node = true;
            joint_desc.lower_ = FCString::Atof(*node->GetAttribute(TEXT("lower")));
            joint_desc.upper_ = FCString::Atof(*node->GetAttribute(TEXT("upper")));
            joint_desc.velocity_ = FCString::Atof(*node->GetAttribute(TEXT("velocity")));
            joint_desc.effort_ = FCString::Atof(*node->GetAttribute(TEXT("effort")));
        }
    }

    ASSERT(has_parent_node);
    ASSERT(has_child_node);
    // revolute and prismatic joints require limit
    ASSERT(!(joint_desc.type_ == REVOLUTE_TYPE || joint_desc.type_ == PRISMATIC_TYPE) || has_limit_node);

    return joint_desc;
}

UrdfMaterialDesc UrdfParser::parseMaterialNode(FXmlNode* material_node)
{
    UrdfMaterialDesc material_desc;
    material_desc.name_ = TCHAR_TO_UTF8(*material_node->GetAttribute(TEXT("name")));

    for (auto& node : material_node->GetChildrenNodes()) {
        FString tag = node->GetTag();
        if (tag.Equals(TEXT("color"))) {
            material_desc.color_ = parseVector4(node->GetAttribute(TEXT("rgba")));
        } else if (tag.Equals(TEXT("texture"))) {
            material_desc.file_name_ = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("filename")));
        }
    }

    return material_desc;
}

void UrdfParser::parseLinkInertialNode(FXmlNode* inertial_node, UrdfLinkDesc& link_desc)
{
    bool has_origin_node = false;
    bool has_mass_node = false;
    bool has_inertia_node = false;

    for (auto& node : inertial_node->GetChildrenNodes()) {
        FString tag = node->GetTag();

        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            link_desc.origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            link_desc.origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("mass"))) {
            ASSERT(!has_mass_node);
            has_mass_node = true;
            link_desc.mass_ = FCString::Atof(*node->GetAttribute(TEXT("value")));
        } else if (tag.Equals(TEXT("inertia"))) {
            ASSERT(!has_inertia_node);
            has_inertia_node = true;
            link_desc.inertia_ = parseInertiaMatrixNode(node);
        }
    }

    ASSERT(has_mass_node);
    ASSERT(has_inertia_node);
}

void UrdfParser::parseLinkVisualNode(FXmlNode* visual_node, UrdfLinkDesc& link_desc)
{
    bool has_origin_node = false;
    bool has_geometry_node = false;
    bool has_material_node = false;

    for (auto& node : visual_node->GetChildrenNodes()) {
        FString tag = node->GetTag();
        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            link_desc.visual_origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            link_desc.visual_origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("geometry"))) {
            ASSERT(!has_geometry_node);
            has_geometry_node = true;
            link_desc.visual_geometry_ = parseGeometryNode(node);
        } else if (tag.Equals(TEXT("material"))) {
            ASSERT(!has_material_node);
            has_material_node = true;
            link_desc.material_name_ = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("name")));
        }
    }
    ASSERT(has_geometry_node);
}

void UrdfParser::parseLinkCollisionNode(FXmlNode* collision_node, UrdfLinkDesc& link_desc)
{
    bool has_origin_node = false;
    bool has_geometry_node = false;

    for (auto& node : collision_node->GetChildrenNodes()) {
        FString tag = node->GetTag();
        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            link_desc.collision_origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            link_desc.collision_origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("geometry"))) {
            ASSERT(!has_geometry_node);
            has_geometry_node = true;
            link_desc.collision_geometry_ = parseGeometryNode(node);
        }
    }
    ASSERT(has_geometry_node);
}

UrdfGeometry UrdfParser::parseGeometryNode(FXmlNode* geometry_node)
{
    UrdfGeometry geometry;
    auto& child_nodes = geometry_node->GetChildrenNodes();
    ASSERT(child_nodes.Num() == 1);
    auto& node = child_nodes[0];
    FString tag = node->GetTag();

    if (tag.Equals(TEXT("box"))) {
        geometry.type_ = BOX;
        geometry.size_ = parseVector(node->GetAttribute("size"));
    } else if (tag.Equals(TEXT("cylinder"))) {
        geometry.type_ = CYLINDER;
        geometry.length_ = FCString::Atof(*node->GetAttribute("length"));
        geometry.radius_ = FCString::Atof(*node->GetAttribute("radius"));
    } else if (tag.Equals(TEXT("sphere"))) {
        geometry.type_ = SPHERE;
        geometry.radius_ = FCString::Atof(*node->GetAttribute("radius"));
    } else if (tag.Equals(TEXT("mesh"))) {
        geometry.type_ = MESH;
        geometry.file_name_ = TCHAR_TO_UTF8(*node->GetAttribute("filename"));
    } else {
        // invalid geometry type
        ASSERT(false);
    }
    return geometry;
}

FVector UrdfParser::parseVector(const FString& input_string)
{
    TArray<FString> split_string_array;
    input_string.ParseIntoArray(split_string_array, TEXT(" "), true);
    ASSERT(split_string_array.Num() == 3);
    return FVector(FCString::Atof(*split_string_array[0]), FCString::Atof(*split_string_array[1]), FCString::Atof(*split_string_array[2]));
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

FVector4 UrdfParser::parseVector4(const FString& input_string)
{
    TArray<FString> split_string_array;
    input_string.ParseIntoArray(split_string_array, TEXT(" "), true);
    ASSERT(split_string_array.Num() == 4);
    return FVector4(FCString::Atof(*split_string_array[0]), FCString::Atof(*split_string_array[1]), FCString::Atof(*split_string_array[2]), FCString::Atof(*split_string_array[3]));
}

Eigen::Matrix3f UrdfParser::parseInertiaMatrixNode(FXmlNode* inertia_node)
{
    Eigen::Matrix3f inertia_matrix;

    float ixx = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixx")));
    float ixy = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixy")));
    float ixz = FCString::Atof(*inertia_node->GetAttribute(TEXT("ixz")));
    float iyy = FCString::Atof(*inertia_node->GetAttribute(TEXT("iyy")));
    float iyz = FCString::Atof(*inertia_node->GetAttribute(TEXT("iyz")));
    float izz = FCString::Atof(*inertia_node->GetAttribute(TEXT("izz")));

    inertia_matrix(0, 0) = ixx;
    inertia_matrix(0, 1) = ixy;
    inertia_matrix(0, 2) = ixz;
    inertia_matrix(1, 0) = ixy;
    inertia_matrix(1, 1) = iyy;
    inertia_matrix(1, 2) = iyz;
    inertia_matrix(2, 0) = ixz;
    inertia_matrix(2, 1) = iyz;
    inertia_matrix(2, 2) = izz;

    ASSERT(!inertia_matrix.hasNaN());
    return inertia_matrix;
}

UrdfJointType UrdfParser::parseJointType(const FString& type)
{
    if (type.Equals(TEXT("revolute"))) {
        return REVOLUTE_TYPE;
    } else if (type.Equals(TEXT("continuous"))) {
        return CONTINUOUS_TYPE;
    } else if (type.Equals(TEXT("prismatic"))) {
        return PRISMATIC_TYPE;
    } else if (type.Equals(TEXT("fixed"))) {
        return FIXED_TYPE;
    } else if (type.Equals(TEXT("floating"))) {
        return FLOATING_TYPE;
    } else if (type.Equals(TEXT("planar"))) {
        return PLANAR_TYPE;
    } else {
        ASSERT(false);
        return UrdfJointType::INVALID_TYPE;
    }
}
