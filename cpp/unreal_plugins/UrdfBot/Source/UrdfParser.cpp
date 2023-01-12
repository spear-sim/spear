//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfParser.h"

#include <XmlFile.h>
#include <XmlNode.h>
#include <XmlParser.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/StdUtils.h"

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

    const TArray<FXmlNode*>& child_nodes = geometry_node->GetChildrenNodes();
    ASSERT(child_nodes.Num() == 1);
    FXmlNode* node = child_nodes[0];
    const FString& tag = node->GetTag();

    if (tag.Equals(TEXT("box"))) {
        geometry.type_ = UrdfGeometryType::Box;
        geometry.size_ = parseVector(node->GetAttribute("size"));
    } else if (tag.Equals(TEXT("cylinder"))) {
        geometry.type_   = UrdfGeometryType::Cylinder;
        geometry.length_ = FCString::Atof(*node->GetAttribute("length"));
        geometry.radius_ = FCString::Atof(*node->GetAttribute("radius"));
    } else if (tag.Equals(TEXT("sphere"))) {
        geometry.type_   = UrdfGeometryType::Sphere;
        geometry.radius_ = FCString::Atof(*node->GetAttribute("radius"));
    } else if (tag.Equals(TEXT("mesh"))) {
        geometry.type_     = UrdfGeometryType::Mesh;
        geometry.filename_ = TCHAR_TO_UTF8(*node->GetAttribute("filename"));
        FString scale      = node->GetAttribute("scale");
        if (!scale.IsEmpty()) {
            geometry.scale_ = FCString::Atof(*scale);
        }
    } else {
        ASSERT(false);
    }

    return geometry;
}

UrdfMaterialDesc UrdfParser::parseMaterialNode(FXmlNode* material_node)
{
    UrdfMaterialDesc material_desc;

    material_desc.name_         = TCHAR_TO_UTF8(*material_node->GetAttribute(TEXT("name")));
    material_desc.is_reference_ = true;

    bool has_color_node   = false;
    bool has_texture_node = false;

    for (auto& node : material_node->GetChildrenNodes()) {
        const FString& tag = node->GetTag();
        if (tag.Equals(TEXT("color"))) {
            ASSERT(!has_color_node);
            has_color_node              = true;
            material_desc.color_        = parseVector4(node->GetAttribute(TEXT("rgba")));
            material_desc.is_reference_ = false;
        } else if (tag.Equals(TEXT("texture"))) {
            ASSERT(!has_texture_node);
            has_texture_node            = true;
            material_desc.texture_      = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("filename")));
            material_desc.is_reference_ = false;
        } else {
            ASSERT(false);
        }
    }
    
    // material node must be either have non-empty name_ or valid color or texture value
    ASSERT(material_desc.name_ != "" || !material_desc.is_reference_);

    return material_desc;
}

UrdfInertialDesc UrdfParser::parseInertialNode(FXmlNode* inertial_node)
{
    UrdfInertialDesc inertial_desc_;
    
    bool has_origin_node  = false;
    bool has_mass_node    = false;
    bool has_inertia_node = false;

    for (auto& node : inertial_node->GetChildrenNodes()) {
        const FString& tag = node->GetTag();
        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            inertial_desc_.origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            inertial_desc_.origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("mass"))) {
            ASSERT(!has_mass_node);
            has_mass_node        = true;
            inertial_desc_.mass_ = FCString::Atof(*node->GetAttribute(TEXT("value")));
        } else if (tag.Equals(TEXT("inertia"))) {
            ASSERT(!has_inertia_node);
            has_inertia_node = true;
            inertial_desc_.inertia_.M[0][0] = FCString::Atof(*node->GetAttribute(TEXT("ixx")));
            inertial_desc_.inertia_.M[0][1] = FCString::Atof(*node->GetAttribute(TEXT("ixy")));
            inertial_desc_.inertia_.M[0][2] = FCString::Atof(*node->GetAttribute(TEXT("ixz")));
            inertial_desc_.inertia_.M[1][0] = FCString::Atof(*node->GetAttribute(TEXT("ixy")));
            inertial_desc_.inertia_.M[1][1] = FCString::Atof(*node->GetAttribute(TEXT("iyy")));
            inertial_desc_.inertia_.M[1][2] = FCString::Atof(*node->GetAttribute(TEXT("iyz")));
            inertial_desc_.inertia_.M[2][0] = FCString::Atof(*node->GetAttribute(TEXT("ixz")));
            inertial_desc_.inertia_.M[2][1] = FCString::Atof(*node->GetAttribute(TEXT("iyz")));
            inertial_desc_.inertia_.M[2][2] = FCString::Atof(*node->GetAttribute(TEXT("izz")));
        } else {
            ASSERT(false);
        }
    }

    ASSERT(has_mass_node);
    ASSERT(has_inertia_node);

    return inertial_desc_;
}

UrdfVisualDesc UrdfParser::parseVisualNode(FXmlNode* visual_node)
{
    UrdfVisualDesc visual_desc_;

    visual_desc_.name_ = TCHAR_TO_UTF8(*visual_node->GetAttribute(TEXT("name")));

    bool has_origin_node   = false;
    bool has_geometry_node = false;
    bool has_material_node = false;

    for (auto& node : visual_node->GetChildrenNodes()) {
        const FString& tag = node->GetTag();
        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            visual_desc_.origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            visual_desc_.origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("geometry"))) {
            ASSERT(!has_geometry_node);
            has_geometry_node           = true;
            visual_desc_.geometry_desc_ = parseGeometryNode(node);
        } else if (tag.Equals(TEXT("material"))) {
            ASSERT(!has_material_node);
            has_material_node           = true;
            visual_desc_.has_material_  = true;
            visual_desc_.material_desc_ = parseMaterialNode(node);
        } else {
            ASSERT(false);
        }
    }

    ASSERT(has_geometry_node);

    return visual_desc_;
}

UrdfCollisionDesc UrdfParser::parseCollisionNode(FXmlNode* collision_node)
{
    UrdfCollisionDesc collision_desc;

    collision_desc.name_ = TCHAR_TO_UTF8(*collision_node->GetAttribute(TEXT("name")));

    bool has_origin_node   = false;
    bool has_geometry_node = false;

    for (auto& node : collision_node->GetChildrenNodes()) {
        const FString& tag = node->GetTag();
        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            collision_desc.origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            collision_desc.origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("geometry"))) {
            ASSERT(!has_geometry_node);
            has_geometry_node             = true;
            collision_desc.geometry_desc_ = parseGeometryNode(node);
        } else {
            ASSERT(false);
        }
    }

    ASSERT(has_geometry_node);

    return collision_desc;
}

UrdfLinkDesc UrdfParser::parseLinkNode(FXmlNode* link_node)
{
    UrdfLinkDesc link_desc;
    link_desc.name_ = TCHAR_TO_UTF8(*link_node->GetAttribute(TEXT("name")));

    bool has_inertia_node = false;

    for (auto& child_node : link_node->GetChildrenNodes()) {
        const FString& tag = child_node->GetTag();
        if (tag.Equals(TEXT("inertial"))) {
            ASSERT(!has_inertia_node);
            has_inertia_node         = true;
            link_desc.inertial_desc_ = parseInertialNode(child_node);
        } else if (tag.Equals(TEXT("visual"))) {
            link_desc.visual_descs_.push_back(parseVisualNode(child_node));
        } else if (tag.Equals(TEXT("collision"))) {
            link_desc.collision_descs_.push_back(parseCollisionNode(child_node));
        } else {
            ASSERT(false);
        }
    }

    return link_desc;
}

UrdfJointDesc UrdfParser::parseJointNode(FXmlNode* joint_node)
{
    UrdfJointDesc joint_desc;

    joint_desc.name_ = TCHAR_TO_UTF8(*joint_node->GetAttribute(TEXT("name")));

    const FString& type = joint_node->GetAttribute(TEXT("type"));
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

    bool has_origin_node            = false;
    bool has_parent_node            = false;
    bool has_child_node             = false;
    bool has_axis_node              = false;
    bool has_calibration_node       = false;
    bool has_dynamics_node          = false;
    bool has_limit_node             = false;
    bool has_mimic_node             = false;
    bool has_safety_controller_node = false;

    for (auto& node : joint_node->GetChildrenNodes()) {
        const FString& tag = node->GetTag();
        if (tag.Equals(TEXT("origin"))) {
            ASSERT(!has_origin_node);
            has_origin_node = true;
            joint_desc.origin_.SetLocation(parseVector(node->GetAttribute("xyz")));
            joint_desc.origin_.SetRotation(parseRotation(node->GetAttribute("rpy")));
        } else if (tag.Equals(TEXT("parent"))) {
            ASSERT(!has_parent_node);
            has_parent_node    = true;
            joint_desc.parent_ = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("link")));
        } else if (tag.Equals(TEXT("child"))) {
            ASSERT(!has_child_node);
            has_child_node    = true;
            joint_desc.child_ = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("link")));
        } else if (tag.Equals(TEXT("axis"))) {
            ASSERT(!has_axis_node);
            has_axis_node    = true;
            joint_desc.axis_ = parseVector(node->GetAttribute(TEXT("xyz")));
        } else if (tag.Equals(TEXT("calibration"))) {
            ASSERT(!has_calibration_node);
            has_calibration_node = true;
            FString rising = node->GetAttribute(TEXT("rising"));
            if (!rising.IsEmpty()) {
                ASSERT(joint_desc.calibration_type_ == CalibrationType::Invalid);
                joint_desc.calibration_type_ = CalibrationType::Rising;
                joint_desc.rising_           = FCString::Atof(*rising);
            }
            FString falling = node->GetAttribute(TEXT("falling"));
            if (!falling.IsEmpty()) {
                ASSERT(joint_desc.calibration_type_ == CalibrationType::Invalid);
                joint_desc.calibration_type_ = CalibrationType::Falling;
                joint_desc.falling_          = FCString::Atof(*falling);
            }
        } else if (tag.Equals(TEXT("dynamics"))) {
            ASSERT(!has_dynamics_node);
            has_dynamics_node = true;
            joint_desc.damping_  = FCString::Atof(*node->GetAttribute(TEXT("damping")));
            joint_desc.friction_ = FCString::Atof(*node->GetAttribute(TEXT("friction")));
        } else if (tag.Equals(TEXT("limit"))) {
            ASSERT(!has_limit_node);
            has_limit_node = true;
            joint_desc.lower_    = FCString::Atof(*node->GetAttribute(TEXT("lower")));
            joint_desc.upper_    = FCString::Atof(*node->GetAttribute(TEXT("upper")));
            joint_desc.effort_   = FCString::Atof(*node->GetAttribute(TEXT("effort")));
            joint_desc.velocity_ = FCString::Atof(*node->GetAttribute(TEXT("velocity")));
        } else if (tag.Equals(TEXT("mimic"))) {
            ASSERT(!has_mimic_node);
            has_mimic_node     = true;
            joint_desc.joint_  = TCHAR_TO_UTF8(*node->GetAttribute(TEXT("joint")));
            FString multiplier = node->GetAttribute(TEXT("multiplier"));
            if (!multiplier.IsEmpty()) {
                joint_desc.multiplier_ = FCString::Atof(*multiplier);
            }
            joint_desc.offset_ = FCString::Atof(*node->GetAttribute(TEXT("offset")));
        } else if (tag.Equals(TEXT("safety_controller"))) {
            ASSERT(!has_safety_controller_node);
            has_safety_controller_node   = true;
            joint_desc.soft_lower_limit_ = FCString::Atof(*node->GetAttribute(TEXT("soft_lower_limit")));
            joint_desc.soft_upper_limit_ = FCString::Atof(*node->GetAttribute(TEXT("soft_upper_limit")));
            joint_desc.k_position_       = FCString::Atof(*node->GetAttribute(TEXT("k_position")));
            joint_desc.k_velocity_       = FCString::Atof(*node->GetAttribute(TEXT("k_velocity")));
        } else {
            ASSERT(false);
        }
    }

    ASSERT(has_parent_node);
    ASSERT(has_child_node);

    // revolute and prismatic joints require limit
    ASSERT(!(joint_desc.type_ == UrdfJointType::Revolute || joint_desc.type_ == UrdfJointType::Prismatic) || has_limit_node);

    return joint_desc;
}

UrdfRobotDesc UrdfParser::parseRobotNode(FXmlNode* robot_node)
{
    UrdfRobotDesc robot_desc;
    
    robot_desc.name_ = TCHAR_TO_UTF8(*robot_node->GetAttribute(TEXT("name")));

    // parse all top-level URDF nodes into their own dictionaries
    for (auto& child_node : robot_node->GetChildrenNodes()) {
        const FString& tag = child_node->GetTag();

        if (tag.Equals(TEXT("link"))) {
            UrdfLinkDesc link_desc = parseLinkNode(child_node);
            ASSERT(!StdUtils::containsKey(robot_desc.link_descs_, link_desc.name_));
            robot_desc.link_descs_[link_desc.name_] = std::move(link_desc);
        } else if (tag.Equals(TEXT("joint"))) {
            UrdfJointDesc joint_desc = parseJointNode(child_node);
            ASSERT(!StdUtils::containsKey(robot_desc.joint_descs_, joint_desc.name_));
            robot_desc.joint_descs_[joint_desc.name_] = std::move(joint_desc);
        } else if (tag.Equals(TEXT("material"))) {
            UrdfMaterialDesc material_desc = parseMaterialNode(child_node);
            ASSERT(material_desc.name_ != "");
            ASSERT(!material_desc.is_reference_);
            ASSERT(!StdUtils::containsKey(robot_desc.material_descs_, material_desc.name_));
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
