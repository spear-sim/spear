//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfParser.h"

#include <string>
#include <vector>

#include <XmlFile.h>
#include <XmlNode.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"

const bool REQUIRED = true;
const bool OPTIONAL = false;

UrdfRobotDesc UrdfParser::parse(const std::string& filename)
{
    FXmlFile file;
    bool file_loaded = file.LoadFile(Unreal::toFString(filename));
    SP_ASSERT(file_loaded);

    // required
    FXmlNode* robot_node = file.GetRootNode();
    SP_ASSERT(robot_node);
    SP_ASSERT(robot_node->GetTag().Equals(Unreal::toFString("robot")));

    return parseRobotNode(robot_node);
}

UrdfRobotDesc UrdfParser::parseRobotNode(FXmlNode* robot_node)
{
    UrdfRobotDesc robot_desc;

    robot_desc.name_ = getAttribute(robot_node, "name", REQUIRED);

    // parse all top-level URDF nodes into their own dictionaries, don't use auto& because GetChildrenNodes() returns an array of pointers
    for (auto child_node : robot_node->GetChildrenNodes()) {
        const FString& tag = child_node->GetTag();

        if (tag.Equals(Unreal::toFString("link"))) {
            UrdfLinkDesc link_desc = parseLinkNode(child_node);
            SP_ASSERT(!Std::containsKey(robot_desc.link_descs_, link_desc.name_)); // name must be unique
            robot_desc.link_descs_[link_desc.name_] = std::move(link_desc);

        } else if (tag.Equals(Unreal::toFString("joint"))) {
            UrdfJointDesc joint_desc = parseJointNode(child_node);
            SP_ASSERT(!Std::containsKey(robot_desc.joint_descs_, joint_desc.name_)); // name must be unique
            robot_desc.joint_descs_[joint_desc.name_] = std::move(joint_desc);

        } else if (tag.Equals(Unreal::toFString("material"))) {
            UrdfMaterialDesc material_desc = parseMaterialNode(child_node);
            SP_ASSERT(material_desc.name_ != "");                                          // must have a name
            SP_ASSERT(!material_desc.is_reference_);                                       // must not be a reference
            SP_ASSERT(!Std::containsKey(robot_desc.material_descs_, material_desc.name_)); // name must be unique
            robot_desc.material_descs_[material_desc.name_] = std::move(material_desc);

        } else {
            SP_ASSERT(false);
        }
    }

    // for a robot with N links, there must be N-1 joints
    SP_ASSERT(robot_desc.joint_descs_.size() + 1 == robot_desc.link_descs_.size());

    // for each joint, update its parent and child links
    for (auto& joint_desc_pair : robot_desc.joint_descs_) {
        UrdfJointDesc* joint_desc = &(joint_desc_pair.second);
        SP_ASSERT(joint_desc);

        UrdfLinkDesc* parent_link_desc = &(robot_desc.link_descs_.at(joint_desc->parent_));
        UrdfLinkDesc* child_link_desc = &(robot_desc.link_descs_.at(joint_desc->child_));
        SP_ASSERT(parent_link_desc); // each joint must have a valid parent
        SP_ASSERT(child_link_desc);  // each joint must have a valid child

        parent_link_desc->child_link_descs_.push_back(child_link_desc);
        parent_link_desc->child_joint_descs_.push_back(joint_desc);

        SP_ASSERT(!child_link_desc->has_parent_); // each link must have at most one parent
        child_link_desc->has_parent_ = true;
        child_link_desc->parent_simulate_physics_ = parent_link_desc->simulate_physics_;

        // a child link's reference frame is defined as the parent joint's reference frame
        child_link_desc->xyz_ = joint_desc->xyz_;
        child_link_desc->rpy_ = joint_desc->rpy_;
    }

    // update pointers for root node and material node
    for (auto& link_desc_pair : robot_desc.link_descs_) {
        UrdfLinkDesc& link_desc = link_desc_pair.second;

        // for each material inside a link, if it refers to another material, update the reference
        for (auto& visual_desc_ : link_desc.visual_descs_) {
            UrdfMaterialDesc& material_desc = visual_desc_.material_desc_;

            if (visual_desc_.has_material_ && material_desc.is_reference_) {
                UrdfMaterialDesc* target_material_desc = &(robot_desc.material_descs_.at(material_desc.name_));
                SP_ASSERT(target_material_desc); // each material reference must have a valid target
                material_desc.material_desc_ = target_material_desc;
            }
        }

        // find the root link
        if (!link_desc.has_parent_) {
            SP_ASSERT(!robot_desc.root_link_desc_); // a robot must have only one root link
            robot_desc.root_link_desc_ = &(link_desc);
        }
    }

    // a robot must have only one root link
    SP_ASSERT(robot_desc.root_link_desc_);

    return robot_desc;
}

UrdfLinkDesc UrdfParser::parseLinkNode(FXmlNode* link_node)
{
    UrdfLinkDesc link_desc;

    link_desc.name_ = getAttribute(link_node, "name", REQUIRED);

    // optional
    FXmlNode* inertial_node = link_node->FindChildNode(Unreal::toFString("inertial"));
    if (inertial_node) {
        link_desc.inertial_desc_ = parseInertialNode(inertial_node);
    }

    for (auto& child_node : link_node->GetChildrenNodes()) {
        const FString& tag = child_node->GetTag();
        if (tag.Equals(Unreal::toFString("visual"))) {
            // optional, can have multiple
            link_desc.visual_descs_.push_back(parseVisualNode(child_node));
        } else if (tag.Equals(Unreal::toFString("collision"))) {
            // optional, can have multiple
            link_desc.collision_descs_.push_back(parseCollisionNode(child_node));
        }
    }

    // optional
    FXmlNode* spear_link_node = link_node->FindChildNode(Unreal::toFString("spear"));
    if (spear_link_node) {
        // optional
        FXmlNode* simulate_physics_node = spear_link_node->FindChildNode(Unreal::toFString("simulate_physics"));
        if (simulate_physics_node) {
            link_desc.simulate_physics_ = parseBool(getAttribute(simulate_physics_node, "value", REQUIRED), true);
        }

        // optional
        FXmlNode* ignore_collisions_node = spear_link_node->FindChildNode(Unreal::toFString("ignore_collisions"));
        if (ignore_collisions_node) {
            link_desc.ignore_collisions_ = parseBool(getAttribute(ignore_collisions_node, "value", REQUIRED), false);
        }
    }

    return link_desc;
}

UrdfJointDesc UrdfParser::parseJointNode(FXmlNode* joint_node)
{
    UrdfJointDesc joint_desc;

    joint_desc.name_ = getAttribute(joint_node, "name", REQUIRED);

    std::string type = getAttribute(joint_node, "type", REQUIRED);
    if (type == "revolute") {
        joint_desc.type_ = UrdfJointType::Revolute;
    } else if (type == "continuous") {
        joint_desc.type_ = UrdfJointType::Continuous;
    } else if (type == "prismatic") {
        joint_desc.type_ = UrdfJointType::Prismatic;
    } else if (type == "fixed") {
        joint_desc.type_ = UrdfJointType::Fixed;
    } else if (type == "floating") {
        joint_desc.type_ = UrdfJointType::Floating;
    } else if (type == "planar") {
        joint_desc.type_ = UrdfJointType::Planar;
    } else {
        SP_ASSERT(false);
    }

    // optional
    FXmlNode* origin_node = joint_node->FindChildNode(Unreal::toFString("origin"));
    if (origin_node) {
        joint_desc.xyz_ = parseVector(getAttribute(origin_node, "xyz", OPTIONAL), {0.0, 0.0, 0.0});
        joint_desc.rpy_ = parseVector(getAttribute(origin_node, "rpy", OPTIONAL), {0.0, 0.0, 0.0});
    }

    // required
    FXmlNode* parent_node = joint_node->FindChildNode(Unreal::toFString("parent"));
    SP_ASSERT(parent_node);
    joint_desc.parent_ = getAttribute(parent_node, "link", REQUIRED); // not documented if the attribute is required, so we assume it is

    // required
    FXmlNode* child_node = joint_node->FindChildNode(Unreal::toFString("child"));
    SP_ASSERT(child_node);
    joint_desc.child_ = getAttribute(child_node, "link", REQUIRED); // not documented if the attribute is required, so we assume it is

    // optional
    FXmlNode* axis_node = joint_node->FindChildNode(Unreal::toFString("axis"));
    if (axis_node) {
        joint_desc.axis_ = parseVector(getAttribute(axis_node, "xyz", REQUIRED), {1.0, 0.0, 0.0});
    }

    // optional
    FXmlNode* calibration_node = joint_node->FindChildNode(Unreal::toFString("calibration"));
    if (calibration_node) {
        // default values are not documented, so we assume 0.0
        joint_desc.rising_ = parseDouble(getAttribute(calibration_node, "rising", OPTIONAL), 0.0);
        joint_desc.falling_ = parseDouble(getAttribute(calibration_node, "falling", OPTIONAL), 0.0);
    }

    // optional
    FXmlNode* dynamics_node = joint_node->FindChildNode(Unreal::toFString("dynamics"));
    if (dynamics_node) {
        joint_desc.damping_ = parseDouble(getAttribute(dynamics_node, "damping", OPTIONAL), 0.0);
        joint_desc.friction_ = parseDouble(getAttribute(dynamics_node, "friction", OPTIONAL), 0.0);
    }

    // required for revolute and prismatic, optional otherwise
    FXmlNode* limit_node = joint_node->FindChildNode(Unreal::toFString("limit"));
    if (joint_desc.type_ == UrdfJointType::Revolute || joint_desc.type_ == UrdfJointType::Prismatic) {
        SP_ASSERT(limit_node);
    }
    if (limit_node) {
        joint_desc.lower_ = parseDouble(getAttribute(limit_node, "lower", OPTIONAL), 0.0);
        joint_desc.upper_ = parseDouble(getAttribute(limit_node, "upper", OPTIONAL), 0.0);
        joint_desc.effort_ = parseDouble(getAttribute(limit_node, "effort", REQUIRED));
        joint_desc.velocity_ = parseDouble(getAttribute(limit_node, "velocity", REQUIRED));
    }

    // optional
    FXmlNode* mimic_node = joint_node->FindChildNode(Unreal::toFString("mimic"));
    if (mimic_node) {
        joint_desc.joint_ = getAttribute(mimic_node, "joint", REQUIRED);
        joint_desc.multiplier_ = parseDouble(getAttribute(mimic_node, "multiplier", REQUIRED));
        joint_desc.offset_ = parseDouble(getAttribute(mimic_node, "offset", REQUIRED));
    }

    // optional
    FXmlNode* safety_controller_node = joint_node->FindChildNode(Unreal::toFString("safety_controller"));
    if (safety_controller_node) {
        joint_desc.soft_lower_limit_ = parseDouble(getAttribute(safety_controller_node, "soft_lower_limit", OPTIONAL), 0.0);
        joint_desc.soft_upper_limit_ = parseDouble(getAttribute(safety_controller_node, "soft_upper_limit", OPTIONAL), 0.0);
        joint_desc.k_position_       = parseDouble(getAttribute(safety_controller_node, "k_position", OPTIONAL), 0.0);
        joint_desc.k_velocity_       = parseDouble(getAttribute(safety_controller_node, "offset", REQUIRED));
    }

    // optional
    FXmlNode* spear_joint_node = joint_node->FindChildNode(Unreal::toFString("spear"));
    if (spear_joint_node) {
        // optional
        FXmlNode* spear_dynamics_node = spear_joint_node->FindChildNode(Unreal::toFString("dynamics"));
        if (spear_dynamics_node) {
            std::string control_type = getAttribute(spear_dynamics_node, "control_type", OPTIONAL);
            if (control_type == "position") {
                joint_desc.control_type_ = UrdfJointControlType::Position;
            } else if (control_type == "velocity") {
                joint_desc.control_type_ = UrdfJointControlType::Velocity;
            } else if (control_type == "position_and_velocity") {
                joint_desc.control_type_ = UrdfJointControlType::PositionAndVelocity;
            } else if (control_type == "torque") {
                joint_desc.control_type_ = UrdfJointControlType::Torque;
            } else if (control_type == "") {
                joint_desc.control_type_ = UrdfJointControlType::NotActuated;
            } else {
                SP_ASSERT(false);
            }

            std::string interface_type = getAttribute(spear_dynamics_node, "interface_type", OPTIONAL);
            if (interface_type == "set") {
                joint_desc.interface_type_ = UrdfJointInterfaceType::Set;
            } else if (interface_type == "add_to") {
                joint_desc.interface_type_ = UrdfJointInterfaceType::AddTo;
            } else if (interface_type == "") {
                joint_desc.interface_type_ = UrdfJointInterfaceType::NoInterface;
            } else {
                SP_ASSERT(false);
            }

            joint_desc.spring_ = parseDouble(getAttribute(spear_dynamics_node, "spring", OPTIONAL), 0.0);

            joint_desc.parent_dominates_ = parseBool(getAttribute(spear_dynamics_node, "parent_dominates", OPTIONAL), false);
        }
    }

    return joint_desc;
}

UrdfInertialDesc UrdfParser::parseInertialNode(FXmlNode* inertial_node)
{
    UrdfInertialDesc inertial_desc;

    // optional
    FXmlNode* origin_node = inertial_node->FindChildNode(Unreal::toFString("origin"));
    if (origin_node) {
        inertial_desc.xyz_ = parseVector(getAttribute(origin_node, "xyz", OPTIONAL), {0.0, 0.0, 0.0});
        inertial_desc.rpy_ = parseVector(getAttribute(origin_node, "rpy", OPTIONAL), {0.0, 0.0, 0.0});
    }

    // not documented if the node or attributes are required, so we assume that both are
    FXmlNode* mass_node = inertial_node->FindChildNode(Unreal::toFString("mass"));
    SP_ASSERT(mass_node);
    inertial_desc.mass_ = parseDouble(getAttribute(mass_node, "value", REQUIRED));

    // not documented if the node or attributes are required, so we assume that both are
    FXmlNode* inertia_node = inertial_node->FindChildNode(Unreal::toFString("inertia"));
    SP_ASSERT(inertia_node);
    inertial_desc.ixx_ = parseDouble(getAttribute(inertia_node, "ixx", REQUIRED));
    inertial_desc.iyy_ = parseDouble(getAttribute(inertia_node, "iyy", REQUIRED));
    inertial_desc.izz_ = parseDouble(getAttribute(inertia_node, "izz", REQUIRED));
    inertial_desc.ixy_ = parseDouble(getAttribute(inertia_node, "ixy", REQUIRED));
    inertial_desc.ixz_ = parseDouble(getAttribute(inertia_node, "ixz", REQUIRED));
    inertial_desc.iyz_ = parseDouble(getAttribute(inertia_node, "iyz", REQUIRED));

    return inertial_desc;
}

UrdfVisualDesc UrdfParser::parseVisualNode(FXmlNode* visual_node)
{
    UrdfVisualDesc visual_desc;

    visual_desc.name_ = getAttribute(visual_node, "name", OPTIONAL);

    // optional
    FXmlNode* origin_node = visual_node->FindChildNode(Unreal::toFString("origin"));
    if (origin_node) {
        visual_desc.xyz_ = parseVector(getAttribute(origin_node, "xyz", OPTIONAL), {0.0, 0.0, 0.0});
        visual_desc.rpy_ = parseVector(getAttribute(origin_node, "rpy", OPTIONAL), {0.0, 0.0, 0.0});
    }

    // required
    FXmlNode* geometry_node = visual_node->FindChildNode(Unreal::toFString("geometry"));
    SP_ASSERT(geometry_node);
    visual_desc.geometry_desc_ = parseGeometryNode(geometry_node);

    // optional
    FXmlNode* material_node = visual_node->FindChildNode(Unreal::toFString("material"));
    if (material_node) {
        visual_desc.has_material_  = true;
        visual_desc.material_desc_ = parseMaterialNode(material_node);
    }

    return visual_desc;
}

UrdfCollisionDesc UrdfParser::parseCollisionNode(FXmlNode* collision_node)
{
    UrdfCollisionDesc collision_desc;

    collision_desc.name_ = getAttribute(collision_node, "name", OPTIONAL);

    // optional
    FXmlNode* origin_node = collision_node->FindChildNode(Unreal::toFString("origin"));
    if (origin_node) {
        collision_desc.xyz_ = parseVector(getAttribute(origin_node, "xyz", OPTIONAL), {0.0, 0.0, 0.0});
        collision_desc.rpy_ = parseVector(getAttribute(origin_node, "rpy", OPTIONAL), {0.0, 0.0, 0.0});
    }

    // required
    FXmlNode* geometry_node = collision_node->FindChildNode(Unreal::toFString("geometry"));
    SP_ASSERT(geometry_node);
    collision_desc.geometry_desc_ = parseGeometryNode(geometry_node);

    return collision_desc;
}

UrdfGeometryDesc UrdfParser::parseGeometryNode(FXmlNode* geometry_node)
{
    UrdfGeometryDesc geometry_desc;

    // optional
    FXmlNode* box_node = geometry_node->FindChildNode(Unreal::toFString("box"));
    if (box_node) {
        SP_ASSERT(geometry_desc.type_ == UrdfGeometryType::Invalid);
        geometry_desc.type_ = UrdfGeometryType::Box;
        geometry_desc.size_ = parseVector(getAttribute(box_node, "size", REQUIRED)); // not documented if the attribute is required, so we assume it is
    }

    // optional
    FXmlNode* cylinder_node = geometry_node->FindChildNode(Unreal::toFString("cylinder"));
    if (cylinder_node) {
        SP_ASSERT(geometry_desc.type_ == UrdfGeometryType::Invalid);
        geometry_desc.type_   = UrdfGeometryType::Cylinder;
        geometry_desc.length_ = parseDouble(getAttribute(cylinder_node, "length", REQUIRED)); // not documented if the attribute is required, so we assume it is
        geometry_desc.radius_ = parseDouble(getAttribute(cylinder_node, "radius", REQUIRED)); // not documented if the attribute is required, so we assume it is
    }

    // optional
    FXmlNode* sphere_node = geometry_node->FindChildNode(Unreal::toFString("sphere"));
    if (sphere_node) {
        SP_ASSERT(geometry_desc.type_ == UrdfGeometryType::Invalid);
        geometry_desc.type_   = UrdfGeometryType::Sphere;
        geometry_desc.radius_ = parseDouble(getAttribute(sphere_node, "radius", REQUIRED)); // not documented if the attribute is required, so we assume it is
    }

    // optional
    FXmlNode* mesh_node = geometry_node->FindChildNode(Unreal::toFString("mesh"));
    if (mesh_node) {
        geometry_desc.type_     = UrdfGeometryType::Mesh;
        geometry_desc.filename_ = getAttribute(mesh_node, "filename", REQUIRED);
        geometry_desc.scale_    = parseDouble(getAttribute(mesh_node, "scale", OPTIONAL), 1.0);
    }

    // optional
    FXmlNode* spear_gometry_node = geometry_node->FindChildNode(Unreal::toFString("spear"));
    if (spear_gometry_node) {
        // required
        FXmlNode* unreal_static_mesh_node = spear_gometry_node->FindChildNode(Unreal::toFString("unreal_static_mesh"));
        SP_ASSERT(unreal_static_mesh_node);
        geometry_desc.type_  = UrdfGeometryType::Mesh;
        geometry_desc.unreal_static_mesh_ = getAttribute(unreal_static_mesh_node, "path", REQUIRED);
        geometry_desc.unreal_static_mesh_scale_ = parseDouble(getAttribute(unreal_static_mesh_node, "scale", OPTIONAL), 1.0);
    }

    SP_ASSERT(geometry_desc.type_ != UrdfGeometryType::Invalid);

    return geometry_desc;
}

UrdfMaterialDesc UrdfParser::parseMaterialNode(FXmlNode* material_node)
{
    UrdfMaterialDesc material_desc;


    // not documented if the attribute is required, but we assume it is not, because link-level materials do not need to be referenced from anywhere else
    material_desc.name_ = getAttribute(material_node, "name", OPTIONAL);

    // assume that the material is a reference unless we find a color or texture node
    material_desc.is_reference_ = true;

    // optional
    FXmlNode* color_node = material_node->FindChildNode(Unreal::toFString("color"));
    if (color_node) {
        material_desc.color_        = parseVector(getAttribute(color_node, "rgba", REQUIRED)); // not documented if the attribute is required, so we assume it is
        material_desc.is_reference_ = false;
    }

    // optional
    FXmlNode* texture_node = material_node->FindChildNode(Unreal::toFString("texture"));
    if (texture_node) {
        material_desc.texture_      = getAttribute(texture_node, "filename", REQUIRED); // not documented if the attribute is required, so we assume it is
        material_desc.is_reference_ = false;
    }

    // optional
    FXmlNode* spear_material_node = material_node->FindChildNode(Unreal::toFString("spear"));
    if (spear_material_node) {
        // required
        FXmlNode* unreal_material_node = spear_material_node->FindChildNode(Unreal::toFString("unreal_material"));
        SP_ASSERT(unreal_material_node);
        material_desc.unreal_material_ = getAttribute(unreal_material_node, "path", REQUIRED);
        material_desc.is_reference_ = false;
    }

    // if the material is a reference, then it must have a valid name
    if (material_desc.is_reference_) {
        SP_ASSERT(material_desc.name_ != "");
    }

    return material_desc;
}

std::string UrdfParser::getAttribute(FXmlNode* node, const std::string& tag, bool required) {
    SP_ASSERT(node);
    bool found = false;
    std::string val = "";
    for (auto& attribute : node->GetAttributes()) {
        if (tag == Unreal::toStdString(attribute.GetTag())) {
            SP_ASSERT(!found);
            found = true;
            val = Unreal::toStdString(attribute.GetValue());
        }
    }
    if (required) {
        SP_ASSERT(found);
    }
    return val;
}

std::vector<double> UrdfParser::parseVector(const std::string& str, const std::vector<double>& default_value)
{
    std::vector<double> val;
    if (str == "") {
        val = default_value;
    } else {
        for (auto& token : Std::tokenize(str, " ")) {
            val.push_back(std::stod(token));
        }
    }
    return val;
}

double UrdfParser::parseDouble(const std::string& str, double default_value)
{
    double val;    
    if (str == "") {
        val = default_value;
    } else {
        val = std::stod(str);
    }
    return val;
}

bool UrdfParser::parseBool(const std::string& str, bool default_value)
{
    std::string str_lower = Std::toLower(str);
    bool val;
    if (str == "") {
        val = default_value;
    } else if (str_lower == "true") {
        val = true;
    } else if (str_lower == "false") {
        val = false;
    } else {
        SP_ASSERT(false);
        val = false;
    }
    return val;
}
