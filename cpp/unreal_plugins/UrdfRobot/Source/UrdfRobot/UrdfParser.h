//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Math/Matrix.h>
#include <Math/Rotator.h>
#include <Math/Transform.h>
#include <Math/Vector.h>

class FXmlNode;

enum class UrdfGeometryType
{
    Invalid,
    Box,
    Sphere,
    Cylinder,
    Mesh
};

// enum values must match EJointType in UrdfJointComponent.h
enum class UrdfJointType
{
    Invalid,
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar
};

// enum values must match EJointControlType in UrdfJointComponent.h
enum class UrdfJointControlType
{
    NotActuated,
    Position,
    Velocity,
    PositionAndVelocity,
    Torque
};

// enum values must match EJointInterfaceType in UrdfJointComponent.h
enum class UrdfJointInterfaceType
{
    None,
    Set,
    AddTo
};

struct UrdfGeometryDesc
{
    UrdfGeometryType type_    = UrdfGeometryType::Invalid;
    std::vector<double> size_ = {1.0, 1.0, 1.0};
    double radius_            = 0.0;
    double length_            = 0.0;
    std::string filename_;
    double scale_             = 1.0;

    // custom SPEAR data
    std::string unreal_static_mesh_;
    double unreal_static_mesh_scale_ = 1.0;
};

struct UrdfMaterialDesc
{
    std::string name_;

    std::vector<double> color_ = {1.0, 1.0, 1.0, 1.0};
    std::string texture_;

    // custom SPEAR data
    std::string unreal_material_;

    // derived data
    bool is_reference_               = false;
    UrdfMaterialDesc* material_desc_ = nullptr;
};

struct UrdfInertialDesc
{
    std::vector<double> xyz_ = {0.0, 0.0, 0.0};
    std::vector<double> rpy_ = {0.0, 0.0, 0.0};
    double mass_             = 0.0;
    double ixx_              = 0.0;
    double iyy_              = 0.0;
    double izz_              = 0.0;
    double ixy_              = 0.0;
    double ixz_              = 0.0;
    double iyz_              = 0.0;
};

struct UrdfVisualDesc
{
    std::string name_;

    std::vector<double> xyz_ = {0.0, 0.0, 0.0};
    std::vector<double> rpy_ = {0.0, 0.0, 0.0};

    UrdfGeometryDesc geometry_desc_;
    UrdfMaterialDesc material_desc_;
    
    // derived data
    bool has_material_ = false;
};

struct UrdfCollisionDesc
{
    std::string name_;

    std::vector<double> xyz_ = {0.0, 0.0, 0.0};
    std::vector<double> rpy_ = {0.0, 0.0, 0.0};

    UrdfGeometryDesc geometry_desc_;
};

struct UrdfJointDesc;
struct UrdfLinkDesc
{
    std::string name_;

    UrdfInertialDesc inertial_desc_;
    std::vector<UrdfVisualDesc> visual_descs_;
    std::vector<UrdfCollisionDesc> collision_descs_;

    // custom SPEAR data
    bool simulate_physics_ = true;
    bool ignore_collisions_ = false;

    // derived data
    bool has_parent_              = false;
    bool parent_simulate_physics_ = false;
    std::vector<UrdfLinkDesc*> child_link_descs_;
    std::vector<UrdfJointDesc*> child_joint_descs_;
    std::vector<double> xyz_ = {0.0, 0.0, 0.0};
    std::vector<double> rpy_ = {0.0, 0.0, 0.0};
};

struct UrdfJointDesc
{
    std::string name_;
    UrdfJointType type_;

    std::vector<double> xyz_ = {0.0, 0.0, 0.0};
    std::vector<double> rpy_ = {0.0, 0.0, 0.0};

    std::string parent_;
    std::string child_;

    std::vector<double> axis_ = {1.0, 0.0, 0.0};

    // calibration
    double rising_  = 0.0;
    double falling_ = 0.0;

    // dynamics
    double damping_  = 0.0;
    double friction_ = 0.0;

    // limit
    double lower_    = 0.0;
    double upper_    = 0.0;
    double effort_   = 0.0;
    double velocity_ = 0.0;

    // mimic
    std::string joint_;
    double multiplier_ = 1.0;
    double offset_     = 0.0;

    // safety_controller
    double soft_lower_limit_ = 0.0;
    double soft_upper_limit_ = 0.0;
    double k_position_       = 0.0;
    double k_velocity_       = 0.0;

    // custom SPEAR data
    UrdfJointInterfaceType interface_type_ = UrdfJointInterfaceType::None;
    UrdfJointControlType control_type_     = UrdfJointControlType::NotActuated;
    double spring_                         = 0.0;
    bool parent_dominates_                 = false;
};

struct UrdfRobotDesc
{
    std::string name_;

    std::map<std::string, UrdfLinkDesc> link_descs_;
    std::map<std::string, UrdfJointDesc> joint_descs_;
    std::map<std::string, UrdfMaterialDesc> material_descs_;

    // derived data
    UrdfLinkDesc* root_link_desc_ = nullptr;
};

class URDFROBOT_API UrdfParser
{
public:
    static UrdfRobotDesc parse(const std::string& filename);

private:
    // parse urdf nodes
    static UrdfRobotDesc parseRobotNode(FXmlNode* robot_node);
    static UrdfLinkDesc parseLinkNode(FXmlNode* link_node);
    static UrdfJointDesc parseJointNode(FXmlNode* joint_node);
    static UrdfInertialDesc parseInertialNode(FXmlNode* inertial_node);
    static UrdfVisualDesc parseVisualNode(FXmlNode* visual_node);
    static UrdfCollisionDesc parseCollisionNode(FXmlNode* collision_node);
    static UrdfGeometryDesc parseGeometryNode(FXmlNode* geometry_node);
    static UrdfMaterialDesc parseMaterialNode(FXmlNode* material_node);

    // get attribute from a node, assert if the attribute is required and not found
    static std::string getAttribute(FXmlNode* node, const std::string& attribute, bool required);

    // parse strings into numerical types
    static std::vector<double> parseVector(const std::string& str, const std::vector<double>& default_value = {});
    static double parseDouble(const std::string& str, double default_value = 0.0);
    static bool parseBool(const std::string& str, bool default_value = false);
};
