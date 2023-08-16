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
    Mesh,
};

enum class UrdfJointType
{
    Invalid,
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
};

enum class UrdfJointControlType
{
    Invalid,
    Position,
    Velocity,
    Torque,
};

struct UrdfGeometryDesc
{
    UrdfGeometryType type_ = UrdfGeometryType::Invalid;
    FVector size_          = FVector::ZeroVector;
    double radius_         = 0.0f;
    double length_         = 0.0f;
    std::string filename_;
    double scale_          = 1.0f;
};

struct UrdfMaterialDesc
{
    std::string name_;

    FVector4 color_ = FVector4(1.0, 1.0, 1.0, 1.0);
    std::string texture_;

    // derived data
    bool is_reference_               = false;
    UrdfMaterialDesc* material_desc_ = nullptr;
};

struct UrdfInertialDesc
{
    FVector xyz_     = FVector::ZeroVector;
    FVector rpy_     = FVector::ZeroVector;
    double mass_     = 0.0f;
    FMatrix inertia_ = FMatrix(EForceInit::ForceInitToZero);
};

struct UrdfVisualDesc
{
    std::string name_;

    FVector xyz_ = FVector::ZeroVector;
    FVector rpy_ = FVector::ZeroVector;

    UrdfGeometryDesc geometry_desc_;
    UrdfMaterialDesc material_desc_;
    
    // derived data
    bool has_material_ = false;
};

struct UrdfCollisionDesc
{
    std::string name_;

    FVector xyz_ = FVector::ZeroVector;
    FVector rpy_ = FVector::ZeroVector;

    UrdfGeometryDesc geometry_desc_;
};

struct UrdfSpearUnrealAssetDesc
{
    std::string name_;

    FVector xyz_ = FVector::ZeroVector;
    FVector rpy_ = FVector::ZeroVector;

    std::string path_;
};

struct UrdfJointDesc;
struct UrdfLinkDesc
{
    std::string name_;

    UrdfInertialDesc inertial_desc_;
    std::vector<UrdfVisualDesc> visual_descs_;
    std::vector<UrdfCollisionDesc> collision_descs_;

    // custom SPEAR data
    std::vector<UrdfSpearUnrealAssetDesc> spear_unreal_asset_descs_;

    // derived data
    bool has_parent_                  = false;
    UrdfJointDesc* parent_joint_desc_ = nullptr;
    std::vector<UrdfLinkDesc*> child_link_descs_;
    std::vector<UrdfJointDesc*> child_joint_descs_;
};

struct UrdfJointDesc
{
    std::string name_;
    UrdfJointType type_;

    FVector xyz_ = FVector::ZeroVector;
    FVector rpy_ = FVector::ZeroVector;

    std::string parent_;
    std::string child_;

    FVector axis_ = FVector(1.0, 0.0, 0.0);

    // calibration
    double rising_  = 0.0f;
    double falling_ = 0.0f;

    // dynamics
    double damping_  = 0.0f;
    double friction_ = 0.0f;

    // limit
    double lower_    = 0.0f;
    double upper_    = 0.0f;
    double effort_   = 0.0f;
    double velocity_ = 0.0f;

    // mimic
    std::string joint_;
    double multiplier_ = 1.0f;
    double offset_     = 0.0f;

    // safety_controller
    double soft_lower_limit_ = 0.0f;
    double soft_upper_limit_ = 0.0f;
    double k_position_       = 0.0f;
    double k_velocity_       = 0.0f;

    // custom SPEAR data
    UrdfJointControlType control_type_ = UrdfJointControlType::Invalid;
    double spring_ = 0.0f;

    // derived data
    UrdfLinkDesc* parent_link_desc_ = nullptr;
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

class UrdfParser
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
    static UrdfSpearUnrealAssetDesc parseSpearUnrealAssetNode(FXmlNode* spear_unreal_asset_node);
    static UrdfGeometryDesc parseGeometryNode(FXmlNode* geometry_node);
    static UrdfMaterialDesc parseMaterialNode(FXmlNode* material_node);

    static std::string getAttribute(FXmlNode* node, const std::string& attribute, bool required);

    // parse basic parameters
    static FVector parseVector(const std::string& str, const FVector& default_value = FVector::ZeroVector);
    static FVector4 parseVector4(const std::string& str, const FVector4& default_value = FVector4(0.0, 0.0, 0.0, 0.0));
    static double parseDouble(const std::string& str, double default_value = 0.0);
};
