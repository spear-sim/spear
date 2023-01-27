//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

class FXmlNode;

struct UrdfJointDesc;

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

enum class CalibrationType
{
    Invalid,
    Rising,
    Falling,
};

struct UrdfGeometryDesc
{
    UrdfGeometryType type_ = UrdfGeometryType::Invalid;
    FVector size_          = FVector::ZeroVector;
    float radius_          = 0.0f;
    float length_          = 0.0f;
    std::string filename_;
    float scale_           = 1.0f;
};

struct UrdfMaterialDesc
{
    std::string name_;

    FVector4 color_;
    std::string texture_;

    // derived data
    bool is_reference_               = false;
    UrdfMaterialDesc* material_desc_ = nullptr;
};

struct UrdfInertialDesc
{
    FTransform origin_ = FTransform::Identity;
    float mass_        = 0.0f;
    FMatrix inertia_   = FMatrix(EForceInit::ForceInitToZero);
};

struct UrdfVisualDesc
{
    std::string name_;

    FTransform origin_ = FTransform::Identity;
    UrdfGeometryDesc geometry_desc_;
    UrdfMaterialDesc material_desc_;
    
    // derived data
    bool has_material_ = false;
};

struct UrdfCollisionDesc
{
    std::string name_;

    FTransform origin_ = FTransform::Identity;
    UrdfGeometryDesc geometry_desc_;
};

struct UrdfLinkDesc
{
    std::string name_;

    UrdfInertialDesc inertial_desc_;
    std::vector<UrdfVisualDesc> visual_descs_;
    std::vector<UrdfCollisionDesc> collision_descs_;

    // derived data
    bool has_parent_ = false;
    std::vector<UrdfLinkDesc*> child_link_descs_;
    std::vector<UrdfJointDesc*> child_joint_descs_;
};

struct UrdfJointDesc
{
    std::string name_;
    UrdfJointType type_;

    FTransform origin_ = FTransform::Identity;
    std::string parent_;
    std::string child_;
    FVector axis_      = FVector(1.0f, 0.0f, 0.0f);

    // dynamics
    float damping_  = 0.0f;
    float friction_ = 0.0f;

    // calibration
    CalibrationType calibration_type_ = CalibrationType::Invalid;
    float rising_                     = 0.0f;
    float falling_                    = 0.0f;

    // limits
    float lower_    = 0.0f;
    float upper_    = 0.0f;
    float effort_   = 0.0f;
    float velocity_ = 0.0f;

    // mimic
    std::string joint_;
    float multiplier_ = 1.0f;
    float offset_     = 0.0f;

    // safety_controller
    float soft_lower_limit_ = 0.0f;
    float soft_upper_limit_ = 0.0f;
    float k_position_       = 0.0f;
    float k_velocity_       = 0.0f;
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
    static UrdfRobotDesc parse(const std::string& file_name);

private:
    // parse urdf nodes
    static UrdfGeometryDesc parseGeometryNode(FXmlNode* geometry_node);
    static UrdfMaterialDesc parseMaterialNode(FXmlNode* material_node);
    static UrdfInertialDesc parseInertialNode(FXmlNode* inertial_node);
    static UrdfVisualDesc parseVisualNode(FXmlNode* visual_node);
    static UrdfCollisionDesc parseCollisionNode(FXmlNode* collision_node);
    static UrdfLinkDesc parseLinkNode(FXmlNode* link_node);
    static UrdfJointDesc parseJointNode(FXmlNode* joint_node);
    static UrdfRobotDesc parseRobotNode(FXmlNode* robot_node);

    // parse basic parameters
    static FVector parseVector(const FString& string);
    static FVector4 parseVector4(const FString& string);
    static FQuat parseRotation(const FString& string);
};
