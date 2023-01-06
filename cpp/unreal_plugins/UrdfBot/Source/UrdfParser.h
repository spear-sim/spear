//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>

#include <CoreMinimal.h>

class FXmlNode;

struct UrdfJointDesc;
struct UrdfLinkDesc;
struct UrdfMaterialDesc;

enum UrdfGeometryType
{
    INVALID_GEOMETRY = 0,
    BOX,
    SPHERE,
    CYLINDER,
    MESH
};

enum UrdfJointType
{
    // Some of these (such as FIXED) are already macros
    INVALID_TYPE = 0,
    REVOLUTE_TYPE,
    CONTINUOUS_TYPE,
    PRISMATIC_TYPE,
    FIXED_TYPE,
    FLOATING_TYPE,
    PLANAR_TYPE
};

struct UrdfGeometry
{
    UrdfGeometryType type_;
    FVector size_; // requried by BOX
    float radius_; // requried by SPHERE, CYLINDER
    float length_; // requried by CYLINDER
    std::string file_name_; // requried by MESH
};

struct UrdfLinkDesc
{
    std::string name_;

    // non-derived data (inertial, visual, etc)...
    FTransform origin_ = FTransform::Identity;
    float mass_;
    Eigen::Matrix3f inertia_;
    FTransform visual_origin_ = FTransform::Identity;
    UrdfGeometry visual_geometry_;
    FTransform collision_origin_ = FTransform::Identity;
    UrdfGeometry collision_geometry_;
    std::string material_name_;

    // derived data
    UrdfLinkDesc* parent_link_desc_ = nullptr;
    UrdfJointDesc* parent_joint_desc_ = nullptr;

    std::vector<UrdfLinkDesc*> child_link_descs_ = {};
    std::vector<UrdfJointDesc*> child_joint_descs_ = {};

    UrdfMaterialDesc* material_desc_ = nullptr;
};

struct UrdfJointDesc
{
    std::string name_;

    // non-derived data (origin, axis, etc)...
    UrdfJointType type_;
    FTransform origin_ = FTransform::Identity;
    std::string parent_link_name_;
    std::string child_link_name_;
    FVector axis_ = FVector(1, 0, 0);
    // dynamics
    float damping_ = 0;
    float friction_ = 0;
    //limits
    float lower_ = 0;
    float upper_ = 0;
    float effort_ = 0;
    float velocity_ = 0;

    // derived data
    UrdfLinkDesc* parent_link_desc_ = nullptr;
    UrdfLinkDesc* child_link_desc_ = nullptr;
};

// material node to describe unreal material that might shared by multiple links
struct UrdfMaterialDesc
{
    std::string name_;

    // non-derived data
    FVector4 color_;
    std::string file_name_;
};

struct UrdfRobotDesc
{
    // non-derived data
    std::map<std::string, UrdfLinkDesc> link_descs_ = {};
    std::map<std::string, UrdfJointDesc> joint_descs_ = {};
    std::map<std::string, UrdfMaterialDesc> material_descs_ = {};

    // derived data
    UrdfLinkDesc* root_link_desc_ = nullptr;
};

class UrdfParser
{
public:
    static UrdfRobotDesc parse(const std::string& file_name);

private:
    // parse urdf nodes
    static UrdfRobotDesc parseRobotNode(FXmlNode* robot_node);
    static UrdfLinkDesc parseLinkNode(FXmlNode* link_node);
    static UrdfJointDesc parseJointNode(FXmlNode* joint_node);
    static UrdfMaterialDesc parseMaterialNode(FXmlNode* material_node);
    static UrdfGeometry parseGeometryNode(FXmlNode* geometry_node);

    // parse link node component
    static void parseLinkInertialNode(FXmlNode* inertial_node, UrdfLinkDesc& link_desc);
    static void parseLinkVisualNode(FXmlNode* visual_node, UrdfLinkDesc& link_desc);
    static void parseLinkCollisionNode(FXmlNode* collision_node, UrdfLinkDesc& link_desc);

    // parse basic parameters
    static FVector parseVector(const FString& string);
    static FVector4 parseVector4(const FString& string);
    static FQuat parseRotation(const FString& string);
    static Eigen::Matrix3f parseInertiaMatrixNode(FXmlNode* inertia_node);
    static UrdfJointType parseJointType(const FString& type);
};
