#pragma once

#include <stdexcept>

#include "CoreMinimal.h"
#include "UrdfLinkSpecification.h"
#include "UrdfJointSpecification.h"
#include "UrdfForceSpecification.h"
#include "UrdfMaterialSpecification.h"
#include "UrdfCameraSpecification.h"
#include "Runtime/XmlParser/Public/XmlFile.h"
#include "Runtime/XmlParser/Public/XmlNode.h"
#include "Runtime/XmlParser/Public/XmlParser.h"
#include "Runtime/XmlParser/Public/FastXml.h"

class UrdfParser
{
public:
    void Parse(FString fileName);
    TMap<FString, UrdfLinkSpecification*> GetLinks();
    TMap<FString, UrdfJointSpecification*> GetJoints();
    TMap<FString, UrdfForceSpecification*> GetForces();
    TMap<FString, UrdfMaterialSpecification*> GetMaterials();

private:
    UrdfLinkSpecification* ParseLinkSpecification(FXmlNode* linkNode);
    UrdfJointSpecification* ParseJointSpecification(FXmlNode* jointNode);
    UrdfForceSpecification* ParseForceSpecification(FXmlNode* forceNode);
    UrdfMaterialSpecification*
    ParseMaterialSpecification(FXmlNode* materialNode);

    // Link parsing
    UrdfLinkInertialSpecification*
    ParseLinkInertialSpecification(FXmlNode* inertialNode);
    UrdfLinkVisualSpecification*
    ParseLinkVisualSpecification(FXmlNode* visualNode);
    UrdfLinkCollisionSpecification*
    ParseLinkCollisionSpecification(FXmlNode* collisionNode);
    UrdfGeometry* ParseUrdfGeometrySpecification(FXmlNode* geometryNode);

    // Joint parsing
    UrdfJointCalibrationSpecification*
    ParseJointCalibrationSpecification(FXmlNode* calibrationNode);
    UrdfJointDynamicsSpecification
    ParseJointDynamicsSpecification(FXmlNode* dynamicsNode);
    UrdfJointLimitSpecification*
    ParseJointLimitSpecification(FXmlNode* limitNode);
    UrdfJointMimicSpecification*
    ParseJointMimicSpecification(FXmlNode* mimicNode);
    UrdfJointSafetyControllerSpecification*
    ParseJointSafetyControllerSpecification(FXmlNode* safetyControllerNode);

    // Force parsing
    UrdfAngularForceSpecification*
    ParseAngularForceSpecification(FXmlNode* angularForceNode);
    UrdfLinearForceSpecification*
    ParseLinearForceSpecification(FXmlNode* linearForceNode);

    // General parsing
    UrdfOrigin ParseUrdfOrigin(FXmlNode* originNode);
    FVector ParseFVectorFromAttributeString(const FString& string);
    FRotator ParseFRotatorFromAttributeString(const FString& string);
    FVector4 ParseFVector4FromAttributeString(const FString& string);
    RobotSim::VectorMathf::Matrix3x3f
    ParseInertiaMatrixFromAttributes(FXmlNode* inertiaNode);
    UrdfGeometryType GeometryTypeFromString(FString typeString);

    TMap<FString, UrdfLinkSpecification*> links_;
    TMap<FString, UrdfJointSpecification*> joints_;
    TMap<FString, UrdfForceSpecification*> forces_;
    TMap<FString, UrdfMaterialSpecification*> materials_;
};