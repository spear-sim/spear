#pragma once

#include "CoreMinimal.h"
#include "UrdfOrigin.h"
#include "UrdfGeometry.h"
#include "UrdfMaterialSpecification.h"
#include "common_utils/VectorMath.hpp"

class ROBOTSIM_API UrdfLinkInertialSpecification
{
	public:
		UrdfOrigin Origin;
		float Mass;
		RobotSim::VectorMathf::Matrix3x3f Inertia;
};

class ROBOTSIM_API UrdfLinkVisualSpecification
{
	public:
		FString Name;
		UrdfOrigin Origin;
		UrdfGeometry* Geometry;
		FString MaterialName;
};

class ROBOTSIM_API UrdfLinkCollisionSpecification
{
	public:
		FString Name;
		UrdfOrigin Origin;
		UrdfGeometry* Geometry;
};

class ROBOTSIM_API UrdfLinkSpecification
{
	public:
		FString Name;
		UrdfLinkInertialSpecification* InertialSpecification = nullptr;
		UrdfLinkVisualSpecification* VisualSpecification = nullptr;
		UrdfLinkCollisionSpecification* CollisionSpecification = nullptr;

		UrdfLinkSpecification* ParentLink;
		TArray<TPair<UrdfLinkSpecification*, class UrdfJointSpecification*> > Children;
};