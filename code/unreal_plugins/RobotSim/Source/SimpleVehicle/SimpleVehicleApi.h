#pragma once
#include "common_utils/VectorMath.hpp"
#include "common_utils/RobotApiBase.hpp"
#include "common_utils/CommonStructs.hpp"

#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"

class AUrdfBotPawn;

class SimpleVehicleApi : public RobotSim::RobotApiBase
{
public:
    virtual void enableApiControl(bool bIsApiControlEnabled) override;
    virtual bool isApiControlEnabled() const override;
    // TODO remove
    virtual bool armDisarm(bool arm) override;
    // TODO remove
    virtual RobotSim::GeoPoint getHomeGeoPoint() const override;

private:
    bool bIsApiControlEnabled = false;
    RobotSim::GeoPoint mHomeGeoPoint;
};