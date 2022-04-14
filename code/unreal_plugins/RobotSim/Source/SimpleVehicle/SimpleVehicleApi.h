#pragma once
#include "common_utils/CommonStructs.hpp"
#include "common_utils/RobotApiBase.hpp"
#include "common_utils/VectorMath.hpp"

#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"

class AUrdfBotPawn;

class SimpleVehicleApi : public RobotSim::RobotApiBase {
public:

    /**
     * @brief 
     * 
     * @param bIsApiControlEnabled 
     */
    virtual void enableApiControl(bool bIsApiControlEnabled) override;

    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    virtual bool isApiControlEnabled() const override;

    // TODO remove
    virtual bool armDisarm(bool arm) override;
    
    // TODO remove
    virtual RobotSim::GeoPoint getHomeGeoPoint() const override;

private:
    bool bIsApiControlEnabled = false;
    RobotSim::GeoPoint mHomeGeoPoint;
};
