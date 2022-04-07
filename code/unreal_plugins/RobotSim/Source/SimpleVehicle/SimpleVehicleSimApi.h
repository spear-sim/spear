#pragma once

#include <functional>

#include "RobotBlueprintLib.h"
#include "physics/Kinematics.hpp"
#include "RobotSimApiBase.h"

#include "SimpleVehicleApi.h"
#include "SimpleVehiclePawn.h"

class SimpleVehicleSimApi : public RobotSimApiBase
{
public:
    virtual ~SimpleVehicleSimApi() = default;

    SimpleVehicleSimApi(Params params);

    virtual void reset() override;
    virtual void update() override;
    virtual void reportState(RobotSim::StateReporter& reporter) override;

    virtual std::string getRecordFileLine(bool is_header_line) const;

    virtual void updateRenderedState(float dt);
    virtual void updateRendering(float dt);

    SimpleVehicleApi* getVehicleApi()
    {
        return vehicle_api_.get();
    }

private:
    std::unique_ptr<SimpleVehicleApi> vehicle_api_;
    std::vector<std::string> vehicle_api_messages_;

    void createVehicleApi(ASimpleVehiclePawn* pawn,
                          const RobotSim::GeoPoint& home_geopoint);
};
