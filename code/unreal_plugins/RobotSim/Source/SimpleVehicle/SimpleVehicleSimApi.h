#pragma once

#include <functional>

#include "RobotBlueprintLib.h"
#include "RobotSimApiBase.h"
#include "physics/Kinematics.hpp"

#include "SimpleVehicleApi.h"
#include "SimpleVehiclePawn.h"

class SimpleVehicleSimApi : public RobotSimApiBase {
public:
    /**
     * @brief Construct a new Simple Vehicle Sim Api object
     *
     * @param params
     */
    SimpleVehicleSimApi(Params params);

    /**
     * @brief Destroy the Simple Vehicle Sim Api object
     *
     */
    virtual ~SimpleVehicleSimApi() = default;

    /**
     * @brief
     *
     */
    virtual void reset() override;

    /**
     * @brief
     *
     */
    virtual void update() override;

    /**
     * @brief
     *
     * @param reporter
     */
    virtual void reportState(RobotSim::StateReporter& reporter) override;

    /**
     * @brief
     *
     * @param is_header_line
     * @return std::string
     */
    virtual std::string getRecordFileLine(bool is_header_line) const;

    /**
     * @brief
     *
     * @param dt
     */
    virtual void updateRenderedState(float dt);

    /**
     * @brief
     *
     * @param dt
     */
    virtual void updateRendering(float dt);

    /**
     * @brief Get the Vehicle Api object
     *
     * @return SimpleVehicleApi*
     */
    SimpleVehicleApi* getVehicleApi()
    {
        return vehicle_api_.get();
    }

private:
    /**
     * @brief Create a Vehicle Api object
     *
     * @param pawn
     * @param home_geopoint
     */
    void createVehicleApi(ASimpleVehiclePawn* pawn, const RobotSim::GeoPoint& home_geopoint);

    std::unique_ptr<SimpleVehicleApi> vehicle_api_;
    std::vector<std::string> vehicle_api_messages_;
};
