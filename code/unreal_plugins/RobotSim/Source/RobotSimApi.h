#pragma once

#include <functional>

#include "UrdfBot/UrdfBotPawn.h"
#include "UrdfBot/RobotApi.h"
#include "RobotBlueprintLib.h"
#include "physics/Kinematics.hpp"
#include "RobotSimApiBase.h"
class RobotSimApi : public RobotSimApiBase
{
public:
	virtual ~RobotSimApi() = default;

	RobotSimApi(Params params);

	virtual void reset() override;
	virtual void update() override;
	virtual void reportState(RobotSim::StateReporter& reporter) override;

	virtual std::string getRecordFileLine(bool is_header_line) const;

	virtual void updateRenderedState(float dt);
	virtual void updateRendering(float dt);

	RobotApi* getVehicleApi()
	{
		return vehicle_api_.get();
	}

private:
	std::unique_ptr<RobotApi> vehicle_api_;
	std::vector<std::string> vehicle_api_messages_;

	void createVehicleApi(AUrdfBotPawn* pawn, const RobotSim::GeoPoint& home_geopoint);
};