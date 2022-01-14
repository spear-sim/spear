#pragma once
#include "SimpleVehicleApi.h"

void SimpleVehicleApi::enableApiControl(bool is_enabled)
{
    this->bIsApiControlEnabled = is_enabled;
}

bool SimpleVehicleApi::isApiControlEnabled() const
{
    return this->bIsApiControlEnabled;
}

// TODO: Figure out what this does.
bool SimpleVehicleApi::armDisarm(bool arm)
{
    unused(arm);
    return true;
}

// TODO: Figure out what is this.
RobotSim::GeoPoint SimpleVehicleApi::getHomeGeoPoint() const
{
    return this->mHomeGeoPoint;
}
