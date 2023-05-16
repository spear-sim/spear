//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "OpenBot/OpenBotWheel.h"

#include <iostream>

#include "CoreUtils/Config.h"

UOpenBotWheel::UOpenBotWheel()
{
    std::cout << "[SPEAR | OpenBotWheel.cpp] UOpenBotWheel::UOpenBotWheel" << std::endl;

    if (!Config::s_initialized_) {
        return;
    }

    bAffectedByHandbrake = true;
    
    // we need to set this for external torques to be used to drive the vehicle
    ExternalTorqueCombineMethod = ETorqueCombineMethod::Override;
    
    // not needed if we are using our own torques via SetDriveTorques()
    //AxleType = EAxleType::Rear;

    // this needs to be non-zero to be able to steer
    MaxSteerAngle = Config::get<float>("OPENBOT.OPENBOT_WHEEL.STEER_ANGLE");
    
    MaxBrakeTorque     = Config::get<float>("OPENBOT.OPENBOT_WHEEL.MAX_BRAKE_TORQUE");
    MaxHandBrakeTorque = Config::get<float>("OPENBOT.OPENBOT_WHEEL.MAX_HAND_BRAKE_TORQUE");

    WheelMass     = Config::get<float>("OPENBOT.OPENBOT_WHEEL.MASS");
    WheelRadius   = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SHAPE_RADIUS");
    WheelWidth    = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SHAPE_WIDTH");

    SuspensionMaxRaise     = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_MAX_RAISE");
    SuspensionMaxDrop      = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_MAX_DROP");
    SuspensionDampingRatio = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_DAMPING_RATIO");
}

UOpenBotWheel::~UOpenBotWheel()
{
    std::cout << "[SPEAR | OpenBotWheel.cpp] UOpenBotWheel::~UOpenBotWheel" << std::endl;
}
