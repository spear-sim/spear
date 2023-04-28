//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "OpenBot/OpenBotWheel.h"

#include <iostream>

#include <ChaosVehicleWheel.h>

#include "CoreUtils/Config.h"

UOpenBotWheel::UOpenBotWheel()
{
    std::cout << "[SPEAR | OpenBotWheel.cpp] UOpenBotWheel::UOpenBotWheel" << std::endl;

    if (!Config::s_initialized_) {
        return;
    }

    ConstructorHelpers::FObjectFinder<UStaticMesh> collision_mesh(TEXT("/Engine/EngineMeshes/Cylinder"));
    ASSERT(collision_mesh.Succeeded());

    CollisionMesh              = collision_mesh.Object;

    ////bAffectedByHandbrake       = Config::get<bool>("OPENBOT.OPENBOT_WHEEL.AFFECTED_BY_HANDBRAKE");

    //SuspensionDampingRatio?
    //DampingRate                = Config::get<float>("OPENBOT.OPENBOT_WHEEL.DAMPING_RATE");
    
    //LatStiffMaxLoad            = Config::get<float>("OPENBOT.OPENBOT_WHEEL.LAT_STIFF_MAX_LOAD");
    //LatStiffValue              = Config::get<float>("OPENBOT.OPENBOT_WHEEL.LAT_STIFF_VALUE");
    //LongStiffValue             = Config::get<float>("OPENBOT.OPENBOT_WHEEL.LONG_STIFF_VALUE");
    //Mass                       = Config::get<float>("OPENBOT.OPENBOT_WHEEL.MASS");

    ////MaxBrakeTorque             = Config::get<float>("OPENBOT.OPENBOT_WHEEL.MAX_BRAKE_TORQUE");
    ////MaxHandBrakeTorque         = Config::get<float>("OPENBOT.OPENBOT_WHEEL.MAX_HAND_BRAKE_TORQUE");

    // WheelRadius, WheelWidth?
    //WheelRadius                = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SHAPE_RADIUS");
    //WheelWidth                 = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SHAPE_WIDTH");
    //ShapeRadius                = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SHAPE_RADIUS");
    //ShapeWidth                 = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SHAPE_WIDTH");

    // MaxSteerAngle?
    //SteerAngle                 = Config::get<float>("OPENBOT.OPENBOT_WHEEL.STEER_ANGLE");

    // Need suspension related params to hold the wheels and body together
    //SuspensionMaxRaise         = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_MAX_RAISE");
    //SuspensionMaxDrop          = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_MAX_DROP");

    // Changed to a FVector
    //SuspensionForceOffset      = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_FORCE_OFFSET");

    //SuspensionNaturalFrequency = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_NATURAL_FREQUENCY");

    //SuspensionDampingRatio     = Config::get<float>("OPENBOT.OPENBOT_WHEEL.SUSPENSION_DAMPING_RATIO");
    
    // ESweepType has only 2 options - Simple and Complex
    //SweepType                  = ESweepType::SimpleAndComplex;

    //bAutoAdjustCollisionSize   = true; // Set to true if you want to scale the wheels manually 
}

UOpenBotWheel::~UOpenBotWheel()
{
    std::cout << "[SPEAR | OpenBotWheel.cpp] UOpenBotWheel::~UOpenBotWheel" << std::endl;
}
