// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common_utils/CommonStructs.hpp"
#include "common_utils/UpdatableObject.hpp"
#include "common_utils/Common.hpp"
#include "common_utils/CommonStructs.hpp"
#include "common_utils/ImageCaptureBase.hpp"

#include <exception>
#include <string>

namespace RobotSim {

class RobotApiBase : public UpdatableObject {
public:
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() const = 0;
    virtual bool armDisarm(bool arm) = 0;
    virtual GeoPoint getHomeGeoPoint() const = 0;

    //default implementation so derived class doesn't have to call on UpdatableObject
    virtual void reset() override
    {
        UpdatableObject::reset();
    }
    virtual void update() override
    {
        UpdatableObject::update();
    }

    virtual void cancelLastTask()
    {
        //if derived class supports async task then override this method
    }
    virtual bool isReady(std::string& message) const
    {
        unused(message);
        return true;
    }

    //if vehicle supports it, call this method to send
    //kinematics and other info to somewhere (ex. log viewer, file, cloud etc)
    virtual void sendTelemetry(float last_interval = -1)
    {
        //no default action
        unused(last_interval);
    }

    //below APIs are used by FastPhysicsEngine
    virtual real_T getActuation(unsigned int actuator_index) const
    {
        unused(actuator_index);
        throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
    }
    virtual size_t getActuatorCount() const
    {
        throw VehicleCommandNotImplementedException("getActuatorCount API is not supported for this vehicle");
    }

    virtual void getStatusMessages(std::vector<std::string>& messages)
    {
        unused(messages);
        //default implementation
    }


    virtual ~RobotApiBase() = default;

    //exceptions
    class VehicleControllerException : public std::runtime_error {
    public:
        VehicleControllerException(const std::string& message)
            : runtime_error(message) {
        }
    };

    class VehicleCommandNotImplementedException : public VehicleControllerException {
    public:
        VehicleCommandNotImplementedException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };

    class VehicleMoveException : public VehicleControllerException {
    public:
        VehicleMoveException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };
};


} //namespace
