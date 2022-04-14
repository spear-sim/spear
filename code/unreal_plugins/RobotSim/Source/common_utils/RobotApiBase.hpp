// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <exception>
#include <string>

#include "common_utils/Common.hpp"
#include "common_utils/CommonStructs.hpp"
#include "common_utils/ImageCaptureBase.hpp"
#include "common_utils/UpdatableObject.hpp"

namespace RobotSim {

    class RobotApiBase : public UpdatableObject {
    public:
        /**
         * @brief
         *
         * @param is_enabled
         */
        virtual void enableApiControl(bool is_enabled) = 0;

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        virtual bool isApiControlEnabled() const = 0;

        /**
         * @brief
         *
         * @param arm
         * @return true
         * @return false
         */
        virtual bool armDisarm(bool arm) = 0;

        /**
         * @brief
         *
         * @return GeoPoint
         */
        virtual GeoPoint getHomeGeoPoint() const = 0;

        // default implementation so derived class doesn't have to call on UpdatableObject

        /**
         * @brief
         *
         */
        virtual void reset() override
        {
            UpdatableObject::reset();
        }

        /**
         * @brief
         *
         */
        virtual void update() override
        {
            UpdatableObject::update();
        }

        /**
         * @brief
         *
         */
        virtual void cancelLastTask()
        {
            // if derived class supports async task then override this method
        }

        /**
         * @brief
         *
         * @param message
         * @return true
         * @return false
         */
        virtual bool isReady(std::string& message) const
        {
            unused(message);
            return true;
        }

        /**
         * @brief if vehicle supports it, call this method to send
         * kinematics and other info to somewhere (ex. log viewer, file, cloud etc)
         *
         * @param last_interval
         */
        virtual void sendTelemetry(float last_interval = -1)
        {
            // no default action
            unused(last_interval);
        }

        // below APIs are used by FastPhysicsEngine

        /**
         * @brief Get the Actuation object
         *
         * @param actuator_index
         * @return real_T
         */
        virtual real_T getActuation(unsigned int actuator_index) const
        {
            unused(actuator_index);
            throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
        }

        /**
         * @brief Get the Actuator Count object
         *
         * @return size_t
         */
        virtual size_t getActuatorCount() const
        {
            throw VehicleCommandNotImplementedException("getActuatorCount API is not supported for this vehicle");
        }

        /**
         * @brief
         *
         * @param messages
         */
        virtual void getStatusMessages(std::vector<std::string>& messages)
        {
            unused(messages);
            // default implementation
        }

        virtual ~RobotApiBase() = default;

        // exceptions
        /**
         * @brief
         *
         */
        class VehicleControllerException : public std::runtime_error {
        public:
            VehicleControllerException(const std::string& message) : runtime_error(message)
            {
            }
        };

        /**
         * @brief
         *
         */
        class VehicleCommandNotImplementedException : public VehicleControllerException {
        public:
            VehicleCommandNotImplementedException(const std::string& message) : VehicleControllerException(message)
            {
            }
        };

        /**
         * @brief
         *
         */
        class VehicleMoveException : public VehicleControllerException {
        public:
            VehicleMoveException(const std::string& message) : VehicleControllerException(message)
            {
            }
        };
    };

} // namespace
