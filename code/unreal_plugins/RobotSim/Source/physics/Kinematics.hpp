// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common_utils/Common.hpp"
#include "common_utils/CommonStructs.hpp"
#include "common_utils/UpdatableObject.hpp"

namespace RobotSim {
    /**
     * @brief
     *
     */
    class Kinematics : public UpdatableObject {
    public:
        struct State {
            Pose pose;
            Twist twist;
            Accelerations accelerations;

            /**
             * @brief
             *
             * @return State
             */
            static State zero()
            {
                State zero_state;
                zero_state.pose.position = Vector3r::Zero();
                zero_state.pose.orientation = Quaternionr::Identity();
                zero_state.twist = Twist::zero();
                zero_state.accelerations = Accelerations::zero();

                return zero_state;
            }
        };

        /**
         * @brief Construct a new Kinematics object
         *
         * @param initial
         */
        Kinematics(const State& initial = State::zero())
        {
            initialize(initial);
        }

        /**
         * @brief
         *
         * @param initial
         */
        void initialize(const State& initial)
        {
            initial_ = initial;
        }

        //*** Start: UpdatableState implementation ***//

        /**
         * @brief
         *
         */
        virtual void reset() override
        {
            UpdatableObject::reset();

            current_ = initial_;
        }

        /**
         * @brief
         *
         */
        virtual void update() override
        {
            UpdatableObject::update();

            // nothing to do because next state should be updated
            // by physics engine. The reason is that final state
            // needs to take in to account state of other objects as well,
            // for example, if collision occurs
        }

        /**
         * @brief
         *
         * @param reporter
         */
        virtual void reportState(StateReporter& reporter) override
        {
            // call base
            UpdatableObject::reportState(reporter);

            reporter.writeValue("Position", current_.pose.position);
            reporter.writeValue("Orientation", current_.pose.orientation);
            reporter.writeValue("Lin-Vel", current_.twist.linear);
            reporter.writeValue("Lin-Accl", current_.accelerations.linear);
            reporter.writeValue("Ang-Vel", current_.twist.angular);
            reporter.writeValue("Ang-Accl", current_.accelerations.angular);
        }
        //*** End: UpdatableState implementation ***//

        /**
         * @brief
         *
         * @return const Pose&
         */
        const Pose& getPose() const
        {
            return current_.pose;
        }

        /**
         * @brief
         *
         * @param pose
         */
        void setPose(const Pose& pose)
        {
            current_.pose = pose;
        }

        /**
         * @brief
         *
         * @return const Twist&
         */
        const Twist& getTwist() const
        {
            return current_.twist;
        }

        /**
         * @brief 
         *
         * @param twist
         */
        void setTwist(const Twist& twist)
        {
            current_.twist = twist;
        }

        /**
         * @brief 
         * 
         * @return const State& 
         */
        const State& getState() const
        {
            return current_;
        }

        /**
         * @brief 
         * 
         * @param state 
         */
        void setState(const State& state)
        {
            current_ = state;
        }

        /**
         * @brief 
         * 
         * @return const State& 
         */
        const State& getInitialState() const
        {
            return initial_;
        }

    private: // fields
        State initial_;
        State current_;
    };

} // namespace
