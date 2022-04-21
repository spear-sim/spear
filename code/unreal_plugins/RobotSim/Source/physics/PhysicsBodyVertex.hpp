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
    class PhysicsBodyVertex : public UpdatableObject {

    public:
        /**
         * @brief Construct a new Physics Body Vertex object
         *
         */
        PhysicsBodyVertex()
        {
            // allow default constructor with later call for initialize
        }

        /**
         * @brief
         *
         * @return real_T
         */
        real_T getDragFactor() const
        {
            return drag_factor_;
        }

        /**
         * @brief
         *
         * @param val
         */
        void setDragFactor(real_T val)
        {
            drag_factor_ = val;
        }

        /**
         * @brief Construct a new Physics Body Vertex object
         *
         * @param position
         * @param normal
         * @param drag_factor
         */
        PhysicsBodyVertex(const Vector3r& position, const Vector3r& normal, real_T drag_factor = 0)
        {
            initialize(position, normal, drag_factor);
        }

        /**
         * @brief
         *
         * @param position
         * @param normal
         * @param drag_factor
         */
        void initialize(const Vector3r& position, const Vector3r& normal, real_T drag_factor = 0)
        {
            initial_position_ = position;
            initial_normal_ = normal;
            drag_factor_ = drag_factor;
        }

        //*** Start: UpdatableState implementation ***//

        /**
         * @brief
         *
         */
        virtual void reset() override
        {
            UpdatableObject::reset();

            position_ = initial_position_;
            normal_ = initial_normal_;

            current_wrench_ = Wrench::zero();
        }

        /**
         * @brief
         *
         */
        virtual void update() override
        {
            UpdatableObject::update();

            setWrench(current_wrench_);
        }
        //*** End: UpdatableState implementation ***//

        // getters, setters

        /**
         * @brief
         *
         * @return Vector3r
         */
        Vector3r getPosition() const
        {
            return position_;
        }

        /**
         * @brief brief description
         *
         * @param position
         */
        void setPosition(const Vector3r& position)
        {
            position_ = position;
        }

        /**
         * @brief
         *
         * @return Vector3r
         */
        Vector3r getNormal() const
        {
            return normal_;
        }

        /**
         * @brief
         *
         * @param normal
         */
        void setNormal(const Vector3r& normal)
        {
            normal_ = normal;
        }

        /**
         * @brief
         *
         * @return Wrench
         */
        Wrench getWrench() const
        {
            return current_wrench_;
        }

    protected:
        /**
         * @brief
         *
         * @param wrench
         */
        virtual void setWrench(Wrench& wrench)
        {
            unused(wrench);
            // derived class should override if this is force/torque
            // generating vertex
        }

    private:
        Vector3r initial_position_, position_;
        Vector3r initial_normal_, normal_;
        Wrench current_wrench_;
        real_T drag_factor_;
    };

} // namespace
