// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "Environment.hpp"
#include "Kinematics.hpp"
#include "PhysicsBodyVertex.hpp"
#include "common_utils/Common.hpp"
#include "common_utils/CommonStructs.hpp"
#include "common_utils/UpdatableObject.hpp"
#include <exception>
#include <unordered_set>

namespace RobotSim {
    /**
     * @brief
     *
     */
    class PhysicsBody : public UpdatableObject {
    public: // interface
        /**
         * @brief
         *
         */
        virtual void kinematicsUpdated() = 0;

        /**
         * @brief Get the Restitution object
         *
         * @return real_T
         */
        virtual real_T getRestitution() const = 0;

        /**
         * @brief Get the Friction object
         *
         * @return real_T
         */
        virtual real_T getFriction() const = 0;

        /**
         * @brief
         * derived class may return covariant type.
         *
         * @return uint
         */
        virtual uint wrenchVertexCount() const
        {
            return 0;
        }

        /**
         * @brief
         *
         * @param index
         * @return PhysicsBodyVertex&
         */
        virtual PhysicsBodyVertex& getWrenchVertex(uint index)
        {
            unused(index);
            throw std::out_of_range("no physics vertex are available");
        }

        /**
         * @brief
         *
         * @param index
         * @return const PhysicsBodyVertex&
         */
        virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const
        {
            unused(index);
            throw std::out_of_range("no physics vertex are available");
        }

        /**
         * @brief
         *
         * @return uint
         */
        virtual uint dragVertexCount() const
        {
            return 0;
        }

        /**
         * @brief
         *
         * @param index
         * @return PhysicsBodyVertex&
         */
        virtual PhysicsBodyVertex& getDragVertex(uint index)
        {
            unused(index);
            throw std::out_of_range("no physics vertex are available");
        }

        /**
         * @brief
         *
         * @param index
         * @return const PhysicsBodyVertex&
         */
        virtual const PhysicsBodyVertex& getDragVertex(uint index) const
        {
            unused(index);
            throw std::out_of_range("no physics vertex are available");
        }

        /**
         * @brief
         *
         * @param collision_info
         */
        virtual void setCollisionInfo(const CollisionInfo& collision_info)
        {
            collision_info_ = collision_info;
        }

    public: // methods
        /**
         * @brief Construct a new Physics Body object
         *
         */
        PhysicsBody()
        {
            // allow default constructor with later call for initialize
        }

        /**
         * @brief Construct a new Physics Body object
         *
         * @param mass
         * @param inertia
         * @param initial_kinematic_state
         * @param environment
         */
        PhysicsBody(real_T mass, const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment)
        {
            initialize(mass, inertia, initial_kinematic_state, environment);
        }

        /**
         * @brief
         *
         * @param mass
         * @param inertia
         * @param initial_kinematic_state
         * @param environment
         */
        void initialize(real_T mass, const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment)
        {
            kinematics_.initialize(initial_kinematic_state);

            mass_ = mass;
            mass_inv_ = 1.0f / mass;
            inertia_ = inertia;
            inertia_inv_ = inertia_.inverse();
            environment_ = environment;
        }

        /**
         * @brief Enable physics body detection
         *
         * @return UpdatableObject*
         */
        virtual UpdatableObject* getPhysicsBody() override
        {
            return this;
        }

        //*** Start: UpdatableState implementation ***//

        /**
         * @brief
         *
         */
        virtual void reset() override
        {
            UpdatableObject::reset();

            kinematics_.reset();

            if (environment_)
                environment_->reset();
            wrench_ = Wrench::zero();
            collision_info_ = CollisionInfo();
            collision_response_ = CollisionResponse();
            grounded_ = false;

            // update individual vertices
            for (uint vertex_index = 0; vertex_index < wrenchVertexCount(); ++vertex_index) {
                getWrenchVertex(vertex_index).reset();
            }
            for (uint vertex_index = 0; vertex_index < dragVertexCount(); ++vertex_index) {
                getDragVertex(vertex_index).reset();
            }
        }

        /**
         * @brief
         *
         */
        virtual void update() override
        {
            UpdatableObject::update();

            // TODO: this is now being done in PawnSimApi::update. We need to re-think this sequence
            // environment_->setPosition(getKinematics().pose.position);
            // environment_->update();

            kinematics_.update();

            // update individual vertices - each vertex takes control signal as input and
            // produces force and thrust as output
            for (uint vertex_index = 0; vertex_index < wrenchVertexCount(); ++vertex_index) {
                getWrenchVertex(vertex_index).update();
            }
            for (uint vertex_index = 0; vertex_index < dragVertexCount(); ++vertex_index) {
                getDragVertex(vertex_index).update();
            }
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

            reporter.writeHeading("Kinematics");
            kinematics_.reportState(reporter);
        }
        //*** End: UpdatableState implementation ***//

        // getters

        /**
         * @brief
         *
         * @return real_T
         */
        real_T getMass() const
        {
            return mass_;
        }

        /**
         * @brief
         *
         * @return real_T
         */
        real_T getMassInv() const
        {
            return mass_inv_;
        }

        /**
         * @brief
         *
         * @return const Matrix3x3r&
         */
        const Matrix3x3r& getInertia() const
        {
            return inertia_;
        }

        /**
         * @brief
         *
         * @return const Matrix3x3r&
         */
        const Matrix3x3r& getInertiaInv() const
        {
            return inertia_inv_;
        }

        /**
         * @brief
         *
         * @return const Pose&
         */
        const Pose& getPose() const
        {
            return kinematics_.getPose();
        }

        /**
         * @brief
         *
         * @param pose
         */
        void setPose(const Pose& pose)
        {
            return kinematics_.setPose(pose);
        }

        /**
         * @brief
         *
         * @return const Twist&
         */
        const Twist& getTwist() const
        {
            return kinematics_.getTwist();
        }

        /**
         * @brief
         *
         * @param twist
         */
        void setTwist(const Twist& twist)
        {
            return kinematics_.setTwist(twist);
        }

        /**
         * @brief
         *
         * @return const Kinematics::State&
         */
        const Kinematics::State& getKinematics() const
        {
            return kinematics_.getState();
        }

        /**
         * @brief
         *
         * @param state
         */
        void setKinematics(const Kinematics::State& state)
        {
            if (VectorMath::hasNan(state.twist.linear)) {
                // Utils::DebugBreak();
                Utils::log("Linear velocity had NaN!", Utils::kLogLevelError);
            }

            kinematics_.setState(state);
        }

        /**
         * @brief
         *
         * @return const Kinematics::State&
         */
        const Kinematics::State& getInitialKinematics() const
        {
            return kinematics_.getInitialState();
        }

        /**
         * @brief
         *
         * @return const Environment&
         */
        const Environment& getEnvironment() const
        {
            return *environment_;
        }

        /**
         * @brief
         *
         * @return Environment&
         */
        Environment& getEnvironment()
        {
            return *environment_;
        }

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool hasEnvironment() const
        {
            return environment_ != nullptr;
        }

        /**
         * @brief
         *
         * @return const Wrench&
         */
        const Wrench& getWrench() const
        {
            return wrench_;
        }

        /**
         * @brief
         *
         * @param wrench
         */
        void setWrench(const Wrench& wrench)
        {
            wrench_ = wrench;
        }

        /**
         * @brief
         *
         * @return const CollisionInfo&
         */
        const CollisionInfo& getCollisionInfo() const
        {
            return collision_info_;
        }

        /**
         * @brief
         *
         * @return const CollisionResponse&
         */
        const CollisionResponse& getCollisionResponseInfo() const
        {
            return collision_response_;
        }

        /**
         * @brief
         *
         * @return CollisionResponse&
         */
        CollisionResponse& getCollisionResponseInfo()
        {
            return collision_response_;
        }

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool isGrounded() const
        {
            return grounded_;
        }

        /**
         * @brief
         *
         * @param grounded
         */
        void setGrounded(bool grounded)
        {
            grounded_ = grounded;
        }

    public:
        // for use in physics engine: //TODO: use getter/setter or friend method?
        TTimePoint last_kinematics_time;

    private:
        real_T mass_, mass_inv_;
        Matrix3x3r inertia_, inertia_inv_;

        Kinematics kinematics_;

        // force is in world frame but torque is not
        Wrench wrench_;

        CollisionInfo collision_info_;
        CollisionResponse collision_response_;

        bool grounded_ = false;

        Environment* environment_ = nullptr;
    };

} // namespace
