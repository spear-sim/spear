// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "PhysicsBody.hpp"
#include "PhysicsEngineBase.hpp"
#include "common_utils/ClockFactory.hpp"
#include "common_utils/Common.hpp"
#include "common_utils/ScheduledExecutor.hpp"
#include "common_utils/UpdatableContainer.hpp"
#include <functional>

namespace RobotSim {
    /**
     * @brief
     *
     */
    class World : public UpdatableContainer<UpdatableObject*> {
    public:
        /**
         * @brief Construct a new World object
         *
         * @param physics_engine
         */
        World(std::unique_ptr<PhysicsEngineBase> physics_engine) : physics_engine_(std::move(physics_engine))
        {
            World::clear();

            if (physics_engine) {
                physics_engine_->clear();
            }
        }

        // override updatable interface so we can synchronize physics engine
        //*** Start: UpdatableState implementation ***//
        virtual void reset() override
        {
            UpdatableContainer::reset();

            if (physics_engine_) {
                physics_engine_->reset();
            }
        }

        virtual void update() override
        {
            ClockFactory::get()->step();

            // First update our objects
            UpdatableContainer::update();

            // Now update kinematics state
            if (physics_engine_)
                physics_engine_->update();
        }

        virtual void reportState(StateReporter& reporter) override
        {
            reporter.writeValue("Sleep", 1.0f / executor_.getSleepTimeAvg());
            if (physics_engine_) {
                physics_engine_->reportState(reporter);
            }

            // Call base
            UpdatableContainer::reportState(reporter);
        }
        //*** End: UpdatableState implementation ***//

        /**
         * @brief Override membership modification methods so we can synchronize physics engine
         *
         */
        virtual void clear() override
        {
            if (physics_engine_)
                physics_engine_->clear();
            UpdatableContainer::clear();
        }

        /**
         * @brief
         *
         * @param member
         */
        virtual void insert(UpdatableObject* member) override
        {
            if (physics_engine_ && member->getPhysicsBody() != nullptr) {
                physics_engine_->insert(static_cast<PhysicsBody*>(member->getPhysicsBody()));
            }

            UpdatableContainer::insert(member);
        }

        /**
         * @brief
         *
         * @param member
         */
        virtual void erase_remove(UpdatableObject* member) override
        {
            if (physics_engine_ && member->getPhysicsBody() != nullptr) {
                physics_engine_->erase_remove(static_cast<PhysicsBody*>(member->getPhysicsBody()));
            }

            UpdatableContainer::erase_remove(member);
        }

        /**
         * @brief Async updater thread
         *
         * @param period
         */
        void startAsyncUpdator(uint64_t period)
        {
            // TODO: probably we shouldn't be passing around fixed period
            executor_.initialize(std::bind(&World::worldUpdatorAsync, this, std::placeholders::_1), period);
            executor_.start();
        }

        /**
         * @brief
         *
         */
        void stopAsyncUpdator()
        {
            executor_.stop();
        }

        /**
         * @brief
         *
         */
        void lock()
        {
            executor_.lock();
        }

        /**
         * @brief
         *
         */
        void unlock()
        {
            executor_.unlock();
        }

        /**
         * @brief Destroy the World object
         *
         */
        virtual ~World()
        {
            executor_.stop();
        }

        /**
         * @brief
         *
         * @param is_paused
         */
        void pause(bool is_paused)
        {
            executor_.pause(is_paused);
        }

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool isPaused() const
        {
            return executor_.isPaused();
        }

        /**
         * @brief
         *
         * @param seconds
         */
        void continueForTime(double seconds)
        {
            executor_.continueForTime(seconds);
        }

    private:
        /**
         * @brief
         *
         * @param dt_nanos
         * @return true
         * @return false
         */
        bool worldUpdatorAsync(uint64_t dt_nanos)
        {
            unused(dt_nanos);

            try {
                update();
            }
            catch (const std::exception& ex) {
                // Utils::DebugBreak();
                Utils::log(Utils::stringf("Exception occurred while updating world: %s", ex.what()), Utils::kLogLevelError);
            }
            catch (...) {
                // Utils::DebugBreak();
                Utils::log("Exception occurred while updating world", Utils::kLogLevelError);
            }

            return true;
        }

    private:
        std::unique_ptr<PhysicsEngineBase> physics_engine_ = nullptr;
        common_utils::ScheduledExecutor executor_;
    };

} // namespace
