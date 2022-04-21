// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "PhysicsBody.hpp"
#include "PhysicsEngineBase.hpp"
#include "World.hpp"
#include "common_utils/Common.hpp"
#include "common_utils/StateReporterWrapper.hpp"
#include "common_utils/UpdatableContainer.hpp"

namespace RobotSim {
    /**
     * @brief
     *
     */
    class PhysicsWorld {
    public:
        /**
         * @brief Construct a new Physics World object
         *
         * @param physics_engine
         * @param bodies
         * @param update_period_nanos
         * @param state_reporter_enabled
         * @param start_async_updator
         */
        PhysicsWorld(std::unique_ptr<PhysicsEngineBase> physics_engine, const std::vector<UpdatableObject*>& bodies,
                     uint64_t update_period_nanos = 3000000LL, bool state_reporter_enabled = false,
                     bool start_async_updator = true) : world_(std::move(physics_engine))
        {
            enableStateReport(state_reporter_enabled);
            update_period_nanos_ = update_period_nanos;
            initializeWorld(bodies, start_async_updator);
        }

        /**
         * @brief
         *
         */
        void lock()
        {
            world_.lock();
        }

        /**
         * @brief
         *
         */
        void unlock()
        {
            world_.unlock();
        }

        /**
         * @brief
         *
         */
        void reset()
        {
            lock();
            world_.reset();
            unlock();
        }

        /**
         * @brief Get the Update Period Nanos object
         *
         * @return uint64_t
         */
        uint64_t getUpdatePeriodNanos() const
        {
            return update_period_nanos_;
        }

        /**
         * @brief
         *
         */
        void startAsyncUpdator()
        {
            world_.startAsyncUpdator(update_period_nanos_);
        }

        /**
         * @brief
         *
         */
        void stopAsyncUpdator()
        {
            world_.stopAsyncUpdator();
        }

        /**
         * @brief
         *
         * @param is_enabled
         */
        void enableStateReport(bool is_enabled)
        {
            reporter_.setEnable(is_enabled);
        }

        /**
         * @brief
         *
         */
        void updateStateReport()
        {
            if (reporter_.canReport()) {
                reporter_.clearReport();
                world_.reportState(*reporter_.getReporter());
            }
        }

        /**
         * @brief Get the Debug Report object
         *
         * @return std::string
         */
        std::string getDebugReport()
        {
            return reporter_.getOutput();
        }

        /**
         * @brief
         *
         * @param is_paused
         */
        void pause(bool is_paused)
        {
            world_.pause(is_paused);
        }

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool isPaused() const
        {
            return world_.isPaused();
        }

        /**
         * @brief
         *
         * @param seconds
         */
        void continueForTime(double seconds)
        {
            world_.continueForTime(seconds);
        }

    private:
        /**
         * @brief
         *
         * @param bodies
         * @param start_async_updator
         */
        void initializeWorld(const std::vector<UpdatableObject*>& bodies, bool start_async_updator)
        {
            reporter_.initialize(false);
            world_.insert(&reporter_);

            for (size_t bi = 0; bi < bodies.size(); bi++) {
                world_.insert(bodies.at(bi));
            }

            world_.reset();

            if (start_async_updator) {
                world_.startAsyncUpdator(update_period_nanos_);
            }
        }

    private:
        std::vector<UpdatableObject*> bodies_;
        StateReporterWrapper reporter_;
        World world_;
        uint64_t update_period_nanos_;
    };

} // namespace
