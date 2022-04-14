// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common_utils/Common.hpp"
#include "common_utils/CommonStructs.hpp"
#include "common_utils/EarthUtils.hpp"
#include "common_utils/UpdatableObject.hpp"

namespace RobotSim {

    class Environment : public UpdatableObject {
    public:
        struct State {
            // these fields must be set at initialization time
            Vector3r position;
            GeoPoint geo_point;

            // these fields are computed
            Vector3r gravity;
            real_T air_pressure;
            real_T temperature;
            real_T air_density;

            /**
             * @brief Construct a new State object
             *
             */
            State()
            {
            }

            /**
             * @brief Construct a new State object
             *
             * @param position_val
             * @param geo_point_val
             */
            State(const Vector3r& position_val, const GeoPoint& geo_point_val) : position(position_val), geo_point(geo_point_val)
            {
            }
        };

    public:
        /**
         * @brief Construct a new Environment object
         *
         */
        Environment()
        {
            // allow default constructor with later call for initialize
        }

        /**
         * @brief Construct a new Environment object
         *
         * @param initial
         */
        Environment(const State& initial)
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

            setHomeGeoPoint(initial_.geo_point);

            updateState(initial_, home_geo_point_);
        }

        /**
         * @brief
         *
         * @param home_geo_point
         */
        void setHomeGeoPoint(const GeoPoint& home_geo_point)
        {
            home_geo_point_ = HomeGeoPoint(home_geo_point);
        }

        /**
         * @brief
         *
         * @return GeoPoint
         */
        GeoPoint getHomeGeoPoint() const
        {
            return home_geo_point_.home_geo_point;
        }

        /**
         * @brief Set the position in local NED coordinates
         *
         * @param position
         */
        void setPosition(const Vector3r& position)
        {
            current_.position = position;
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
         * @return State&
         */
        State& getState()
        {
            return current_;
        }

        //*** Start: UpdatableState implementation ***//

        /**
         * @brief
         *
         */
        virtual void reset()
        {
            current_ = initial_;
        }

        /**
         * @brief
         *
         */
        virtual void update()
        {
            updateState(current_, home_geo_point_);
        }
        //*** End: UpdatableState implementation ***//

    private:
        /**
         * @brief
         *
         * @param state
         * @param home_geo_point
         */
        static void updateState(State& state, const HomeGeoPoint& home_geo_point)
        {
            state.geo_point = EarthUtils::nedToGeodetic(state.position, home_geo_point);

            real_T geo_pot = EarthUtils::getGeopotential(state.geo_point.altitude / 1000.0f);
            state.temperature = EarthUtils::getStandardTemperature(geo_pot);
            state.air_pressure = EarthUtils::getStandardPressure(geo_pot, state.temperature);
            state.air_density = EarthUtils::getAirDensity(state.air_pressure, state.temperature);

            // TODO: avoid recalculating square roots
            state.gravity = Vector3r(0, 0, EarthUtils::getGravity(state.geo_point.altitude));
        }

    private:
        State initial_, current_;
        HomeGeoPoint home_geo_point_;
    };

} // namespace
