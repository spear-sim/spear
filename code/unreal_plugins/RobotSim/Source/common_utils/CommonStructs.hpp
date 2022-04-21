// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <ostream>

#include "common_utils/Common.hpp"

namespace RobotSim {

    /**
     * @brief velocity
     *
     */
    struct Twist {
        Vector3r linear, angular;

        /**
         * @brief Construct a new Twist object
         *
         */
        Twist()
        {
        }

        /**
         * @brief Construct a new Twist object
         *
         * @param linear_val
         * @param angular_val
         */
        Twist(const Vector3r& linear_val, const Vector3r& angular_val) : linear(linear_val), angular(angular_val)
        {
        }

        /**
         * @brief
         *
         * @return const Twist
         */
        static const Twist zero()
        {
            static const Twist zero_twist(Vector3r::Zero(), Vector3r::Zero());
            return zero_twist;
        }
    };

    /**
     * @brief Force & torque
     *
     */
    struct Wrench {
        Vector3r force, torque;

        /**
         * @brief Construct a new Wrench object
         *
         */
        Wrench()
        {
        }

        /**
         * @brief Construct a new Wrench object
         *
         * @param force_val
         * @param torque_val
         */
        Wrench(const Vector3r& force_val, const Vector3r& torque_val) : force(force_val), torque(torque_val)
        {
        }

        // support basic arithmatic

        /**
         * @brief
         *
         * @param other
         * @return Wrench
         */
        Wrench operator+(const Wrench& other) const
        {
            Wrench result;
            result.force = this->force + other.force;
            result.torque = this->torque + other.torque;
            return result;
        }

        /**
         * @brief
         *
         * @param other
         * @return Wrench
         */
        Wrench operator+=(const Wrench& other)
        {
            force += other.force;
            torque += other.torque;
            return *this;
        }

        /**
         * @brief
         *
         * @param other
         * @return Wrench
         */
        Wrench operator-(const Wrench& other) const
        {
            Wrench result;
            result.force = this->force - other.force;
            result.torque = this->torque - other.torque;
            return result;
        }

        /**
         * @brief
         *
         * @param other
         * @return Wrench
         */
        Wrench operator-=(const Wrench& other)
        {
            force -= other.force;
            torque -= other.torque;
            return *this;
        }

        /**
         * @brief
         *
         * @return const Wrench
         */
        static const Wrench zero()
        {
            static const Wrench zero_wrench(Vector3r::Zero(), Vector3r::Zero());
            return zero_wrench;
        }
    };

    /**
     * @brief
     *
     */
    struct Momentums {
        Vector3r linear;
        Vector3r angular;

        /**
         * @brief Construct a new Momentums object
         *
         */
        Momentums()
        {
        }

        /**
         * @brief Construct a new Momentums object
         *
         * @param linear_val
         * @param angular_val
         */
        Momentums(const Vector3r& linear_val, const Vector3r& angular_val) : linear(linear_val), angular(angular_val)
        {
        }

        /**
         * @brief
         *
         * @return const Momentums
         */
        static const Momentums zero()
        {
            static const Momentums zero_val(Vector3r::Zero(), Vector3r::Zero());
            return zero_val;
        }
    };

    /**
     * @brief
     *
     */
    struct Accelerations {
        Vector3r linear;
        Vector3r angular;

        /**
         * @brief Construct a new Accelerations object
         *
         */
        Accelerations()
        {
        }

        /**
         * @brief Construct a new Accelerations object
         *
         * @param linear_val
         * @param angular_val
         */
        Accelerations(const Vector3r& linear_val, const Vector3r& angular_val) : linear(linear_val), angular(angular_val)
        {
        }

        /**
         * @brief
         *
         * @return const Accelerations
         */
        static const Accelerations zero()
        {
            static const Accelerations zero_val(Vector3r::Zero(), Vector3r::Zero());
            return zero_val;
        }
    };

    /**
     * @brief
     *
     */
    struct PoseWithCovariance {
        VectorMath::Pose pose;
        vector<real_T> covariance; // 36 elements, 6x6 matrix

        /**
         * @brief Construct a new Pose With Covariance object
         *
         */
        PoseWithCovariance() : covariance(36, 0)
        {
        }
    };

    /**
     * @brief
     *
     */
    struct PowerSupply {
        vector<real_T> voltage, current;
    };

    /**
     * @brief
     *
     */
    struct TwistWithCovariance {
        Twist twist;
        vector<real_T> covariance; // 36 elements, 6x6 matrix

        /**
         * @brief Construct a new Twist With Covariance object
         *
         */
        TwistWithCovariance() : covariance(36, 0)
        {
        }
    };

    /**
     * @brief
     *
     */
    struct Joystick {
        vector<float> axes;
        vector<int> buttons;
    };

    /**
     * @brief
     *
     */
    struct Odometry {
        PoseWithCovariance pose;
        TwistWithCovariance twist;
    };

    /**
     * @brief
     *
     */
    struct GeoPoint {
        double latitude = 0, longitude = 0;
        float altitude = 0;

        /**
         * @brief Construct a new Geo Point object
         *
         */
        GeoPoint()
        {
        }

        /**
         * @brief Construct a new Geo Point object
         *
         * @param latitude_val
         * @param longitude_val
         * @param altitude_val
         */
        GeoPoint(double latitude_val, double longitude_val, float altitude_val)
        {
            set(latitude_val, longitude_val, altitude_val);
        }

        /**
         * @brief
         *
         * @param latitude_val
         * @param longitude_val
         * @param altitude_val
         */
        void set(double latitude_val, double longitude_val, float altitude_val)
        {
            latitude = latitude_val, longitude = longitude_val;
            altitude = altitude_val;
        }

        /**
         * @brief
         *
         * @param os
         * @param g
         * @return std::ostream&
         */
        friend std::ostream& operator<<(std::ostream& os, GeoPoint const& g)
        {
            return os << "[" << g.latitude << ", " << g.longitude << ", " << g.altitude << "]";
        }

        /**
         * @brief
         *
         * @return std::string
         */
        std::string to_string() const
        {
            return std::to_string(latitude) + string(", ") + std::to_string(longitude) + string(", ") + std::to_string(altitude);
        }
    };

    /**
     * @brief
     *
     */
    struct HomeGeoPoint {
        GeoPoint home_geo_point;
        double lat_rad, lon_rad;
        double cos_lat, sin_lat;

        /**
         * @brief Construct a new Home Geo Point object
         *
         */
        HomeGeoPoint()
        {
        }

        /**
         * @brief Construct a new Home Geo Point object
         *
         * @param home_geo_point_val
         */
        HomeGeoPoint(const GeoPoint& home_geo_point_val)
        {
            initialize(home_geo_point_val);
        }

        /**
         * @brief
         *
         * @param home_geo_point_val
         */
        void initialize(const GeoPoint& home_geo_point_val)
        {
            home_geo_point = home_geo_point_val;
            lat_rad = Utils::degreesToRadians(home_geo_point.latitude);
            lon_rad = Utils::degreesToRadians(home_geo_point.longitude);
            cos_lat = cos(lat_rad);
            sin_lat = sin(lat_rad);
        }
    };

    /**
     * @brief
     *
     */
    struct ProjectionMatrix {
        float matrix[4][4];

        void setTo(float val)
        {
            for (auto i = 0; i < 4; ++i)
                for (auto j = 0; j < 4; ++j)
                    matrix[i][j] = val;
        }
    };

    /**
     * @brief
     *
     */
    struct CollisionInfo {
        bool has_collided = false;
        Vector3r normal = Vector3r::Zero();
        Vector3r impact_point = Vector3r::Zero();
        Vector3r position = Vector3r::Zero();
        real_T penetration_depth = 0;
        TTimePoint time_stamp = 0;
        unsigned int collision_count = 0;
        std::string object_name;
        int object_id = -1;

        /**
         * @brief Construct a new Collision Info object
         *
         */
        CollisionInfo()
        {
        }

        /**
         * @brief Construct a new Collision Info object
         *
         * @param has_collided_val
         * @param normal_val
         * @param impact_point_val
         * @param position_val
         * @param penetration_depth_val
         * @param time_stamp_val
         * @param object_name_val
         * @param object_id_val
         */
        CollisionInfo(bool has_collided_val,
                      const Vector3r& normal_val,
                      const Vector3r& impact_point_val,
                      const Vector3r& position_val,
                      real_T penetration_depth_val,
                      TTimePoint time_stamp_val,
                      const std::string& object_name_val,
                      int object_id_val)
            : has_collided(has_collided_val), normal(normal_val),
              impact_point(impact_point_val), position(position_val),
              penetration_depth(penetration_depth_val), time_stamp(time_stamp_val),
              object_name(object_name_val), object_id(object_id_val)
        {
        }
    };

    /**
     * @brief
     *
     */
    struct CameraInfo {
        Pose pose;
        float fov;
        ProjectionMatrix proj_mat;

        /**
         * @brief Construct a new Camera Info object
         *
         */
        CameraInfo()
        {
        }

        /**
         * @brief Construct a new Camera Info object
         *
         * @param pose_val
         * @param fov_val
         * @param proj_mat_val
         */
        CameraInfo(const Pose& pose_val,
                   float fov_val,
                   const ProjectionMatrix& proj_mat_val)
            : pose(pose_val), fov(fov_val), proj_mat(proj_mat_val)
        {
        }
    };

    /**
     * @brief
     *
     */
    struct CollisionResponse {
        unsigned int collision_count_raw = 0;
        unsigned int collision_count_non_resting = 0;
        TTimePoint collision_time_stamp = 0;
    };

    /**
     * @brief
     *
     */
    struct GeoPose {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Quaternionr orientation;
        GeoPoint position;
    };

    // struct RCData {
    //     TTimePoint timestamp = 0;
    //     //pitch, roll, yaw should be in range -1 to 1
    //     //switches should be integer value indicating its state, 0=on, 1=off for example.
    //     float pitch = 0, roll = 0, throttle = 0, yaw = 0;
    //     float left_z = 0, right_z = 0;
    //     uint16_t switches = 0;
    //     std::string vendor_id = "";
    //     bool is_initialized = false; //is RC connected?
    //     bool is_valid = false; //must be true for data to be valid
    //
    //     unsigned int getSwitch(uint16_t index) const
    //     {
    //         return switches && (1 << index) ? 1 : 0;
    //     }
    //
    //     void add(const RCData& other)
    //     {
    //         pitch += other.pitch; roll += other.roll; throttle += other.throttle; yaw += other.yaw;
    //     }
    //     void subtract(const RCData& other)
    //     {
    //         pitch -= other.pitch; roll -= other.roll; throttle -= other.throttle; yaw -= other.yaw;
    //     }
    //     void divideBy(float k)
    //     {
    //         pitch /= k; roll /= k; throttle /= k; yaw /= k;
    //     }
    //     bool isAnyMoreThan(float k)
    //     {
    //         using std::abs;
    //         return abs(pitch) > k || abs(roll) > k || abs(throttle) > k || abs(yaw) > k;
    //     }
    //     string toString()
    //     {
    //         return Utils::stringf("RCData[pitch=%f, roll=%f, throttle=%f, yaw=%f]", pitch, roll, throttle, yaw);
    //     }
    // };

    /**
     * @brief
     *
     */
    struct LidarData {

        TTimePoint time_stamp = 0;
        vector<real_T> point_cloud;

        LidarData()
        {
        }
    };

    /**
     * @brief
     *
     */
    struct CameraPose {
        std::string camera_name;
        Vector3r translation;
        Quaternionr rotation;
    };

    /**
     * @brief
     *
     */
    struct RayCastRequest {
        Vector3r position;
        Vector3r direction;
        std::string reference_frame_link;
        bool through_blocking;
        float persist_seconds;
    };

    /**
     * @brief
     *
     */
    struct RayCastHit {
        std::string collided_actor_name;
        Vector3r hit_point;
        Vector3r hit_normal;
    };

    /**
     * @brief
     *
     */
    struct RayCastResponse {
        std::vector<RayCastHit> hits;
    };

    /**
     * @brief
     *
     */
    struct DrawableShape {
        std::string reference_frame_link;
        int type;
        std::vector<float> shape_params;
    };

    /**
     * @brief
     *
     */
    struct QueueVector3r {
        std::vector<RobotSim::Vector3r> mVector3rArray;
        QueueVector3r(const std::vector<RobotSim::Vector3r>& rhs) : mVector3rArray(rhs){};
    };

    /**
     * @brief
     *
     */
    struct DrawableShapeRequest {
        std::unordered_map<std::string, DrawableShape> shapes;
        bool persist_unmentioned;
    };

} // namespace
