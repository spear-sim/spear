// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "CommonStructs.hpp"

namespace RobotSim {
    /**
     * @brief
     *
     */
    class WorldSimApiBase {
    public:
        /**
         * @brief Destroy the World Sim Api Base object
         *
         */
        virtual ~WorldSimApiBase() = default;

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        virtual bool isPaused() const = 0;

        /**
         * @brief
         *
         */
        virtual void reset() = 0;

        /**
         * @brief
         *
         * @param is_paused
         */
        virtual void pause(bool is_paused) = 0;

        /**
         * @brief
         *
         * @param seconds
         */
        virtual void continueForTime(double seconds) = 0;

        /**
         * @brief
         *
         * @param mesh_name
         * @param object_id
         * @param is_name_regex
         * @return true
         * @return false
         */
        virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;

        /**
         * @brief Get the Segmentation Object I D object
         *
         * @param mesh_name
         * @return int
         */
        virtual int getSegmentationObjectID(const std::string& mesh_name) const = 0;

        /**
         * @brief
         *
         * @param message
         * @param message_param
         * @param severity
         */
        virtual void printLogMessage(const std::string& message,
                                     const std::string& message_param = "", unsigned char severity = 0) = 0;

        /**
         * @brief
         *
         * @param object_name
         * @return Pose
         */
        virtual Pose getObjectPose(const std::string& object_name) const = 0;

        /**
         * @brief Set the Object Pose object
         *
         * @param object_name
         * @param pose
         * @param teleport
         * @return true
         * @return false
         */
        virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) = 0;

        /**
         * @brief
         *
         * @param object_class_name
         * @param object_name
         * @param pose
         * @return true
         * @return false
         */
        virtual bool spawnStaticMeshObject(const std::string& object_class_name, const std::string& object_name, const Pose& pose) = 0;

        /**
         * @brief
         *
         * @param object_name
         * @return true
         * @return false
         */
        virtual bool deleteObject(const std::string& object_name) = 0;

        //----------- APIs to control ACharacter in scene ----------/

        /**
         * @brief
         *
         * @param expression_name
         * @param value
         * @param character_name
         */
        virtual void charSetFaceExpression(const std::string& expression_name, float value, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param expression_name
         * @param character_name
         * @return float
         */
        virtual float charGetFaceExpression(const std::string& expression_name, const std::string& character_name) const = 0;

        /**
         * @brief
         *
         * @return std::vector<std::string>
         */
        virtual std::vector<std::string> charGetAvailableFaceExpressions() = 0;

        /**
         * @brief
         *
         * @param value
         * @param character_name
         */
        virtual void charSetSkinDarkness(float value, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param character_name
         * @return float
         */
        virtual float charGetSkinDarkness(const std::string& character_name) const = 0;

        /**
         * @brief
         *
         * @param value
         * @param character_name
         */
        virtual void charSetSkinAgeing(float value, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param character_name
         * @return float
         */
        virtual float charGetSkinAgeing(const std::string& character_name) const = 0;

        /**
         * @brief
         *
         * @param q
         * @param character_name
         */
        virtual void charSetHeadRotation(const RobotSim::Quaternionr& q, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param character_name
         * @return RobotSim::Quaternionr
         */
        virtual RobotSim::Quaternionr charGetHeadRotation(const std::string& character_name) const = 0;

        /**
         * @brief
         *
         * @param bone_name
         * @param pose
         * @param character_name
         */
        virtual void charSetBonePose(const std::string& bone_name, const RobotSim::Pose& pose, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param bone_name
         * @param character_name
         * @return RobotSim::Pose
         */
        virtual RobotSim::Pose charGetBonePose(const std::string& bone_name, const std::string& character_name) const = 0;

        /**
         * @brief
         *
         * @param bone_name
         * @param character_name
         */
        virtual void charResetBonePose(const std::string& bone_name, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param preset_name
         * @param value
         * @param character_name
         */
        virtual void charSetFacePreset(const std::string& preset_name, float value, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param presets
         * @param character_name
         */
        virtual void charSetFacePresets(const std::unordered_map<std::string, float>& presets, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param poses
         * @param character_name
         */
        virtual void charSetBonePoses(const std::unordered_map<std::string, RobotSim::Pose>& poses, const std::string& character_name) = 0;

        /**
         * @brief
         *
         * @param bone_names
         * @param character_name
         * @return std::unordered_map<std::string, RobotSim::Pose>
         */
        virtual std::unordered_map<std::string, RobotSim::Pose> charGetBonePoses(const std::vector<std::string>& bone_names, const std::string& character_name) const = 0;
    };

} // namespace
