//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <memory> // std::unique_ptr
#include <string>
#include <vector>

#include "SpCore/ArrayDesc.h"

#include "SpServices/Legacy/Agent.h"
#include "SpServices/Legacy/ClassRegistrationUtils.h"

class UWorld;

class AUrdfRobotPawn;
class CameraSensor;

class UrdfRobotAgent : public Agent
{
public:
    UrdfRobotAgent() = delete;
    UrdfRobotAgent(UWorld* world);
    ~UrdfRobotAgent();

    void findObjectReferences(UWorld* world) override;
    void cleanUpObjectReferences() override;

    std::map<std::string, ArrayDesc> getActionSpace() const override;
    std::map<std::string, ArrayDesc> getObservationSpace() const override;
    std::map<std::string, ArrayDesc> getStepInfoSpace() const override;

    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;

    void reset() override;
    bool isReady() const override;

private:
    AUrdfRobotPawn* urdf_robot_pawn_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_;

    inline static auto s_class_registration_handler_ = ClassRegistrationUtils::registerClass<UrdfRobotAgent>(Agent::s_class_registrar_, "UrdfRobotAgent");
};
