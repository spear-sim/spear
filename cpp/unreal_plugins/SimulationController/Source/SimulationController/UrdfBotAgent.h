//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "SimulationController/Agent.h"

class AActor;
class UWorld;

class AUrdfBotPawn;
class CameraSensor;
class ImuSensor;
class SonarSensor;
struct Box;

class UrdfBotAgent : public Agent
{
public:
    UrdfBotAgent(UWorld* world);
    ~UrdfBotAgent();

    void findObjectReferences(UWorld* world) override;
    void cleanUpObjectReferences() override;

    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    std::map<std::string, Box> getStepInfoSpace() const override;

    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;

    void reset() override;
    bool isReady() const override;

private:
    AUrdfBotPawn* urdf_bot_pawn_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_;
    std::unique_ptr<ImuSensor> imu_sensor_;
    std::unique_ptr<SonarSensor> sonar_sensor_;
};
