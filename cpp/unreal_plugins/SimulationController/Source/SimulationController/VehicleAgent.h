//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "SimulationController/Agent.h"

class UWorld;

class AVehiclePawn;
class CameraSensor;
class ImuSensor;
class SonarSensor;
struct ArrayDesc;

class VehicleAgent : public Agent
{
public:
    VehicleAgent(UWorld* world);
    ~VehicleAgent();

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

    AVehiclePawn* wheeled_vehicle_pawn_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_;
    std::unique_ptr<ImuSensor> imu_sensor_;
    std::unique_ptr<SonarSensor> sonar_sensor_;
};
