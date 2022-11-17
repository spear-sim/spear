#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Agent.h"

class ACameraActor;
class ARecastNavMesh;
class UNavigationSystemV1;
class UWorld;

class CameraSensor;

struct Box;

class CameraAgent : public Agent
{
public:

    CameraAgent(UWorld* world);
    ~CameraAgent();
    
    void findObjectReferences(UWorld* world) override;
    void cleanUpObjectReferences() override;

    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    std::map<std::string, Box> getStepInfoSpace() const override;

    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;

    void reset() override;
    bool isReady() const override;
    
private:

    void buildNavMesh();

    ACameraActor* camera_actor_ = nullptr;

    UNavigationSystemV1* nav_sys_ = nullptr;
    ARecastNavMesh* nav_mesh_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_;

    std::map<std::string, std::vector<float>> action_;
};
