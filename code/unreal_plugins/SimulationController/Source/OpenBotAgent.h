#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Math/Vector.h>

#include "Agent.h"

class AActor;
class ANavigationData;
class ARecastNavMesh;
class UNavigationSystemV1;
class UWorld;

class AOpenBotPawn;
class CameraSensor;

struct Box;

class OpenBotAgent : public Agent
{
public:

    OpenBotAgent(UWorld* world);
    ~OpenBotAgent();

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

    // Rebuild the navigation mesh of the agent
    void buildNavMesh();

    // Generate a collision-free trajectory between an initial and a target location
    void generateTrajectoryToGoal();

    AOpenBotPawn* open_bot_pawn_ = nullptr;
    AActor* goal_actor_ = nullptr;

    // Navigation
    UNavigationSystemV1* nav_sys_ = nullptr;
    ARecastNavMesh* nav_mesh_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_ = nullptr;

    std::vector<float> trajectory_; // An array containing the different waypoints to be followed by the agent, converted into a serialized format X0, Y0, Z0, X1, Y1, Z1, ... Xn, Yn, Zn 
};
