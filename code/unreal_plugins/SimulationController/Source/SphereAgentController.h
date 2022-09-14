#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

class AActor;
class UTickEvent;
class UWorld;

class CameraSensor;
struct Box;

class SphereAgentController : public AgentController
{
public:

    SphereAgentController(UWorld* world);
    ~SphereAgentController();
 
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

    void postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick);

private:

    AActor* agent_actor_ = nullptr;
    AActor* camera_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    std::unique_ptr<CameraSensor> observation_camera_sensor_ = nullptr;

    UStaticMeshComponent* sphere_static_mesh_component_ = nullptr;
    UStaticMeshComponent* goal_static_mesh_component_ = nullptr;

    UTickEvent* post_physics_pre_render_tick_event_ = nullptr;
    FDelegateHandle post_physics_pre_render_tick_event_handle_;
};
