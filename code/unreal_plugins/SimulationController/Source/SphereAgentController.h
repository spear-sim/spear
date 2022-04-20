#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UTickEvent;
class UWorld;

struct Box;

class SphereAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    SphereAgentController(UWorld* world);
    ~SphereAgentController();
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;

    void postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick);

private:

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* observation_camera_actor_ = nullptr;
    AActor* new_object_parent_actor_ = nullptr;

    UStaticMeshComponent* sphere_static_mesh_component_ = nullptr;
    UStaticMeshComponent* goal_static_mesh_component_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;

    UTickEvent* post_physics_pre_render_tick_event_ = nullptr;
    FDelegateHandle post_physics_pre_render_tick_event_handle_;
};
