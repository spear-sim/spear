#pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

struct Box;
class AActor;
class USceneCaptureComponent2D;
class UTickEvent;
class UWorld;

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

    void postPhysicsPreRenderTick(float delta_time, enum ELevelTick tick_type, FActorComponentTickFunction *this_tick_function);

private:

    AActor* agent_actor_ = nullptr;
    AActor* goal_actor_ = nullptr;
    AActor* debug_camera_actor_ = nullptr;
    AActor* observation_camera_actor_ = nullptr;

    UStaticMeshComponent* sphere_static_mesh_component_ = nullptr;
    UStaticMeshComponent* cone_static_mesh_component_ = nullptr;
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;

    UTickEvent* post_physics_event_ = nullptr;
    FDelegateHandle post_physics_event_handle_;
};
