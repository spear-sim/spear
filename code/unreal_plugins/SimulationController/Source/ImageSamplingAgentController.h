    #pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

class AActor;
class ANavMeshBoundsVolume;
class ARecastNavMesh;
class CameraSensor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

struct FBox;
struct FNavAgentProperties;
struct FVector;

struct Box;

class ImageSamplingAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    ImageSamplingAgentController(UWorld* world);
    ~ImageSamplingAgentController();
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    void reset() override;
    bool isReady() const override;

    //testing
    void TweakLights();

private:

    void rebuildNavSystem();
    FBox getWorldBoundingBox(bool bScaleCeiling = true);
    
    UWorld* world_;

    ARecastNavMesh* nav_mesh_ = nullptr;
    ANavMeshBoundsVolume* dummy_navmesh_bound_volume_ = nullptr;

    CameraSensor* rgb_camera_sensor_ = nullptr;
    
    //AActor* camera_actor_ = nullptr;
    //AActor* virtual_world_level_manager_ = nullptr;

    //UTextureRenderTarget2D* texture_render_target_ = nullptr;
    //USceneCaptureComponent2D* scene_capture_component_ = nullptr;
};
