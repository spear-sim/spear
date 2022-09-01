    #pragma once

#include <map>
#include <string>
#include <vector>

#include "AgentController.h"

class AActor;
class CameraSensor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UWorld;

struct Box;

class SlamDatasetAgentController : public AgentController
{
public:

    // This UWorld pointer passed here points to the only running game world.
    SlamDatasetAgentController(UWorld* world);
    ~SlamDatasetAgentController();
    
    std::map<std::string, Box> getActionSpace() const override;
    std::map<std::string, Box> getObservationSpace() const override;
    void applyAction(const std::map<std::string, std::vector<float>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    void reset() override;
    bool isReady() const override;

    //testing
    void RemoveVirtualLights();

private:

    void rebuildNavSystem();
    FBox getWorldBoundingBox(bool bScaleCeiling = true);
    void generateTrajectoryToPredefinedTarget();
    
    void ChangeTranslucentToOpaque();
    void EnableTranslucency(bool activate) const;

    UWorld* world_;

    ARecastNavMesh* nav_mesh_ = nullptr;
    TArray<FNavPathPoint> way_points_;
    ANavigationData* nav_data_;
    FPathFindingQuery nav_query_;
    UNavigationSystemV1* nav_sys_;

    //AActor* camera_actor_ = nullptr;
    CameraSensor* rgb_camera_sensor_ = nullptr;
    CameraSensor* depth_camera_sensor_ = nullptr;

    AActor* new_object_parent_actor_ = nullptr;

    TArray<UMaterialInstanceDynamic*> translucent_materials_;

    //UTextureRenderTarget2D* texture_render_target_ = nullptr;
    //USceneCaptureComponent2D* scene_capture_component_ = nullptr;
};

//swap AActor, scenecapture and texturerendertarget for CameraActor