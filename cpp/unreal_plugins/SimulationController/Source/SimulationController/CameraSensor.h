//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include "CoreUtils/ArrayDesc.h"
#include "SimulationController/BoostInterprocess.h"

class AActor;
class UCameraComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

struct RenderPassDesc
{
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;

    // strictly speaking, we could store width and height once for all render passes, but we store them here for simplicity
    int width_ = -1;
    int height_ = -1;
    int num_bytes_ = -1;

    // only used if SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY is set to True
    std::string shared_memory_name_; // externally visible name
    std::string shared_memory_id_;   // ID used to manage the shared memory resource internally
    boost::interprocess::mapped_region shared_memory_mapped_region_;
};

class CameraSensor
{
public:
    CameraSensor(UCameraComponent* camera_component, const std::vector<std::string>& render_pass_names, unsigned int width, unsigned int height, float fov);
    ~CameraSensor();

    // high-level interface for getting render data as an observation that conforms to our Agent interface
    std::map<std::string, ArrayDesc> getObservationSpace(const std::vector<std::string>& observation_components) const;
    std::map<std::string, std::vector<uint8_t>> getObservation(const std::vector<std::string>& observation_components) const;

    // Unreal resources for each render pass are public in case they need to be modified by user code
    std::map<std::string, RenderPassDesc> render_pass_descs_;

private:
    AActor* parent_actor_ = nullptr;
};
