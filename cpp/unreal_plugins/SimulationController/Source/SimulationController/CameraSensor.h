//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include "SimulationController/BoostInterprocess.h"
#include "SpCore/ArrayDesc.h"

class AActor;
class UCameraComponent;
class USceneCaptureComponent2D;

struct RenderPassDesc
{
    USceneCaptureComponent2D* scene_capture_component_2d_ = nullptr;

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
    CameraSensor() = delete;
    CameraSensor(UCameraComponent* camera_component, const std::vector<std::string>& render_pass_names, unsigned int width, unsigned int height, float fov);
    ~CameraSensor();

    // Used by Agents.
    std::map<std::string, ArrayDesc> getObservationSpace() const;
    std::map<std::string, std::vector<uint8_t>> getObservation() const;

    // Unreal resources for each render pass are public in case they need to be modified by user code.
    std::map<std::string, RenderPassDesc> render_pass_descs_;

private:
    AActor* actor_ = nullptr;
};
