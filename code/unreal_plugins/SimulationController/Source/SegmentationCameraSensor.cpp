#include "SegmentationCameraSensor.h"

#include "CameraSensor.h"
#include <Engine/World.h>
#include <UObject/UObjectGlobals.h>

#include "Assert.h"
#include "Config.h"
#include "Serialize.h"

SegmentationCameraSensor::SegmentationCameraSensor(UWorld* world) : CameraSensor(world) {
    //Add segmentation postprocess blendable
    AddPostProcessingMaterial(TEXT("/SimulationController/PostProcessMaterials/PostProcess_Segmentation.PostProcess_Segmentation"));

    //Set Blendable Materials
    SetPostProcessBlendables(); 
}

SegmentationCameraSensor::~SegmentationCameraSensor(){

}