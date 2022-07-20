#include "DepthCameraSensor.h"

#include "CameraSensor.h"
#include <Engine/World.h>
#include <UObject/UObjectGlobals.h>

#include "Assert.h"
#include "Config.h"
#include "Serialize.h"

DepthCameraSensor::DepthCameraSensor(UWorld* world) : CameraSensor(world) {
    //Add depth postprocess blendable
    AddPostProcessingMaterial(TEXT("/SimulationController/PostProcessMaterials/PostProcessBlendable.PostProcessBlendable"));

    //Set Blendable Materials
    SetPostProcessBlendables(); 
}

DepthCameraSensor::~DepthCameraSensor(){

}