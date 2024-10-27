//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpSceneCaptureComponent2D.h"

#include <memory> // std::make_unique

#include <Engine/EngineTypes.h> // EEndPlayReason
#include <Math/Rotator.h>
#include <Math/Vector.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/SpFuncArray.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealObj.h"

#include "SpComponents/SpFuncComponent.h"

USpSceneCaptureComponent2D::USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent = Unreal::createComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);
}

USpSceneCaptureComponent2D::~USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpSceneCaptureComponent2D::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    USceneCaptureComponent2D::BeginPlay();

    int shared_memory_num_bytes = 1024;
    shared_memory_region_ = std::make_unique<SharedMemoryRegion>(shared_memory_num_bytes);
    SP_ASSERT(shared_memory_region_);

    // the "smem_observation" name needs to be unique within this SpFuncComponent, but does not need to be globally unique
    shared_memory_view_ = SpFuncSharedMemoryView(shared_memory_region_->getView(), SpFuncSharedMemoryUsageFlags::ReturnValue);
    SpFuncComponent->registerSharedMemoryView("smem_observation", shared_memory_view_);

    SpFuncComponent->registerFunc("hello_world", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {

        // define arg objects
        SpFuncArrayView<double> action("action");
        SpFuncArrayView<double> action_shared("action_shared");
        UnrealObj<FVector> in_location("in_location");
        UnrealObj<FRotator> in_rotation("in_rotation");

        // initialize arg objects from the data bundle that was passed in
        SpFuncArrayUtils::setViewsFromPackedArrays({action.getPtr(), action_shared.getPtr()}, args.packed_arrays_);
        UnrealObjUtils::setObjectPropertiesFromStrings({in_location.getPtr(), in_rotation.getPtr()}, args.unreal_obj_strings_);

        SP_LOG("action[0]:        ", Std::at(action.getView(), 0));
        SP_LOG("action[1]:        ", Std::at(action.getView(), 1));
        SP_LOG("action[2]:        ", Std::at(action.getView(), 2));
        SP_LOG("action_shared[0]: ", Std::at(action_shared.getView(), 0));
        SP_LOG("action_shared[1]: ", Std::at(action_shared.getView(), 1));
        SP_LOG("action_shared[2]: ", Std::at(action_shared.getView(), 2));
        SP_LOG("in_location:      ", in_location.getObj().X, " ", in_location.getObj().Y, " ", in_location.getObj().Z);
        SP_LOG("in_rotation:      ", in_rotation.getObj().Pitch, " ", in_rotation.getObj().Yaw, " ", in_rotation.getObj().Roll);
        SP_LOG("info:             ", args.info_);

        // define return value objects
        SpFuncArray<double> observation("observation");
        SpFuncArray<double> observation_shared("observation_shared");
        UnrealObj<FVector> out_location("out_location");
        UnrealObj<FRotator> out_rotation("out_rotation");

        // set return value objects
        observation.setData({12.0, 13.0, 14.0});
        observation_shared.setData(shared_memory_view_, {3}, "smem_observation");
        observation_shared.setDataValues({15.0, 16.0, 17.0});
        out_location.setObj(FVector(18.0, 19.0, 20.0));
        out_rotation.setObj(FRotator(21.0, 22.0, 23.0));
        std::string info = "Success";

        SP_LOG("observation[0]:        ", Std::at(observation.getView(), 0));
        SP_LOG("observation[1]:        ", Std::at(observation.getView(), 1));
        SP_LOG("observation[2]:        ", Std::at(observation.getView(), 2));
        SP_LOG("observation_shared[0]: ", Std::at(observation_shared.getView(), 0));
        SP_LOG("observation_shared[1]: ", Std::at(observation_shared.getView(), 1));
        SP_LOG("observation_shared[2]: ", Std::at(observation_shared.getView(), 2));
        SP_LOG("out_location:          ", out_location.getObj().X, " ", out_location.getObj().Y, " ", out_location.getObj().Z);
        SP_LOG("out_rotation:          ", out_rotation.getObj().Pitch, " ", out_rotation.getObj().Yaw, " ", out_rotation.getObj().Roll);
        SP_LOG("info:                  ", info);

        // initialize output data bundle from return value objects
        SpFuncDataBundle return_values;
        return_values.packed_arrays_ = SpFuncArrayUtils::moveToPackedArrays({observation.getPtr(), observation_shared.getPtr()});
        return_values.unreal_obj_strings_ = UnrealObjUtils::getObjectPropertiesAsStrings({out_location.getPtr(), out_rotation.getPtr()});
        return_values.info_ = info;

        return return_values;
    });
}

void USpSceneCaptureComponent2D::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    USceneCaptureComponent2D::EndPlay(end_play_reason);

    SpFuncComponent->unregisterFunc("hello_world");
    SpFuncComponent->unregisterSharedMemoryView("smem_observation");

    shared_memory_region_ = nullptr;
}
