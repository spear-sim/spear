//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpSceneCaptureComponent2D.h"

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpFuncArray.h"
#include "SpCore/Unreal.h"

#include "SpComponents/SpFuncComponent.h"

USpSceneCaptureComponent2D::USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent = Unreal::createComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);

    SpFuncComponent->registerFunc("hello_world", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
        SpFuncArray<uint8_t> hello("hello");
        SpFuncArray<double> my_data("my_data");

        hello.setData("Hello World!");
        my_data.setData({1.0, 2.0, 3.0});

        SpFuncDataBundle return_values;
        return_values.packed_arrays_ = SpFuncArrayUtils::moveToPackedArrays({hello.getPtr(), my_data.getPtr()});
        return return_values;
    });
}

USpSceneCaptureComponent2D::~USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();
}
