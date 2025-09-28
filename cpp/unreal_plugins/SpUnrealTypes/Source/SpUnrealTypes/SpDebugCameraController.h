//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/DebugCameraController.h>

#include "SpDebugCameraController.generated.h"

UCLASS()
class ASpDebugCameraController : public ADebugCameraController
{
    GENERATED_BODY()
public:
    ASpDebugCameraController();
    ~ASpDebugCameraController() override;

    // APlayerController interface
    void PostInitializeComponents() override;
};
