//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/DebugCameraHUD.h>

#include "SpDebugCameraHUD.generated.h"

class UMeshComponent;

UCLASS()
class ASpDebugCameraHUD : public ADebugCameraHUD
{
    GENERATED_BODY()
public:
    // AActor interface
    void PostRender() override;
};
