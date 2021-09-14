#pragma once

#include "UrdfOrigin.h"

class UrdfCameraSpecification
{
    public:
        UrdfOrigin OriginTransform;
        FString Name;
        FString ParentLinkName;
        FString CameraKeyBinding;
        float LagScale = 0;
};