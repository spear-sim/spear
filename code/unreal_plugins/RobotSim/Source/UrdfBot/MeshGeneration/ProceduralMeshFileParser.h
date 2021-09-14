#pragma once

#include "ProceduralMeshSpecification.h"
#include "ProceduralMeshFileType.h"
#include "ProceduralMeshFileUtilities.h"

class ProceduralMeshFileParser
{
    public:
        virtual ~ProceduralMeshFileParser() {};
        virtual ProceduralMeshSpecification ParseFromFile(FString fileName) = 0;
};