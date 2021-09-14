#pragma once

#include <stdexcept>

#include "ProceduralMeshFileParser.h"
#include "ProceduralMeshFileType.h"
#include "StlAsciiProceduralMeshFileParser.h"

class ProceduralMeshFileParserFactory
{
    public:
        TSharedPtr<ProceduralMeshFileParser> Create(ProceduralMeshFileType fileType);
};
