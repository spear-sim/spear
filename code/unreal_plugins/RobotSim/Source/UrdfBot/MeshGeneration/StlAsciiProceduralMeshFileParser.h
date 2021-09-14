#pragma once

#include "ProceduralMeshFileType.h"
#include "ProceduralMeshFileParser.h"
#include "ProceduralMeshSpecification.h"

class StlAsciiProceduralMeshFileParser : public ProceduralMeshFileParser
{
    public:
        virtual ProceduralMeshSpecification ParseFromFile(FString fileName) override;

    private:

        TMap<FVector, int> _vertexIndexes;
        int _maxVertexIndex;
};
