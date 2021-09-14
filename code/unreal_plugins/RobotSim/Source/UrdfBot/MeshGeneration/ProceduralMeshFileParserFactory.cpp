#include "ProceduralMeshFileParserFactory.h"

TSharedPtr<ProceduralMeshFileParser> ProceduralMeshFileParserFactory::Create(ProceduralMeshFileType fileType)
{
    switch (fileType)
    {
        case ProceduralMeshFileType::STL_ASCII:
            return TSharedPtr<ProceduralMeshFileParser>(new StlAsciiProceduralMeshFileParser());
        default:
            throw new std::runtime_error("Unsupported file type for procedural mesh parsing.");
    }
}
