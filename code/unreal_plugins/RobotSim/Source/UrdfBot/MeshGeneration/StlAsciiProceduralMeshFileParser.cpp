#include "StlAsciiProceduralMeshFileParser.h"

ProceduralMeshSpecification StlAsciiProceduralMeshFileParser::ParseFromFile(FString fileName)
{
    IFileHandle* fileHandle = ProceduralMeshFileUtilities::OpenFile(fileName, true);

    this->_vertexIndexes.Empty();
    this->_maxVertexIndex = 0;
  
    ProceduralMeshSpecification meshSpecification;
    meshSpecification.Triangles.Empty();
    meshSpecification.Verticies.Empty();

    // In an ascii file, there are ~218 characters in a facet specification
    // For each facet, there are vertexes and 3 indexes
    // This is an overestimate, as there are duplicate indexes and vertexes
    // The point here is to avoid constant reallocation; a rough estimate is OK.
    meshSpecification.Triangles.Reserve((fileHandle->Size() * 3) / 220);
    meshSpecification.Verticies.Reserve((fileHandle->Size() * 3) / 220);

    FString line;
    bool insideFace = false;

    bool reachedEnd = false;

    while (ProceduralMeshFileUtilities::ReadNextLineTrimmingLeadingWhitespace(fileHandle, line) || line.Len() > 0)
    {
        if (line.Len() > 0)
        {
            if (line.StartsWith(TEXT("endsolid")))
            {
                reachedEnd = true;
            }

            if (line.StartsWith(TEXT("vertex")))
            {
                if (reachedEnd)
                {
                    delete fileHandle;
                    throw std::runtime_error("Corrupt STL file " + std::string(TCHAR_TO_UTF8(*fileName)) + ": vertex found after endsolid.");
                }

                TArray<FString> splitLine;
                line.ParseIntoArray(splitLine, TEXT(" "), true);
                if (splitLine.Num() != 4)
                {
                    delete fileHandle;
                    throw std::runtime_error("Improperly formatted vertex line: " + std::string(TCHAR_TO_UTF8(*line)) + ".");
                }

                FVector newVertex(FCString::Atof(*splitLine[1]), FCString::Atof(*splitLine[2]), FCString::Atof(*splitLine[3]));
                if (!this->_vertexIndexes.Contains(newVertex))
                {
                    meshSpecification.Verticies.Add(newVertex);
                    this->_vertexIndexes.Add(newVertex, this->_maxVertexIndex);
                    this->_maxVertexIndex++;
                }

                meshSpecification.Triangles.Add(this->_vertexIndexes[newVertex]);
            }
        }
    }

    if (!reachedEnd)
    {
        delete fileHandle;
        throw std::runtime_error("Corrupt STL file " + std::string(TCHAR_TO_UTF8(*fileName)) + ": no endsolid found.");
    }
    if (meshSpecification.Triangles.Num() % 3 != 0)
    {
        delete fileHandle;
        throw std::runtime_error("Improper number of vertexes in stl file. Found " + std::to_string(this->_vertexIndexes.Num()) + ", which is not divisible by 3.");
    }

    meshSpecification.ParseSuccessful = true;
    return meshSpecification;
}
