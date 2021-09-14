#pragma once

#include "CoreMinimal.h"
#include "Runtime/Core/Public/HAL/PlatformFilemanager.h"
#include "Runtime/Core/Public/GenericPlatform/GenericPlatformFile.h"

#include <stdexcept>
#include <string>

class ProceduralMeshFileUtilities
{
    public:
        static IFileHandle* OpenFile(FString fileName, bool Read)
        {
            // replace backslashes with forward slashes
            fileName = fileName.Replace(TEXT("\\"), TEXT("/"));

            IPlatformFile& platformFile = FPlatformFileManager::Get().GetPlatformFile();

            

            IFileHandle* fileHandle = nullptr;
            if (Read)
            {
                if (!platformFile.FileExists(*fileName))
                {
                    throw std::runtime_error("Unable to find requested file " + std::string(TCHAR_TO_UTF8(*fileName)) + ".");
                }

                fileHandle = platformFile.OpenRead(*fileName, false);
            }
            else
            {
                fileHandle = platformFile.OpenWrite(*fileName, false, false);
            }

            if (!fileHandle)
            {
                throw std::runtime_error("Unable to open file " + std::string(TCHAR_TO_UTF8(*fileName)) + ".");
            }

            return fileHandle;
        }

        static bool ReadNextLineTrimmingLeadingWhitespace(IFileHandle* fileHandle, FString &nextLine, int expectedLineLength = 128)
        {
            nextLine.Empty();
            uint8 currentCharacter = 0;

            // Won't crash if too short, but will cause O(N) reallocation
            nextLine.Reserve(expectedLineLength);

            // Remove leading spaces
            bool seenFirstNonSpaceChar = false;

            while (fileHandle->Read(&currentCharacter, 1))
            {
                if (currentCharacter == '\n')
                {
                    break;
                }
                else if (currentCharacter != '\r' && ((currentCharacter != ' ' && currentCharacter != '\t') || seenFirstNonSpaceChar))
                {
                    nextLine.AppendChar(currentCharacter);
                    seenFirstNonSpaceChar = true;
                }
            }

            return !(fileHandle->Tell() == fileHandle->Size());
        }

        static FString DeepCopyFString(FString in)
        {
            FString out;
            out.AppendChars(*in, in.Len());
            return out;
        }
};
