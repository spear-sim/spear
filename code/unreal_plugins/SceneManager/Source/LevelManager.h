#pragma once

#include <string>
#include <vector>

#include <CoreMinimal.h>
#include <IPlatformFilePak.h>

class SCENEMANAGER_API LevelManager
{
public:
    // load DLC from external .pak files
    static bool mountPakFromPath(const std::string& pak_file_path);
    // find all available levels in mounted paks.
    static void getAllMapsInPak(std::vector<std::string>& map_list);
};
