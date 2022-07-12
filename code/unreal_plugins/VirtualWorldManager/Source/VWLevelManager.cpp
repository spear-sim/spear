#include "VWLevelManager.h"

bool VWLevelManager::mountPakFromPath(const std::string& PakPath)
{
    if (FCoreDelegates::MountPak.IsBound()) {
        if (FCoreDelegates::MountPak.Execute(FString(PakPath.c_str()), 2)) {
            UE_LOG(LogTemp, Warning, TEXT("[AVWLevelManager] MountPak success"));
            return true;
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("[AVWLevelManager] MountPak Failed"));
            return false;
        }
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("[AVWLevelManager] OnMountPak.IsBound() Failed"));
        return false;
    }
}

void VWLevelManager::getAllMapsInPak(std::vector<std::string>& map_list)
{
    // init FPakPlatformFile
    FPakPlatformFile* pak_platform_file;
    FString platform_file_name = FPlatformFileManager::Get().GetPlatformFile().GetName();
    UE_LOG(LogTemp, Warning, TEXT("[AVWLevelManager] platform_file_name %s"),*platform_file_name);
    if (platform_file_name.Equals(FString(TEXT("PakFile")))) {
        UE_LOG(LogTemp, Warning, TEXT("[AVWLevelManager] FPakPlatformFile exist"));
        pak_platform_file = static_cast<FPakPlatformFile*>(&FPlatformFileManager::Get().GetPlatformFile());
    }
    else {
        pak_platform_file = new FPakPlatformFile;
            UE_LOG(LogTemp, Error, TEXT("[AVWLevelManager] FPakPlatformFile initialize"));
        if (!pak_platform_file->Initialize(&FPlatformFileManager::Get().GetPlatformFile(), TEXT(""))) {
            UE_LOG(LogTemp, Error, TEXT("[AVWLevelManager] FPakPlatformFile failed to initialize"));
            return;
        }
        FPlatformFileManager::Get().SetPlatformFile(*pak_platform_file);
    }
    // find all mounted .pak files
    TArray<FString> all_mounted_pak_files;
    pak_platform_file->GetMountedPakFilenames(all_mounted_pak_files);
    // find all asset from mounted .pak
    for (auto& pak_file_name : all_mounted_pak_files) {
        FString pak_file_name_full = FPaths::ConvertRelativePathToFull(pak_file_name);
        FPakFile pak_file(pak_platform_file, *pak_file_name_full, false);
        TArray<FString> file_list;
        FString MountPoint = pak_file.GetMountPoint();
        pak_file.FindFilesAtPath(file_list, *MountPoint, true, false, true);
        for (int32 i = 0; i < file_list.Num(); i++) {
            FString asset_name = file_list[i];
            FString asset_short_name = FPackageName::GetShortName(asset_name);
            FString file_name, file_ext;
            asset_short_name.Split(TEXT("."), &file_name, &file_ext);
            // find all .umap file
            if (file_ext.Equals("umap")) {
                FString new_map_path;
                FString failure_reason;
                FPackageName::TryConvertFilenameToLongPackageName(asset_name, new_map_path, &failure_reason);
                // add all maps from /Game, ignore maps from /Engine
                if (new_map_path.StartsWith("/Game")) {
                    map_list.emplace_back(TCHAR_TO_UTF8(*new_map_path));
                }
            }
        }
    }
}
