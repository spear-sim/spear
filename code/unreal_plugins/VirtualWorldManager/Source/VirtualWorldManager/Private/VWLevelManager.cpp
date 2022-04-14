#include "VWLevelManager.h"

bool AVWLevelManager::MountPakFromPath(const FString& PakPath)
{
    if (FCoreDelegates::MountPak.IsBound()) {
        if (FCoreDelegates::MountPak.Execute(FPaths::ProjectContentDir() + PakPath, 2)) {
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

void AVWLevelManager::GetAllMapsInPak(TArray<FString>& MapList)
{
    // init FPakPlatformFile
    FPakPlatformFile* PakPlatformFile;
    FString PlatformFileName =
        FPlatformFileManager::Get().GetPlatformFile().GetName();
    if (PlatformFileName.Equals(FString(TEXT("PakFile")))) {
        PakPlatformFile = static_cast<FPakPlatformFile*>(
            &FPlatformFileManager::Get().GetPlatformFile());
    }
    else {
        PakPlatformFile = new FPakPlatformFile;
        if (!PakPlatformFile->Initialize(
                &FPlatformFileManager::Get().GetPlatformFile(), TEXT(""))) {
            UE_LOG(LogTemp, Error, TEXT("[AVWLevelManager] FPakPlatformFile failed to initialize"));
            return;
        }
        FPlatformFileManager::Get().SetPlatformFile(*PakPlatformFile);
    }
    // find all mounted .pak files
    TArray<FString> ArrAllMountedPakFile;
    PakPlatformFile->GetMountedPakFilenames(ArrAllMountedPakFile);
    // find all asset from mounted .pak
    for (auto& PakFilename : ArrAllMountedPakFile) {
        FString PakFilePathFull = FPaths::ConvertRelativePathToFull(PakFilename);
        FPakFile PakFile(PakPlatformFile, *PakFilePathFull, false);
        TArray<FString> FileList;
        FString MountPoint = PakFile.GetMountPoint();
        PakFile.FindFilesAtPath(FileList, *MountPoint, true, false, true);
        for (int32 i = 0; i < FileList.Num(); i++) {
            FString AssetName = FileList[i];
            FString AssetShortName = FPackageName::GetShortName(AssetName);
            FString FileName, FileExt;
            AssetShortName.Split(TEXT("."), &FileName, &FileExt);
            // find all .umap file
            if (FileExt.Equals("umap")) {
                FString NewMapName;
                FString OutFailureReason;
                FPackageName::TryConvertFilenameToLongPackageName(AssetName, NewMapName, &OutFailureReason);
                // add all maps from /Game, ignore maps from /Engine
                if (NewMapName.StartsWith("/Game")) {
                    MapList.Add(NewMapName);
                }
            }
        }
    }
}