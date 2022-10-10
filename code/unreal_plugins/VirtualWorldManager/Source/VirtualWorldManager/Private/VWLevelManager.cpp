#include "VWLevelManager.h"

#include "Kismet/GameplayStatics.h"

AVWLevelManager::AVWLevelManager()
{
    ConstructorHelpers::FObjectFinder<UMaterialInterface> finder_semantic(
        TEXT("Material'/Game/Koolab/Materials/Segmentation/SegmentationMaterial.SegmentationMaterial'"));
    if (finder_semantic.Succeeded()) {
        this->pp_material_semantic_ = finder_semantic.Object;
    }
    ConstructorHelpers::FObjectFinder<UMaterialInterface> finder_diffuse(
        TEXT("Material'/VirtualWorldManager/Koolab/Materials/CelSHader/PP_Diffuse.PP_Diffuse'"));
    if (finder_diffuse.Succeeded()) {
        this->pp_material_diffuse_ = finder_diffuse.Object;
    }

    ConstructorHelpers::FObjectFinder<UMaterial> finder_cel_shader(
        TEXT("Material'/VirtualWorldManager/Koolab/Materials/CelSHader/PP_CelShader.PP_CelShader'"));
    if (finder_cel_shader.Succeeded()) {
        this->pp_material_cel_shader_ = finder_cel_shader.Object;
    }

    ConstructorHelpers::FObjectFinder<UMaterialInterface> finder_painter(
        TEXT("MaterialInstanceConstant'/VirtualWorldManager/Koolab/Materials/Painter/PPI_Kuwahara.PPI_Kuwahara'"));
    if (finder_painter.Succeeded()) {
        this->pp_material_painter_ = finder_painter.Object;
    }
}

bool AVWLevelManager::mountPakFromPath(const FString& PakPath)
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
    return false;
}

void AVWLevelManager::getAllMapsInPak(TArray<FString>& map_list)
{
    // init FPakPlatformFile
    FPakPlatformFile* pak_platform_file;
    FString platform_file_name =
       FPlatformFileManager::Get().GetPlatformFile().GetName();
    if (platform_file_name.Equals(FString(TEXT("PakFile")))) {
       pak_platform_file = static_cast<FPakPlatformFile*>(
           &FPlatformFileManager::Get().GetPlatformFile());
    }
    else {
       pak_platform_file = new FPakPlatformFile;
       if (!pak_platform_file->Initialize(
               &FPlatformFileManager::Get().GetPlatformFile(), TEXT(""))) {
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
                   map_list.Add(new_map_path);
               }
           }
       }
    }
}

UMaterialInterface* AVWLevelManager::getPostProcessMaterial(EPostProcessMaterialType material_type)
{
    switch (material_type) {
    case Semantic:
        return this->pp_material_semantic_;
    case Diffuse:
        return this->pp_material_diffuse_;
    case CelShader:
        return this->pp_material_cel_shader_;
    case Painter:
        return this->pp_material_painter_;
    default:
        return NULL;
    }
}

void AVWLevelManager::LambertRendering(bool is_enable)
{
    // find all static mesh actors in the scene
    TArray<AActor*> furniture_actors;
    TArray<AActor*> architecure_actors;
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), "Furniture", furniture_actors);
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), "architecture", architecure_actors);
    furniture_actors.Append(architecure_actors);
    // update materials in the scene
    for (auto& actor : furniture_actors) {
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components) {
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);
            for (UMaterialInterface*& mtl : materials) {
                if (is_enable && !mtl->IsA(UMaterialInstanceDynamic::StaticClass())) {
                    // replace current material with MID, and disable metallic,roughness and specular
                    UMaterialInstanceDynamic* DynMaterial = UMaterialInstanceDynamic::Create(mtl, component, FName(mtl->GetName() + "_Dynamic"));
                    DynMaterial->SetVectorParameterValue("Metallic_Color", FVector(0, 0, 0));
                    DynMaterial->SetVectorParameterValue("Gloss_Color", FVector(0, 0, 0));
                    DynMaterial->SetVectorParameterValue("Specular_Color", FVector(0, 0, 0));
                    component->SetMaterial(0, DynMaterial);
                }
                else {
                    // if mid is already setup, switch config
                    UMaterialInstanceDynamic* DynMaterial = static_cast<UMaterialInstanceDynamic*>(mtl);
                    if (DynMaterial != nullptr) {
                        if (is_enable) {
                            // override settings if not exist
                            DynMaterial->SetVectorParameterValue("Metallic_Color", FVector(0, 0, 0));
                            DynMaterial->SetVectorParameterValue("Gloss_Color", FVector(0, 0, 0));
                            DynMaterial->SetVectorParameterValue("Specular_Color", FVector(0, 0, 0));
                        }
                        else {
                            // clear override setting to disable lambert
                            DynMaterial->ClearParameterValues();
                        }
                    }
                }
            }
        }
    }
}