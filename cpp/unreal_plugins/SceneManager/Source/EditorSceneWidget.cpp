#include "EditorSceneWidget.h"

#include <AssetRegistryModule.h>
#include <ContentBrowserModule.h>
#include <Engine/MeshMerging.h>
#include <Engine/Selection.h>
#include <IContentBrowserSingleton.h>
#include <MeshMergeModule.h>
#include <MeshUtilities.h>
#include <Misc/ScopedSlowTask.h>

UEditorSceneWidget::UEditorSceneWidget()
{
}

UEditorSceneWidget::~UEditorSceneWidget()
{
}

//FReply UEditorSceneWidget::MergeActorsClicked()
//{
//	if (CurrentlySelectedTool >= 0 && CurrentlySelectedTool < RegisteredTools.Num())
//	{
//		const FString DefaultPackageName = RegisteredTools[CurrentlySelectedTool]->GetDefaultPackageName();
//		if (DefaultPackageName.Len() > 0)
//		{
//			const FString DefaultPath = FPackageName::GetLongPackagePath(DefaultPackageName);
//			const FString DefaultName = FPackageName::GetShortName(DefaultPackageName);
//
//			// Initialize SaveAssetDialog config
//			FSaveAssetDialogConfig SaveAssetDialogConfig;
//			SaveAssetDialogConfig.DialogTitleOverride = LOCTEXT("CreateMergedActorTitle", "Create Merged Actor");
//			SaveAssetDialogConfig.DefaultPath = DefaultPath;
//			SaveAssetDialogConfig.DefaultAssetName = DefaultName;
//			SaveAssetDialogConfig.ExistingAssetPolicy = ESaveAssetDialogExistingAssetPolicy::AllowButWarn;
//			SaveAssetDialogConfig.AssetClassNames = { UStaticMesh::StaticClass()->GetFName() };
//
//			FContentBrowserModule& ContentBrowserModule = FModuleManager::LoadModuleChecked<FContentBrowserModule>("ContentBrowser");
//			FString SaveObjectPath = ContentBrowserModule.Get().CreateModalSaveAssetDialog(SaveAssetDialogConfig);
//			if (!SaveObjectPath.IsEmpty())
//			{
//				const FString PackageName = FPackageName::ObjectPathToPackageName(SaveObjectPath);
//
//				RegisteredTools[CurrentlySelectedTool]->RunMerge(PackageName);
//			}
//		}
//		else
//		{
//			RegisteredTools[CurrentlySelectedTool]->RunMerge(DefaultPackageName);
//		}
//	}
//
//	return FReply::Handled();
//}

bool UEditorSceneWidget::RunMerge(const FString& destination_path, bool bReplaceSourceActors)
{
	const IMeshMergeUtilities& mesh_utilities = FModuleManager::Get().LoadModuleChecked<IMeshMergeModule>("MeshMergeUtilities").GetUtilities();
	USelection* selected_actors = GEditor->GetSelectedActors();
	TArray<AActor*> actors;
	TArray<ULevel*> unique_levels;
	TArray<UPrimitiveComponent*> merge_components;

	FString package_name;
	if (destination_path == "") {
		package_name = FPackageName::FilenameToLongPackageName(FPaths::ProjectContentDir() + TEXT("MERGED_"));
	}
	else {
		package_name = FPackageName::FilenameToLongPackageName(FPaths::ProjectContentDir() + destination_path + TEXT("MERGED_"));
	}

	bool name_selected = false;

	for (FSelectionIterator Iter(*selected_actors); Iter; ++Iter)
	{
		AActor* actor = Cast<AActor>(*Iter);
		if (actor)
		{
			if (!name_selected) {
				FString name = actor->GetName();
				package_name = FString::Printf(TEXT("%s_%s"), *package_name, *name);

				if (package_name.IsEmpty())
				{
					package_name = MakeUniqueObjectName(NULL, UPackage::StaticClass(), *package_name).ToString();
				}

				name_selected = true;
			}

			USceneComponent* root = actor->GetRootComponent();
			UPrimitiveComponent* primitive = Cast<UPrimitiveComponent>(root);
			if (primitive != nullptr) {
				merge_components.Add(primitive);
			}

			TArray<USceneComponent*> childrens;
			root->GetChildrenComponents(true, childrens);

			for (USceneComponent* component : childrens) {
				UPrimitiveComponent* primitive_children = Cast<UPrimitiveComponent>(component);
				if (primitive_children != nullptr) {
					merge_components.Add(primitive_children);
				}
			}

			actors.Add(actor);
			unique_levels.AddUnique(actor->GetLevel());
		}
	}

	// This restriction is only for replacement of selected actors with merged mesh actor
	//if (UniqueLevels.Num() > 1 && bReplaceSourceActors)
	//{
	//	FText Message = NSLOCTEXT("UnrealEd", "FailedToMergeActorsSublevels_Msg", "The selected actors should be in the same level");
	//	const FText Title = NSLOCTEXT("UnrealEd", "FailedToMergeActors_Title", "Unable to merge actors");
	//	FMessageDialog::Open(EAppMsgType::Ok, Message, &Title);
	//	return false;
	//}

	FVector merged_actor_location;
	TArray<UObject*> assets_to_sync;

	// Merge...
	if (merge_components.Num())
	{
		UWorld* world = merge_components[0]->GetWorld();
		checkf(world != nullptr, TEXT("Invalid World retrieved from Mesh components"));
		const float screen_area_size = TNumericLimits<float>::Max();

		// If the merge destination package already exists, it is possible that the mesh is already used in a scene somewhere, or its materials or even just its textures.
		// Static primitives uniform buffers could become invalid after the operation completes and lead to memory corruption. To avoid it, we force a global reregister.
		if (FindObject<UObject>(nullptr, *package_name))
		{
			FGlobalComponentReregisterContext global_reregister;
			mesh_utilities.MergeComponentsToStaticMesh(merge_components, world, merge_settings_, nullptr, nullptr, package_name, assets_to_sync, merged_actor_location, screen_area_size, true);
		}
		else
		{
			mesh_utilities.MergeComponentsToStaticMesh(merge_components, world, merge_settings_, nullptr, nullptr, package_name, assets_to_sync, merged_actor_location, screen_area_size, true);
		}
	}

	if (assets_to_sync.Num())
	{
		FAssetRegistryModule& asset_registry = FModuleManager::Get().LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
		int32 asset_count = assets_to_sync.Num();
		for (int32 asset_index = 0; asset_index < asset_count; asset_index++)
		{
			asset_registry.AssetCreated(assets_to_sync[asset_index]);
			GEditor->BroadcastObjectReimported(assets_to_sync[asset_index]);
		}

		//Also notify the content browser that the new assets exists
		FContentBrowserModule& ContentBrowserModule = FModuleManager::Get().LoadModuleChecked<FContentBrowserModule>("ContentBrowser");
		ContentBrowserModule.Get().SyncBrowserToAssets(assets_to_sync, true);

		// Place new mesh in the world -- replace code --
		//if (bReplaceSourceActors)
		//{
		//	UStaticMesh* MergedMesh = nullptr;
		//	if (assets_to_sync.FindItemByClass(&MergedMesh))
		//	{
		//		const FScopedTransaction Transaction(LOCTEXT("PlaceMergedActor", "Place Merged Actor"));
		//		UniqueLevels[0]->Modify();

		//		UWorld* World = UniqueLevels[0]->OwningWorld;
		//		FActorSpawnParameters Params;
		//		Params.OverrideLevel = UniqueLevels[0];
		//		FRotator MergedActorRotation(ForceInit);

		//		AStaticMeshActor* MergedActor = World->SpawnActor<AStaticMeshActor>(MergedActorLocation, MergedActorRotation, Params);
		//		MergedActor->GetStaticMeshComponent()->SetStaticMesh(MergedMesh);
		//		MergedActor->SetActorLabel(MergedMesh->GetName());
		//		World->UpdateCullDistanceVolumes(MergedActor, MergedActor->GetStaticMeshComponent());
		//		GEditor->SelectNone(true, true);
		//		GEditor->SelectActor(MergedActor, true, true);
		//		// Remove source actors
		//		for (AActor* Actor : Actors)
		//		{
		//			Actor->Destroy();
		//		}
		//	}
		//}
	}

	return true;
}
