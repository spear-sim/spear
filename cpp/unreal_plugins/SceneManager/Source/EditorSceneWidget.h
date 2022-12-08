#pragma once

#include <CoreMinimal.h>
#include <Editor/Blutility/Classes/EditorUtilityWidget.h>
#include <Widgets/DeclarativeSyntaxSupport.h>

#include "EditorSceneWidget.generated.h"

/**
 *
 */
UCLASS(BlueprintType)
class SCENEMANAGER_API UEditorSceneWidget : public UEditorUtilityWidget
{
	GENERATED_BODY()

public:
	UEditorSceneWidget();
	~UEditorSceneWidget();

	UFUNCTION(BlueprintCallable, Category = mesh_merge)
		bool RunMerge(const FString& destination_path, bool bReplaceSourceActors);

	UPROPERTY(EditAnywhere, meta = (ShowOnlyInnerProperties), Category = mesh_merge)
		FMeshMergingSettings merge_settings_;
};