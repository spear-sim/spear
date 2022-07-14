#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/ScriptInterface.h"
#include "Engine/BlendableInterface.h"
#include "Camera/CameraTypes.h"
#include "Components/SceneCaptureComponent2D.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "PostProcessCaptureComponent2D.generated.h"

UCLASS(hidecategories=(Collision, Object, Physics, SceneComponent), ClassGroup=Rendering, editinlinenew, meta=(BlueprintSpawnableComponent))
class UPostProcessCaptureComponent2D : public USceneCaptureComponent2D
{
	GENERATED_BODY()
		
public:
	UPostProcessCaptureComponent2D(const FObjectInitializer &ObjectInitializer);

	bool bEnablePostProcessingEffects = true;

	bool AddPostProcessingMaterial(const FString &Path);

	void SetPostProcessBlendables();
	void AddPostProcessBlendable(UMaterial* mat);

	bool ActivateBlendablePass(uint8 pass_id);
	bool ActivateBlendablePass(std::string pass_name);

	void SetCameraComponent();

    virtual void OnComponentDestroyed(bool bDestroyingHierarchy) override;
	virtual void OnRegister() override;
	virtual void SendRenderTransform_Concurrent() override;

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

private:

	TArray<UMaterial*> materialsFound;

	void SetCameraDefaultOverrides();
	void ConfigureShowFlags(bool bPostProcessing);
};