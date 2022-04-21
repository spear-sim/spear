#pragma once

#include <functional>

#include "Blueprint/UserWidget.h"
#include "CoreMinimal.h"
#include "PIPCamera.h"
#include "RobotSimHUDWidget.generated.h"

UCLASS()
class ROBOTSIM_API USimHUDWidget : public UUserWidget {
    GENERATED_BODY()

public:
    /**
     * @brief
     *
     */
    UFUNCTION(BlueprintCallable, Category = "Event handler")
    void onToggleRecordingButtonClick();

public:
    typedef std::function<void(void)> OnToggleRecording;

    // TODO: Tick is not working virtual void Tick_Implementation(FGeometry MyGeometry, float InDeltaTime) override;

    /**
     * @brief
     *
     * @param text
     */
    void updateDebugReport(const std::string& text);

    /**
     * @brief
     *
     * @param is_visible
     */
    void setReportVisible(bool is_visible);

    /**
     * @brief
     *
     */
    void toggleHelpVisibility();

    /**
     * @brief
     *
     * @param handler
     */
    void setOnToggleRecordingHandler(OnToggleRecording handler);

public:
    // Below are implemented in Blueprint. The return value is forced to be
    // bool even when not needed because of Unreal quirk that if return value
    // is not there then below are treated as events instead of overridable
    // functions

    /**
     * @brief
     *
     * @param window_index
     * @param is_visible
     * @param render_target
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setSubwindowVisibility(int window_index,
                                bool is_visible,
                                UTextureRenderTarget2D* render_target);

    /**
     * @brief
     *
     * @param window_index
     * @return int
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    int getSubwindowVisibility(int window_index);

    /**
     * @brief
     *
     * @param is_visible
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setRecordButtonVisibility(bool is_visible);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getRecordButtonVisibility();

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool initializeForPlay();

protected:
    /**
     * @brief
     *
     * @param is_visible
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setReportContainerVisibility(bool is_visible);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getReportContainerVisibility();

    /**
     * @brief
     *
     * @param is_visible
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setHelpContainerVisibility(bool is_visible);

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool getHelpContainerVisibility();

    /**
     * @brief
     *
     * @param text
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
    bool setReportText(const FString& text);

private:
    OnToggleRecording on_toggle_recording_;
};
