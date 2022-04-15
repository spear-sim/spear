// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "PIPCamera.h"
#include "RobotSimHUDWidget.h"
#include "SimMode/SimModeBase.h"

#include "RobotSimHUD.generated.h"

/**
 * @brief
 *
 */
UCLASS()
class ROBOTSIM_API ARobotSimHUD : public AHUD {
    GENERATED_BODY()

public:

    /**
     * @brief 
     * 
     */
    virtual void BeginPlay() override;

private:

    /**
     * @brief 
     * 
     */
    void initializeSettings();

    /**
     * @brief 
     * 
     */
    void setUnrealEngineSettings();

    /**
     * @brief 
     * 
     * @param settingsText 
     * @return true 
     * @return false 
     */
    bool getSettingsTextContent(std::string& settingsText);

    /**
     * @brief 
     * 
     * @param settingsText 
     * @return true 
     * @return false 
     */
    bool getSettingsTextFromCommandLine(std::string& settingsText);

    /**
     * @brief 
     * 
     * @param settingsFilepath 
     * @param settingsText 
     * @return true 
     * @return false 
     */
    bool readSettingsTextFromFile(FString settingsFilepath, std::string& settingsText);

    /**
     * @brief 
     * 
     */
    void createSimMode();

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string getSimModeFromUser();

    UPROPERTY()
    USimHUDWidget* widget_;
    UPROPERTY()
    ASimModeBase* simmode_;
};
