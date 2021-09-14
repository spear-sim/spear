// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "RobotSimHUDWidget.h"
#include "SimMode/SimModeBase.h"
#include "PIPCamera.h"
#include <memory>
#include "RobotSimHUD.generated.h"

/**
 *
 */
UCLASS()
class ROBOTSIM_API ARobotSimHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;

private:
    void initializeSettings();
    void setUnrealEngineSettings();

    bool getSettingsTextContent(std::string& settingsText);
    bool getSettingsTextFromCommandLine(std::string& settingsText);
    bool readSettingsTextFromFile(FString settingsFilepath,
                                  std::string& settingsText);
    void createSimMode();

    std::string getSimModeFromUser();
    UPROPERTY() USimHUDWidget* widget_;
    UPROPERTY() ASimModeBase* simmode_;
};
