// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameUserSettings.h"
#include "RobotSimGameMode.generated.h"

/**
 * @brief
 *
 */
UCLASS()
class ROBOTSIM_API ARobotSimGameMode : public AGameModeBase {
    GENERATED_BODY()
public:
    /**
     * @brief
     *
     */
    virtual void StartPlay() override;

    /**
     * @brief 
     *
     * @param ObjectInitializer
     */
    ARobotSimGameMode(const FObjectInitializer& ObjectInitializer);
};