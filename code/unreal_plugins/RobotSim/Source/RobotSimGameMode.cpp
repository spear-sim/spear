// Fill out your copyright notice in the Description page of Project Settings.

#include "RobotSimGameMode.h"
#include "RobotSimHUD.h"

#include "Misc/FileHelper.h"

void ARobotSimGameMode::StartPlay()
{
    Super::StartPlay();
}

ARobotSimGameMode::ARobotSimGameMode(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    DefaultPawnClass = nullptr;
    HUDClass = ARobotSimHUD::StaticClass();
}
