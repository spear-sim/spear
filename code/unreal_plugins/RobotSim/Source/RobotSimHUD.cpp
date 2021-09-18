// Fill out your copyright notice in the Description page of Project Settings.

#include "RobotSimHUD.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/FileHelper.h"

#include "UrdfBot/SimModeUrdfBot.h"

//#include "common_utils/Settings.hpp"
#include "common_utils/RobotSimSettings.hpp"
#include "RobotBlueprintLib.h"
#include <stdexcept>

using namespace RobotSim;
//
//
void ARobotSimHUD::BeginPlay()
{
    Super::BeginPlay();

    try
    {
        URobotBlueprintLib::OnBeginPlay();
        initializeSettings();
        setUnrealEngineSettings();
        createSimMode();
    }
    catch (std::exception& ex)
    {
        URobotBlueprintLib::LogMessageString("Error at startup: ", ex.what(),
                                             LogDebugLevel::Failure);
        URobotBlueprintLib::ShowMessage(
            EAppMsgType::Ok, std::string("Error at startup: ") + ex.what(),
            "Error");
    }
}

void ARobotSimHUD::initializeSettings()
{
    std::string settingsText;
    if (getSettingsTextContent(settingsText))
        RobotSimSettings::initializeSettings(settingsText);
    else
        RobotSimSettings::createDefaultSettingsFile();

    //加载settings.json文件。并且解析相关的配置到arisimsetting。
    RobotSimSettings::singleton().load(
        std::bind(&ARobotSimHUD::getSimModeFromUser, this));
    for (const auto& warning : RobotSimSettings::singleton().warning_messages)
    {
        URobotBlueprintLib::LogMessageString(warning, "",
                                             LogDebugLevel::Failure);
    }
    for (const auto& error : RobotSimSettings::singleton().error_messages)
    {
        URobotBlueprintLib::ShowMessage(EAppMsgType::Ok, error,
                                        "settings.json");
    }
}

void ARobotSimHUD::createSimMode()
{
    std::string simmode_name = RobotSimSettings::singleton().simmode_name;

    FActorSpawnParameters simmode_spawn_params;
    simmode_spawn_params.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    if (simmode_name == "UrdfBot")
        simmode_ = this->GetWorld()->SpawnActor<ASimModeUrdfBot>(
            FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
    else
        URobotBlueprintLib::LogMessageString(
            "SimMode is not valid: ", simmode_name, LogDebugLevel::Failure);
}

bool ARobotSimHUD::getSettingsTextContent(std::string& settingsText)
{
    // FString  CurrentProjectFilePath =
    // RobotSim::Settings::getPorjectDirectoryFullPath("settings.json").c_str();
    FString CurrentProjectFilePath =
        RobotSim::Settings::getAnyPossiblePath("settings.json")
            .c_str();
    if (!FPaths::FileExists(CurrentProjectFilePath))
    {
            throw std::runtime_error(
            "settings.json not found" + std::string(TCHAR_TO_UTF8(*CurrentProjectFilePath)));
	}
    return readSettingsTextFromFile(CurrentProjectFilePath, settingsText);
}

bool ARobotSimHUD::getSettingsTextFromCommandLine(std::string& settingsText)
{

    bool found = false;
    FString settingsTextFString;
    const TCHAR* commandLineArgs = FCommandLine::Get();

    if (FParse::Param(commandLineArgs, TEXT("-settings")))
    {
        FString commandLineArgsFString = FString(commandLineArgs);
        int idx = commandLineArgsFString.Find(TEXT("-settings"));
        FString settingsJsonFString =
            commandLineArgsFString.RightChop(idx + 10);
        if (FParse::QuotedString(*settingsJsonFString, settingsTextFString))
        {
            settingsText = std::string(TCHAR_TO_UTF8(*settingsTextFString));
            found = true;
        }
    }

    return found;
}

bool ARobotSimHUD::readSettingsTextFromFile(FString settingsFilepath,
                                            std::string& settingsText)
{
    bool found = FPaths::FileExists(settingsFilepath);
    if (found)
    {
        FString settingsTextFStr;
        bool readSuccessful =
            FFileHelper::LoadFileToString(settingsTextFStr, *settingsFilepath);
        if (readSuccessful)
        {
            URobotBlueprintLib::LogMessageString(
                "Loaded settings from ", TCHAR_TO_UTF8(*settingsFilepath),
                LogDebugLevel::Informational);
            settingsText = TCHAR_TO_UTF8(*settingsTextFStr);
        }
        else
        {
            URobotBlueprintLib::LogMessageString(
                "Cannot read file ", TCHAR_TO_UTF8(*settingsFilepath),
                LogDebugLevel::Failure);
            throw std::runtime_error("Cannot read settings file.");
        }
    }

    return found;
}

std::string ARobotSimHUD::getSimModeFromUser()
{
    return "UrdfBot";
}
void ARobotSimHUD::setUnrealEngineSettings()
{
    // TODO: should we only do below on SceneCapture2D components and cameras?
    // avoid motion blur so capture images don't get
    GetWorld()->GetGameViewport()->GetEngineShowFlags()->SetMotionBlur(false);

    // use two different methods to set console var because sometime it doesn't
    // seem to work
    static const auto custom_depth_var =
        IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    custom_depth_var->Set(3);

    // Equivalent to enabling Custom Stencil in Project > Settings > Rendering >
    // Postprocessing
    UKismetSystemLibrary::ExecuteConsoleCommand(GetWorld(),
                                                FString("r.CustomDepth 3"));

    // during startup we init stencil IDs to random hash and it takes long time
    // for large environments we get error that GameThread has timed out after
    // 30 sec waiting on render thread
    static const auto render_timeout_var =
        IConsoleManager::Get().FindConsoleVariable(
            TEXT("g.TimeoutForBlockOnRenderFence"));
    render_timeout_var->Set(300000);
}