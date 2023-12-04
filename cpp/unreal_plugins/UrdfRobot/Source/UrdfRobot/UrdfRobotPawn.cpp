//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfRobot/UrdfRobotPawn.h"

#include <Camera/CameraComponent.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <UObject/UObjectGlobals.h> // NewObject

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfParser.h"
#include "UrdfRobot/UrdfRobotComponent.h"

const auto DEFAULT_URDF_FILE = std::filesystem::path() / ".." / ".." / ".." / "python" / "spear" / "urdf" / "horizontal_pendulum.urdf";

AUrdfRobotPawn::AUrdfRobotPawn(const FObjectInitializer& object_initializer) : APawn(object_initializer)
{
    SP_LOG_CURRENT_FUNCTION();

    // UUrdfRobotComponent
    UrdfRobotComponent = CreateDefaultSubobject<UUrdfRobotComponent>(Unreal::toFName("urdf_robot_component"));
    SP_ASSERT(UrdfRobotComponent);
    SetRootComponent(UrdfRobotComponent);

    // UCameraComponent
    CameraComponent = CreateDefaultSubobject<UCameraComponent>(Unreal::toFName("camera_component"));
    SP_ASSERT(CameraComponent);
    CameraComponent->SetupAttachment(UrdfRobotComponent);
}

AUrdfRobotPawn::~AUrdfRobotPawn()
{
    SP_LOG_CURRENT_FUNCTION();

    // Pawns don't need to be cleaned up explicitly.

    SP_ASSERT(CameraComponent);
    CameraComponent = nullptr;

    SP_ASSERT(UrdfRobotComponent);
    UrdfRobotComponent = nullptr;

    UrdfFile = Unreal::toFString("");
}

void AUrdfRobotPawn::Initialize()
{
    // parse URDF file
    std::filesystem::path urdf_file;
    if (Config::s_initialized_) {
        urdf_file =
            std::filesystem::path() /
            Config::get<std::string>("URDF_ROBOT.URDF_ROBOT_PAWN.URDF_DIR") /
            Config::get<std::string>("URDF_ROBOT.URDF_ROBOT_PAWN.URDF_FILE");
    } else if (std::filesystem::exists(Unreal::toStdString(UrdfFile))) {
        urdf_file = Unreal::toStdString(UrdfFile);
    } else if (WITH_EDITOR) { // defined in an auto-generated header
        urdf_file = std::filesystem::canonical(Unreal::toStdString(FPaths::ProjectDir()) / DEFAULT_URDF_FILE);
    }
    SP_ASSERT(std::filesystem::exists(urdf_file));

    UrdfRobotDesc robot_desc = UrdfParser::parse(urdf_file.string());
    SP_ASSERT(!Std::containsSubstring(robot_desc.name_, "."));

    // UrdfRobotComponent
    UrdfRobotComponent->initialize(&robot_desc);

    // UCameraComponent
    FVector camera_location;
    FRotator camera_rotation;
    float field_of_view;
    float aspect_ratio;
    if (Config::s_initialized_) {
        camera_location = {
            Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.LOCATION_X"),
            Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.LOCATION_Y"),
            Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.LOCATION_Z")};
        camera_rotation = {
            Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ROTATION_PITCH"),
            Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ROTATION_YAW"),
            Config::get<double>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ROTATION_ROLL")};
        field_of_view = Config::get<float>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.FOV");
        aspect_ratio = Config::get<float>("URDF_ROBOT.URDF_ROBOT_PAWN.CAMERA_COMPONENT.ASPECT_RATIO");
    } else {
        // Fetch defaults, see python/spear/config/default_config.urdf_robot.yaml
        camera_location = {0.0, 0.0, 50.0};
        camera_rotation = FRotator::ZeroRotator;
        field_of_view = 90.0;
        aspect_ratio = 1.0;
    }

    CameraComponent->SetRelativeLocationAndRotation(camera_location, camera_rotation);
    CameraComponent->bUsePawnControlRotation = false;
    CameraComponent->FieldOfView = field_of_view;
    CameraComponent->AspectRatio = aspect_ratio;
    CameraComponent->AttachToComponent(UrdfRobotComponent, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
}
