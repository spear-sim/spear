#include "SimModeUrdfBot.h"
//
ASimModeUrdfBot::ASimModeUrdfBot()
{
}

void ASimModeUrdfBot::BeginPlay()
{
    // Will call setupVehiclesAndCamera()
    Super::BeginPlay();

    // this->checkVehicleReady();
    // this->initializePauseState();
    URobotBlueprintLib::BindActionToKey(
        "inputEventCycleCameraBackward", EKeys::P, this,
        &ASimModeUrdfBot::cycleVisibleCameraBackward);

    URobotBlueprintLib::BindActionToKey(
        "SpawnAgent", EKeys::One, this,
        &ASimModeUrdfBot::setupVehiclesAndCamera);

    URobotBlueprintLib::BindActionToKey("DestroyAgent", EKeys::Two, this,
                                        &ASimModeUrdfBot::DestroyPawn);
}

void ASimModeUrdfBot::getExistingVehiclePawns(TArray<RobotBase*>& pawns) const
{
    for (TActorIterator<AUrdfBotPawn> it(this->GetWorld()); it; ++it)
    {
        pawns.Add(static_cast<RobotBase*>(*it));
    }
}

void ASimModeUrdfBot::cycleVisibleCameraForward()
{
    this->cycleVisibleCamera(true);
}

void ASimModeUrdfBot::cycleVisibleCameraBackward()
{
    this->cycleVisibleCamera(false);
}

void ASimModeUrdfBot::cycleVisibleCamera(bool forward)
{
    int currentIndex = this->camera_index_;

    this->camera_index_ += (forward ? 1 : -1);

    if (this->camera_index_ < 0)
    {
        this->camera_index_ = this->cameras_.Num() - 1;
    }
    else if (this->camera_index_ >= this->cameras_.Num())
    {
        this->camera_index_ = 0;
    }

    this->cameras_[currentIndex]->DeactivateCamera();
    this->cameras_[this->camera_index_]->ActivateCamera();
}

void ASimModeUrdfBot::setupVehiclesAndCamera()
{
    FTransform uu_origin = this->getGlobalNedTransform().getGlobalTransform();
    //
    TArray<RobotBase*> pawns;
    getExistingVehiclePawns(pawns);
    if (pawns.Num() > 0)
    {
        URobotBlueprintLib::LogMessage(
            FString("respawn not allowed when robot exist"), "",
            LogDebugLevel::Failure, 30);
        return;
    }
    AUrdfBotPawn* fpv_pawn = nullptr;

    for (auto const& vehicle_setting_pair : this->getSettings().vehicles)
    {
        // std::string vehicle_name = vehicle_setting_pair.first;
        std::string vehicle_name = vehicle_setting_pair.first + "_" +
                                   std::to_string(this->reset_count);
        const auto& vehicle_setting = *vehicle_setting_pair.second;

        if (vehicle_setting.auto_create &&
            vehicle_setting.vehicle_type ==
                RobotSimSettings::kVehicleTypeUrdfBot)
        {
            // Spawn urdf pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.Name = FName(vehicle_name.c_str());
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::
                    AdjustIfPossibleButAlwaysSpawn;
            float scale = 100; // TODO world scale
            FVector spawnPosition = uu_origin.GetTranslation();
            const auto& settings_position = vehicle_setting.position;
            spawnPosition =
                spawnPosition + FVector(settings_position.x() * scale,
                                        settings_position.y() * scale,
                                        settings_position.z() * scale);

            const auto& rotation = vehicle_setting.rotation;
            FRotator spawnRotation =
                uu_origin.GetRotation().Rotator() +
                FRotator(rotation.pitch, rotation.yaw, rotation.roll);
            // whether use NavMesh to find random point from available area
            if (vehicle_setting.enable_random_spawn)
            {
                // random Rz
                spawnRotation =
                    spawnRotation + FRotator(0, FMath::FRandRange(0, 360), 0);

                this->NavSystemRebuild(35);

                ARecastNavMesh* navMesh = GetNavMesh();
                RobotSim::NavMeshUtil::GetRandomPoint(navMesh, spawnPosition,
                                                      20.0f);

                // FBox worldBox = GetWorldBoundingBox();
                // RobotSim::NavMeshUtil::GetRandomPointFromLargestCluster(
                //    navMesh, worldBox, spawnPosition);
            }
            // use BoxTracing to adjust pawn spawn height.
            if (vehicle_setting.enable_spawn_tracing_ground)
            {
                const auto& settings_boundingbox = vehicle_setting.bounding_box;
                FVector boundingBox(settings_boundingbox.x() * scale,
                                    settings_boundingbox.y() * scale,
                                    settings_boundingbox.z() * scale);

                const auto& setting_bb_ofset =
                    vehicle_setting.bounding_box_offset;
                FVector boundingBoxOffset(setting_bb_ofset.x() * scale,
                                          setting_bb_ofset.y() * scale,
                                          setting_bb_ofset.z() * scale);
                FVector BoundingBoxPosition = spawnPosition + boundingBoxOffset;
                traceGround(BoundingBoxPosition, spawnRotation,
                            boundingBox / 2);
                spawnPosition = BoundingBoxPosition - boundingBoxOffset;
            }
            URobotBlueprintLib::LogMessage("spawn at pos",
                                           spawnPosition.ToString() + " ori: " +
                                               spawnRotation.Euler().ToString(),
                                           LogDebugLevel::Informational, 30);
            AUrdfBotPawn* spawned_pawn =
                this->GetWorld()->SpawnActor<AUrdfBotPawn>(
                    spawnPosition, spawnRotation, pawn_spawn_params);

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);

            if (vehicle_setting.is_fpv_vehicle)
                fpv_pawn = spawned_pawn;
        }
    }

    // create API objects for each pawn we have
    this->cameras_.Empty();
    int camera_offset = 0;
    for (RobotBase* pawn : pawns)
    {
        // initialize each vehicle pawn we found
        AUrdfBotPawn* vehicle_pawn = static_cast<AUrdfBotPawn*>(pawn);
        vehicle_pawn->InitializeForBeginPlay();

        // create vehicle sim api
        const auto& ned_transform = getGlobalNedTransform();
        const auto& pawn_ned_pos =
            ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
        const auto& home_geopoint = RobotSim::EarthUtils::nedToGeodetic(
            pawn_ned_pos, getSettings().origin_geopoint);

        RobotSimApi::Params params;
        params.vehicle = vehicle_pawn;
        params.global_transform = &getGlobalNedTransform();
        params.pawn_events = vehicle_pawn->GetPawnEvents();
        params.cameras = vehicle_pawn->GetCameras();
        params.pip_camera_class = pip_camera_class;
        params.collision_display_template = collision_display_template;
        params.home_geopoint = home_geopoint;
        params.vehicle_name = "UrdfBot"; // TODO: This may need to be changed
                                         // for multiple vehicles.

        auto vehicle_sim_api =
            std::unique_ptr<RobotSimApi>(new RobotSimApi(params));

        for (APIPCamera* camera : vehicle_sim_api->getAllCameras())
        {
            int add_index = camera_offset + camera->getIndex();
            while (this->cameras_.Num() <= add_index)
            {
                this->cameras_.Emplace(nullptr);
            }
            this->cameras_[add_index] = camera;
        }
        camera_offset += vehicle_sim_api->getCameraCount();

        std::string vehicle_name = vehicle_sim_api->getVehicleName();
    }

    URobotBlueprintLib::EnableInput(this);
    this->cameras_[0]->ActivateCamera();

    this->reset_count = this->reset_count + 1;
}

void ASimModeUrdfBot::setupRobot(const FTransform& transform)
{
    TArray<RobotBase*> pawns;
    getExistingVehiclePawns(pawns);
    if (pawns.Num() > 0)
    {
        URobotBlueprintLib::LogMessage(
            FString("respawn not allowed when robot exist"), "",
            LogDebugLevel::Failure, 30);
        return;
    }
    AUrdfBotPawn* fpv_pawn = nullptr;

    for (auto const& vehicle_setting_pair : this->getSettings().vehicles)
    {
        std::string vehicle_name = vehicle_setting_pair.first + "_" +
                                   std::to_string(this->reset_count);
        const auto& vehicle_setting = *vehicle_setting_pair.second;

        if (vehicle_setting.auto_create &&
            vehicle_setting.vehicle_type ==
                RobotSimSettings::kVehicleTypeUrdfBot)
        {
            // Spawn vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.Name = FName(vehicle_name.c_str());
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::
                    AdjustIfPossibleButAlwaysSpawn;
            AUrdfBotPawn* spawned_pawn =
                this->GetWorld()->SpawnActor<AUrdfBotPawn>(
                    transform.GetLocation(), transform.Rotator(),
                    pawn_spawn_params);

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);

            if (vehicle_setting.is_fpv_vehicle)
                fpv_pawn = spawned_pawn;
        }
    }

    // create API objects for each pawn we have
    this->cameras_.Empty();
    int camera_offset = 0;
    for (RobotBase* pawn : pawns)
    {
        // initialize each vehicle pawn we found
        AUrdfBotPawn* vehicle_pawn = static_cast<AUrdfBotPawn*>(pawn);
        vehicle_pawn->InitializeForBeginPlay();

        // create vehicle sim api
        const auto& ned_transform = getGlobalNedTransform();
        const auto& pawn_ned_pos =
            ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
        const auto& home_geopoint = RobotSim::EarthUtils::nedToGeodetic(
            pawn_ned_pos, getSettings().origin_geopoint);

        RobotSimApi::Params params;
        params.vehicle = vehicle_pawn;
        params.global_transform = &getGlobalNedTransform();
        params.pawn_events = vehicle_pawn->GetPawnEvents();
        params.cameras = vehicle_pawn->GetCameras();
        params.pip_camera_class = pip_camera_class;
        params.collision_display_template = collision_display_template;
        params.home_geopoint = home_geopoint;
        params.vehicle_name = "UrdfBot"; // TODO: This may need to be changed
                                         // for multiple vehicles.

        auto vehicle_sim_api =
            std::unique_ptr<RobotSimApi>(new RobotSimApi(params));

        for (APIPCamera* camera : vehicle_sim_api->getAllCameras())
        {
            int add_index = camera_offset + camera->getIndex();
            while (this->cameras_.Num() <= add_index)
            {
                this->cameras_.Emplace(nullptr);
            }
            this->cameras_[add_index] = camera;
        }
        camera_offset += vehicle_sim_api->getCameraCount();

        std::string vehicle_name = vehicle_sim_api->getVehicleName();
    }

    URobotBlueprintLib::EnableInput(this);
    // URobotBlueprintLib::BindActionToKey("inputEventCycleCameraForward",
    // EKeys::N, this, &ASimModeUrdfBot::cycleVisibleCameraForward);
    URobotBlueprintLib::BindActionToKey(
        "inputEventCycleCameraBackward", EKeys::P, this,
        &ASimModeUrdfBot::cycleVisibleCameraBackward);
    this->cameras_[0]->ActivateCamera();
    this->reset_count = this->reset_count + 1;
}

void ASimModeUrdfBot::DestroyPawn()
{
    TArray<RobotBase*> pawns;
    getExistingVehiclePawns(pawns);
    for (auto& pawn : pawns)
    {
        AUrdfBotPawn* urdfBotPawn = static_cast<AUrdfBotPawn*>(pawn);
        urdfBotPawn->Destroy();
    }
    // remove cameras
    for (auto& camera : this->cameras_)
    {
        camera->Destroy();
    }
}
