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
}

void ASimModeUrdfBot::getExistingVehiclePawns(
    TArray<RobotBase*>& pawns) const
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

    this->cameras_[currentIndex]->disableMain();
    this->cameras_[this->camera_index_]->showToScreen();
}

void ASimModeUrdfBot::setupVehiclesAndCamera()
{
    FTransform uu_origin = this->getGlobalNedTransform().getGlobalTransform();
    //
    TArray<RobotBase*> pawns;
    getExistingVehiclePawns(pawns);

    AUrdfBotPawn* fpv_pawn = nullptr;

    for (auto const& vehicle_setting_pair : this->getSettings().vehicles)
    {
        std::string vehicle_name = vehicle_setting_pair.first;
        const auto& vehicle_setting = *vehicle_setting_pair.second;

        if (vehicle_setting.auto_create &&
            vehicle_setting.vehicle_type ==
                RobotSimSettings::kVehicleTypeUrdfBot)
        {

            // Compute initial pose
            FVector spawn_position = uu_origin.GetLocation();
            FRotator spawn_rotation = uu_origin.Rotator();
            RobotSim::Vector3r settings_position = vehicle_setting.position;
            if (!RobotSim::VectorMath::hasNan(settings_position))
                spawn_position =
                    getGlobalNedTransform().fromLocalNed(settings_position);
            const auto& rotation = vehicle_setting.rotation;
            if (!std::isnan(rotation.yaw))
                spawn_rotation.Yaw = rotation.yaw;
            if (!std::isnan(rotation.pitch))
                spawn_rotation.Pitch = rotation.pitch;
            if (!std::isnan(rotation.roll))
                spawn_rotation.Roll = rotation.roll;

            // Spawn vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.Name =
                FName(vehicle_setting.vehicle_name.c_str());
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::
                    AdjustIfPossibleButAlwaysSpawn;
            AUrdfBotPawn* spawned_pawn =
                this->GetWorld()->SpawnActor<AUrdfBotPawn>(
                    spawn_position, spawn_rotation, pawn_spawn_params);

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

        auto vehicle_api = vehicle_sim_api->getVehicleApi();
        auto vehicle_sim_api_p = vehicle_sim_api.get();
        // getApiProvider()->insert_or_assign(vehicle_name, vehicle_api,
        // vehicle_sim_api_p);
        /*if ((fpv_pawn == vehicle_pawn ||
           !getApiProvider()->hasDefaultVehicle()) && vehicle_name != "")
                getApiProvider()->makeDefaultVehicle(vehicle_name);*/

        vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));
    }

    // if (getApiProvider()->hasDefaultVehicle()) {
    //	getVehicleSimApi()->possess();
    //}

    // Set up camera cycling
    // if (this->cameras_.Num() < 1) {
    //    throw std::runtime_error("UrdfBotPawn has no cameras defined. Please
    //    define at least one camera in settings.json");
    //}

    URobotBlueprintLib::EnableInput(this);
    // URobotBlueprintLib::BindActionToKey("inputEventCycleCameraForward",
    // EKeys::N, this, &ASimModeUrdfBot::cycleVisibleCameraForward);
    URobotBlueprintLib::BindActionToKey(
        "inputEventCycleCameraBackward", EKeys::P, this,
        &ASimModeUrdfBot::cycleVisibleCameraBackward);
    this->cameras_[0]->showToScreen();

    for (auto& api : vehicle_sim_apis_)
    {
        api->reset();
    }
}
