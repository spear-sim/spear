#include "SimModeSimpleVehicle.h"
//
ASimModeSimpleVehicle::ASimModeSimpleVehicle()
{
}

void ASimModeSimpleVehicle::BeginPlay()
{
    // Will call setupVehiclesAndCamera()
    Super::BeginPlay();
    URobotBlueprintLib::BindActionToKey(
        "test", EKeys::One, this,
        &ASimModeSimpleVehicle::setupVehiclesAndCamera);

    URobotBlueprintLib::BindActionToKey("test2", EKeys::Two, this,
                                        &ASimModeSimpleVehicle::DestroyPawn);
    URobotBlueprintLib::BindActionToKey(
        "inputEventCycleCameraForward", EKeys::N, this,
        &ASimModeSimpleVehicle::cycleVisibleCameraForward);
    URobotBlueprintLib::BindActionToKey(
        "inputEventCycleCameraBackward", EKeys::P, this,
        &ASimModeSimpleVehicle::cycleVisibleCameraBackward);

    URobotBlueprintLib::BindActionToKey("test", EKeys::Three, this,
                                        &ASimModeSimpleVehicle::Test);
}

void ASimModeSimpleVehicle::getExistingVehiclePawns(
    TArray<RobotBase*>& pawns) const
{
    for (TActorIterator<ASimpleVehiclePawn> it(this->GetWorld()); it; ++it)
    {
        pawns.Add(static_cast<RobotBase*>(*it));
    }
}

void ASimModeSimpleVehicle::cycleVisibleCameraForward()
{
    this->cycleVisibleCamera(true);
}

void ASimModeSimpleVehicle::cycleVisibleCameraBackward()
{
    this->cycleVisibleCamera(false);
}

void ASimModeSimpleVehicle::cycleVisibleCamera(bool forward)
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

// TODO what is this?
void ASimModeSimpleVehicle::setupVehiclesAndCamera()
{
    FTransform uu_origin = this->getGlobalNedTransform().getGlobalTransform();

    TArray<RobotBase*> pawns;
    getExistingVehiclePawns(pawns);
    if (pawns.Num() > 0)
    {
        UE_LOG(LogTemp, Warning,
               TEXT("ASimModeSimpleVehicle::setupVehiclesAndCamera "
                    "pawns.length()>0"));
        return;
    }
    ASimpleVehiclePawn* fpv_pawn = nullptr;

    for (auto const& vehicle_setting_pair : this->getSettings().vehicles)
    {
        // std::string vehicle_name = vehicle_setting_pair.first;
        std::string vehicle_name = vehicle_setting_pair.first + "_" +
                                   std::to_string(this->reset_count);
        const auto& vehicle_setting = *vehicle_setting_pair.second;

        if (vehicle_setting.auto_create &&
            vehicle_setting.vehicle_type ==
                RobotSimSettings::kVehicleTypeSimpleVehicle)
        {
            // Spawn setting
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.Name = FName(vehicle_name.c_str());
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::
                    AdjustIfPossibleButAlwaysSpawn;

            std::string PawnBP =
                getSettings().pawn_paths.at("SimpleVehicle").pawn_bp;
            UClass* vehicle_bp_class;
            if (PawnBP.length() > 0)
            {
                vehicle_bp_class = URobotBlueprintLib::LoadClass(PawnBP);
            }
            else
            {
                vehicle_bp_class = ASimpleVehiclePawn::StaticClass();
            }

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

            ASimpleVehiclePawn* defaultObject =
                static_cast<ASimpleVehiclePawn*>(
                    vehicle_bp_class->GetDefaultObject());
            USkeletalMeshComponent* mesh = defaultObject->GetMesh();
            FBoxSphereBounds bound = mesh->SkeletalMesh->GetBounds();

            if (vehicle_setting.enable_random_spawn)
            {
                // random Rz
                spawnRotation =
                    spawnRotation + FRotator(0, FMath::FRandRange(0, 360), 0);

                this->NavSystemRebuild(bound.SphereRadius);

                ARecastNavMesh* navMesh = GetNavMesh();
                RobotSim::NavMeshUtil::GetRandomPoint(navMesh, spawnPosition,
                                                      20.0f);

                // FBox worldBox = GetWorldBoundingBox();
                // RobotSim::NavMeshUtil::GetRandomPointFromLargestCluster(
                //    navMesh, worldBox, spawnPosition);
            }
            if (vehicle_setting.enable_spawn_tracing_ground)
            {
                // use BoxTracing to adjust pawn spawn height.
                // use mesh bounding box instead of setting

                FVector center = bound.GetBox().GetCenter();
                traceGround(spawnPosition, spawnRotation,
                            bound.GetBox().GetSize() / 2);
                spawnPosition = spawnPosition + FVector(0, 0, -3);
            }
            URobotBlueprintLib::LogMessage("spawn at pos",
                                           spawnPosition.ToString() + " ori: " +
                                               spawnRotation.Euler().ToString(),
                                           LogDebugLevel::Informational, 30);
            ASimpleVehiclePawn* spawned_pawn =
                static_cast<ASimpleVehiclePawn*>(this->GetWorld()->SpawnActor(
                    vehicle_bp_class, &spawnPosition, &spawnRotation,
                    pawn_spawn_params));

            // Apply settings from the parameter file:
            spawned_pawn->SetRobotParameters(vehicle_setting);

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
        ASimpleVehiclePawn* vehicle_pawn =
            static_cast<ASimpleVehiclePawn*>(pawn);

        // Note Quentin: uncomment to activate keyboard
        if (getSettings().pawn_paths.at("SimpleVehicle").enable_keyboard)
        {
            vehicle_pawn->SetupInputBindings();
        }

        // create vehicle sim api
        const auto& ned_transform = getGlobalNedTransform();
        const auto& pawn_ned_pos =
            ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
        const auto& home_geopoint = RobotSim::EarthUtils::nedToGeodetic(
            pawn_ned_pos, getSettings().origin_geopoint);

        RobotSimApiBase::Params params;
        params.vehicle = vehicle_pawn;
        params.global_transform = &getGlobalNedTransform();
        // collision info
        params.pawn_events = vehicle_pawn->GetPawnEvents();
        params.pip_camera_class = pip_camera_class;
        params.collision_display_template = collision_display_template;
        params.home_geopoint = home_geopoint;
        params.vehicle_name = "SimpleVehicle"; // TODO: This may need to be
                                               // changed for multiple vehicles.

        auto vehicle_sim_api = std::unique_ptr<SimpleVehicleSimApi>(
            new SimpleVehicleSimApi(params));

        for (APIPCamera* camera : vehicle_sim_api->getAllCameras())
        {
            int add_index = camera_offset + camera->GetIndex();
            while (this->cameras_.Num() <= add_index)
            {
                this->cameras_.Emplace(nullptr);
            }
            this->cameras_[add_index] = camera;
        }
        camera_offset += vehicle_sim_api->getCameraCount();

        std::string vehicle_name = vehicle_sim_api->getVehicleName();
    }

    this->cameras_[0]->ActivateCamera();

    this->reset_count = this->reset_count + 1;
}

void ASimModeSimpleVehicle::DestroyPawn()
{
    TArray<RobotBase*> pawns;
    getExistingVehiclePawns(pawns);
    for (auto& pawn : pawns)
    {
        ASimpleVehiclePawn* simpleVehiclePawn =
            static_cast<ASimpleVehiclePawn*>(pawn);
        simpleVehiclePawn->Destroy();
    }
    // remove cameras
    for (auto& camera : this->cameras_)
    {
        camera->Destroy();
    }
    this->cameras_.Empty();
    int camera_offset = 0;
}
