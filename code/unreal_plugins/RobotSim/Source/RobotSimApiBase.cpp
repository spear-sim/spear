#include "RobotSimApiBase.h"
#include "Engine/World.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Camera/CameraComponent.h"

#include "RobotBlueprintLib.h"
#include "PIPCamera.h"
#include "NedTransform.h"
#include "common_utils/EarthUtils.hpp"

void RobotSimApiBase::updateRenderedState(float dt)
{
    updateKinematics(dt);
}

void RobotSimApiBase::updateRendering(float dt)
{
    unused(dt);
}

std::string RobotSimApiBase::getRecordFileLine(bool is_header_line) const
{
    // TODO: Do something acutally useful here.
    return "Not implemented yet. Whee!\n";
}

//#include "DrawDebugHelpers.h"

RobotSimApiBase::RobotSimApiBase(Params params)
    : params_(params),
      ned_transform_(params.vehicle->GetPawn(), *params.global_transform)
{
    RobotSim::Environment::State initial_environment;
    initial_environment.position = getPose().position;
    initial_environment.geo_point = params_.home_geopoint;
    environment_.reset(new RobotSim::Environment(initial_environment));

    // initialize state
    params_.vehicle->GetPawn()->GetActorBounds(true, initial_state_.mesh_origin,
                                               initial_state_.mesh_bounds);
    initial_state_.ground_offset = FVector(0, 0, initial_state_.mesh_bounds.Z);
    initial_state_.transformation_offset =
        params_.vehicle->GetPawn()->GetActorLocation() -
        initial_state_.ground_offset;
    ground_margin_ =
        FVector(0, 0, 20); // TODO: can we explain params_.pawn experimental
                           // setting? 7 seems to be minimum
    ground_trace_end_ = initial_state_.ground_offset + ground_margin_;

    setStartPosition(getUUPosition(), getUUOrientation());

    initial_state_.tracing_enabled = getVehicleSetting()->enable_trace;
    initial_state_.collisions_enabled = getVehicleSetting()->enable_collisions;
    initial_state_.passthrough_enabled =
        getVehicleSetting()->enable_collision_passthrough;

    initial_state_.collision_info = CollisionInfo();

    initial_state_.was_last_move_teleport = false;
    initial_state_.was_last_move_teleport = canTeleportWhileMove();

    setupCamerasFromSettings(params_.cameras);

    // add listener for pawn's collision event
	//TODO 
    //params_.pawn_events->getCollisionSignal().connect_member(
    //    this, &RobotSimApiBase::onCollision);
    //params_.pawn_events->getPawnTickSignal().connect_member(
    //    this, &RobotSimApiBase::pawnTick);

    // start with no shapes
    this->drawable_shapes_.clear();
}

void RobotSimApiBase::setStartPosition(const FVector& position,
                                       const FRotator& rotator)
{
    initial_state_.start_location = getUUPosition();
    initial_state_.start_rotation = getUUOrientation();

    initial_state_.last_position = initial_state_.start_location;
    initial_state_.last_debug_position = initial_state_.start_location;

    // compute our home point
    // Flip z if sim mode name == "UrdfBot"
    this->flip_z_for_gps_ =
        (RobotSimSettings::singleton().simmode_name == "UrdfBot");
    Vector3r nedWrtOrigin =
        ned_transform_.toGlobalNed(initial_state_.start_location);

    if (this->flip_z_for_gps_)
    {
        nedWrtOrigin =
            Vector3r(nedWrtOrigin.x(), nedWrtOrigin.y(), -1 * nedWrtOrigin.z());
    }

    home_geo_point_ = RobotSim::EarthUtils::nedToGeodetic(
        nedWrtOrigin, RobotSimSettings::singleton().origin_geopoint,
        this->flip_z_for_gps_);
}

void RobotSimApiBase::pawnTick(float dt)
{
    update();
    updateRenderedState(dt);
    updateRendering(dt);
    serviceMoveCameraRequests();

    if (this->should_refresh_drawable_shapes_)
    {
        drawDrawShapes();
        this->should_refresh_drawable_shapes_ = false;
    }
}

void RobotSimApiBase::detectUsbRc()
{
    /*if (getRemoteControlID() >= 0) {
            joystick_.getJoyStickState(getRemoteControlID(), joystick_state_);

            rc_data_.is_initialized = joystick_state_.is_initialized;

            if (rc_data_.is_initialized)
                    URobotBlueprintLib::LogMessageString("RC Controller on USB:
    ", joystick_state_.pid_vid == "" ?
                            "(Detected)" : joystick_state_.pid_vid,
    LogDebugLevel::Informational); else URobotBlueprintLib::LogMessageString("RC
    Controller on USB not detected: ",
                            std::to_string(joystick_state_.connection_error_code),
    LogDebugLevel::Informational);
    }*/
}

void RobotSimApiBase::setupCamerasFromSettings(
    const common_utils::UniqueValueMap<std::string, APIPCamera*>& cameras)
{
    // add cameras that already exists in pawn
    cameras_.clear();
    for (const auto& p : cameras.getMap())
        cameras_.insert_or_assign(p.first, p.second);

    // create or replace cameras specified in settings
    createCamerasFromSettings();

    // setup individual cameras
    typedef RobotSim::RobotSimSettings RobotSimSettings;
    const auto& camera_defaults = RobotSimSettings::singleton().camera_defaults;
    for (auto& pair : cameras_.getMap())
    {
        const auto& camera_setting = Utils::findOrDefault(
            getVehicleSetting()->cameras, pair.first, camera_defaults);
        APIPCamera* camera = pair.second;
        camera->setupCameraFromSettings(camera_setting, getNedTransform());
    }
}

void RobotSimApiBase::createCamerasFromSettings()
{
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
    const auto& transform = getNedTransform();

    int camera_index = 0;
    for (const auto& camera_setting_pair : getVehicleSetting()->cameras)
    {
        const auto& setting = camera_setting_pair.second;

        // TODO: Properly attach camera. Determine NED transform
        FString componentName(setting.attach_link.c_str());
        APawn* pawn = params_.vehicle->GetPawn();
        pawn->GetTransform();
        USceneComponent* attachComponent =
            params_.vehicle->GetPawn()->GetRootComponent();

        FVector componentTranslation;
        FRotator componentRotation;
        FTransform componentTransform =
            params_.vehicle->GetPawn()->GetTransform();

        // TODO: Other pawns are using localNED. Is this OK?
        FVector localPositionOffset = FVector(
            setting.position.x(), setting.position.y(), setting.position.z());
        // FVector worldPositionOffset =
        // componentRotation.UnrotateVector(localPositionOffset); FVector
        // position = worldPositionOffset + componentTranslation; FTransform
        // camera_transform(FRotator(setting.rotation.pitch,
        // setting.rotation.yaw, setting.rotation.roll) + componentRotation,
        //	position, FVector(1., 1., 1.));

        FTransform world2BaseTransform(componentTransform.GetRotation(),
                                       componentTransform.GetTranslation(),
                                       FVector(1., 1., 1.));
        FVector world2CameraPos =
            world2BaseTransform.TransformPosition(localPositionOffset);
        FQuat world2CameraOri = world2BaseTransform.TransformRotation(
            FRotator(setting.rotation.pitch, setting.rotation.yaw,
                     setting.rotation.roll)
                .Quaternion());
        FTransform camera_transform(world2CameraOri, world2CameraPos,
                                    FVector(1., 1., 1.));

        // spawn and attach camera to pawn
        APIPCamera* camera =
            params_.vehicle->GetPawn()->GetWorld()->SpawnActor<APIPCamera>(
                params_.pip_camera_class, camera_transform,
                camera_spawn_params);
        camera->setIndex(camera_index);
        camera_index++;
        camera->AttachToComponent(
            attachComponent, FAttachmentTransformRules::KeepWorldTransform);

        // add on to our collection
        cameras_.insert_or_assign(camera_setting_pair.first, camera);
    }
}

void RobotSimApiBase::onCollision(class UPrimitiveComponent* MyComp,
                                  class AActor* Other,
                                  class UPrimitiveComponent* OtherComp,
                                  bool bSelfMoved,
                                  FVector HitLocation,
                                  FVector HitNormal,
                                  FVector NormalImpulse,
                                  const FHitResult& Hit)
{
    // Deflect along the surface when we collide.
    // FRotator CurrentRotation = GetActorRotation(RootComponent);
    // SetActorRotation(FQuat::Slerp(CurrentRotation.Quaternion(),
    // HitNormal.ToOrientationQuat(), 0.025f));

    UPrimitiveComponent* comp = Cast<class UPrimitiveComponent>(
        Other
            ? (Other->GetRootComponent() ? Other->GetRootComponent() : nullptr)
            : nullptr);

    state_.collision_info.has_collided = true;
    state_.collision_info.normal =
        Vector3r(Hit.ImpactNormal.X, Hit.ImpactNormal.Y, -Hit.ImpactNormal.Z);
    state_.collision_info.impact_point =
        ned_transform_.toLocalNed(Hit.ImpactPoint);
    state_.collision_info.position = ned_transform_.toLocalNed(getUUPosition());
    state_.collision_info.penetration_depth =
        ned_transform_.toNed(Hit.PenetrationDepth);
    // state_.collision_info.time_stamp =
    // RobotSim::ClockFactory::get()->nowNanos();
    state_.collision_info.object_name =
        std::string(Other ? TCHAR_TO_UTF8(*(Other->GetName())) : "(null)");
    state_.collision_info.object_id = comp ? comp->CustomDepthStencilValue : -1;

    ++state_.collision_info.collision_count;

    URobotBlueprintLib::LogMessageString(
        "Collision",
        Utils::stringf("#%d with %s - ObjID %d",
                       state_.collision_info.collision_count,
                       state_.collision_info.object_name.c_str(),
                       state_.collision_info.object_id),
        LogDebugLevel::Informational);
}

void RobotSimApiBase::possess()
{
    APlayerController* controller =
        params_.vehicle->GetPawn()->GetWorld()->GetFirstPlayerController();
    controller->UnPossess();
    controller->Possess(params_.vehicle->GetPawn());
}

const NedTransform& RobotSimApiBase::getNedTransform() const
{
    return ned_transform_;
}

APawn* RobotSimApiBase::getPawn()
{
    return params_.vehicle->GetPawn();
}

void RobotSimApiBase::setCameraPose(const RobotSim::CameraPose camera_pose)
{
    // auto matchedCamera =
    // this->cameras_.findOrDefault(camera_pose.camera_name, nullptr);

    // if (matchedCamera == nullptr)
    //{
    //    return;
    //}

    // AActor* cameraActor = matchedCamera->GetAttachParentActor();

    MoveCameraRequest r;
    r.camera_name = camera_pose.camera_name;
    r.transformVec =
        FVector(camera_pose.translation.x(), camera_pose.translation.y(),
                camera_pose.translation.z()) *
        URobotBlueprintLib::GetWorldToMetersScale(getPawn());
    FQuat qq(camera_pose.rotation.x(), camera_pose.rotation.y(),
             camera_pose.rotation.z(), camera_pose.rotation.w());
    r.rotator = qq.Rotator();

    this->move_camera_requests_.Enqueue(r);

    // busy-wait until camera is actually moved
    FGenericPlatformProcess::ConditionalSleep(
        [&] { return this->move_camera_requests_.IsEmpty(); }, 0.005);

    // FVector transformVec = FVector(camera_pose.translation.x(),
    // camera_pose.translation.y(), camera_pose.translation.z()); transformVec
    // *= URobotBlueprintLib::GetWorldToMetersScale(cameraActor); // API will be
    // specified in meters FQuat qq(camera_pose.rotation.x(),
    // camera_pose.rotation.y(), camera_pose.rotation.z(),
    // camera_pose.rotation.w()); FRotator rotation = qq.Rotator();

    // FDetachmentTransformRules detatchRules(EDetachmentRule::KeepWorld,
    // EDetachmentRule::KeepWorld, EDetachmentRule::KeepWorld, true); try
    //{
    //    auto ss = cameraActor->GetActorLocation();
    //    matchedCamera->DetachSceneComponentsFromParent(cameraActor->GetRootComponent(),
    //    true); matchedCamera->SetActorLocation(cameraActor->GetActorLocation()
    //    + transformVec);
    //    matchedCamera->SetActorRotation(cameraActor->GetActorRotation() +
    //    rotation);
    //    matchedCamera->AttachToComponent(cameraActor->GetRootComponent(),
    //    FAttachmentTransformRules::KeepWorldTransform);
    //}
    // catch (std::exception e)
    //{
    //    e.what();
    //}
}

void RobotSimApiBase::displayCollisionEffect(FVector hit_location,
                                             const FHitResult& hit)
{
    if (params_.collision_display_template != nullptr &&
        Utils::isDefinitelyLessThan(hit.ImpactNormal.Z, 0.0f))
    {
        UParticleSystemComponent* particles =
            UGameplayStatics::SpawnEmitterAtLocation(
                params_.vehicle->GetPawn()->GetWorld(),
                params_.collision_display_template, FTransform(hit_location),
                true);
        particles->SetWorldScale3D(FVector(0.1f, 0.1f, 0.1f));
    }
}

const TArray<APIPCamera*> RobotSimApiBase::getAllCameras() const
{
    TArray<APIPCamera*> result;
    result.Reserve(cameras_.valsSize());
    for (auto it = cameras_.begin(); it != cameras_.end(); ++it)
    {
        result.Emplace(*it);
    }
    return result;
}

const APIPCamera*
RobotSimApiBase::getCamera(const std::string& camera_name) const
{
    return cameras_.findOrDefault(camera_name, nullptr);
}

APIPCamera* RobotSimApiBase::getCamera(const std::string& camera_name)
{
    return const_cast<APIPCamera*>(
        static_cast<const RobotSimApiBase*>(this)->getCamera(camera_name));
}

int RobotSimApiBase::getCameraCount()
{
    return cameras_.valsSize();
}

void RobotSimApiBase::reset()
{
    UpdatableObject::reset();

    state_ = initial_state_;
    // rc_data_ = RobotSim::RCData();
    params_.vehicle->GetPawn()->SetActorLocationAndRotation(
        state_.start_location, state_.start_rotation, false, nullptr,
        ETeleportType::TeleportPhysics);

    environment_->reset();
}

void RobotSimApiBase::update()
{
    // update position from kinematics so we have latest position after physics
    // update
    environment_->setPosition(kinematics_.pose.position);
    environment_->update();
    // kinematics_->update();

    UpdatableObject::update();
}

void RobotSimApiBase::serviceMoveCameraRequests()
{
    while (!this->move_camera_requests_.IsEmpty())
    {
        MoveCameraRequest request;
        this->move_camera_requests_.Dequeue(request);

        auto matchedCamera =
            this->cameras_.findOrDefault(request.camera_name, nullptr);

        if (matchedCamera == nullptr)
        {
            continue;
        }

        AActor* cameraActor = matchedCamera->GetAttachParentActor();

        FDetachmentTransformRules detatchRules(
            EDetachmentRule::KeepWorld, EDetachmentRule::KeepWorld,
            EDetachmentRule::KeepWorld, true);
        matchedCamera->DetachAllSceneComponents(cameraActor->GetRootComponent(),
                                                detatchRules);
        matchedCamera->DetachFromActor(detatchRules);
        // matchedCamera->DetachSceneComponentsFromParent(cameraActor->GetRootComponent(),
        // true);
        auto a1 = cameraActor->GetActorLocation();
        auto a2 = cameraActor->GetRootComponent()->GetComponentLocation();
        matchedCamera->SetActorLocation(
            cameraActor->GetRootComponent()->GetComponentLocation() +
            request.transformVec);
        matchedCamera->SetActorRotation(
            cameraActor->GetRootComponent()->GetComponentRotation() +
            request.rotator);
        matchedCamera->AttachToComponent(
            cameraActor->GetRootComponent(),
            FAttachmentTransformRules::KeepWorldTransform);
    }
}

// void playBack()
//{
// if (params_.pawn->GetRootPrimitiveComponent()->IsAnySimulatingPhysics()) {
//    params_.pawn->GetRootPrimitiveComponent()->SetSimulatePhysics(false);
//    params_.pawn->GetRootPrimitiveComponent()->SetSimulatePhysics(true);
//}
// TODO: refactor below code used for playback
// std::ifstream sim_log("C:\\temp\\mavlogs\\circle\\sim_cmd_006_orbit
// 5 1.txt.pos.txt"); plot(sim_log, FColor::Purple, Vector3r(0, 0, -3));
// std::ifstream real_log("C:\\temp\\mavlogs\\circle\\real_cmd_006_orbit
// 5 1.txt.pos.txt"); plot(real_log, FColor::Yellow, Vector3r(0, 0, -3));

// std::ifstream sim_log("C:\\temp\\mavlogs\\square\\sim_cmd_005_square
// 5 1.txt.pos.txt"); plot(sim_log, FColor::Purple, Vector3r(0, 0, -3));
// std::ifstream real_log("C:\\temp\\mavlogs\\square\\real_cmd_012_square
// 5 1.txt.pos.txt"); plot(real_log, FColor::Yellow, Vector3r(0, 0, -3));
//}

RobotSimApiBase::CollisionInfo RobotSimApiBase::getCollisionInfo() const
{
    return state_.collision_info;
}

FVector RobotSimApiBase::getUUPosition() const
{
    return params_.vehicle->GetPawn()
        ->GetActorLocation(); // - state_.mesh_origin
}

FRotator RobotSimApiBase::getUUOrientation() const
{
    return params_.vehicle->GetPawn()->GetActorRotation();
}

void RobotSimApiBase::toggleTrace()
{
    state_.tracing_enabled = !state_.tracing_enabled;

    if (!state_.tracing_enabled)
        UKismetSystemLibrary::FlushPersistentDebugLines(
            params_.vehicle->GetPawn()->GetWorld());
    else
    {
        state_.debug_position_offset =
            state_.current_debug_position - state_.current_position;
        state_.last_debug_position = state_.last_position;
    }
}

void RobotSimApiBase::allowPassthroughToggleInput()
{
    state_.passthrough_enabled = !state_.passthrough_enabled;
    URobotBlueprintLib::LogMessage("enable_passthrough_on_collisions: ",
                                   FString::FromInt(state_.passthrough_enabled),
                                   LogDebugLevel::Informational);
}

void RobotSimApiBase::plot(std::istream& s,
                           FColor color,
                           const Vector3r& offset)
{
    using namespace RobotSim;

    Vector3r last_point = VectorMath::nanVector();
    uint64_t timestamp;
    float heading, x, y, z;
    while (s >> timestamp >> heading >> x >> y >> z)
    {
        std::string discarded_line;
        std::getline(s, discarded_line);

        Vector3r current_point(x, y, z);
        current_point += offset;
        if (!VectorMath::hasNan(last_point))
        {
            UKismetSystemLibrary::DrawDebugLine(
                params_.vehicle->GetPawn()->GetWorld(),
                ned_transform_.fromLocalNed(last_point),
                ned_transform_.fromLocalNed(current_point), color, 0, 3.0F);
        }
        last_point = current_point;
    }
}

RobotSim::CameraInfo
RobotSimApiBase::getCameraInfo(const std::string& camera_name) const
{
    RobotSim::CameraInfo camera_info;

    const APIPCamera* camera = getCamera(camera_name);
    camera_info.pose.position =
        ned_transform_.toLocalNed(camera->GetActorLocation());
    camera_info.pose.orientation =
        ned_transform_.toNed(camera->GetActorRotation().Quaternion());
    camera_info.fov = camera->GetCameraComponent()->FieldOfView;
    camera_info.proj_mat =
        camera->getProjectionMatrix(APIPCamera::ImageType::Scene);
    return camera_info;
}

void RobotSimApiBase::setCameraOrientation(
    const std::string& camera_name, const RobotSim::Quaternionr& orientation)
{
    URobotBlueprintLib::RunCommandOnGameThread(
        [this, camera_name, orientation]() {
            APIPCamera* camera = getCamera(camera_name);
            FQuat quat = ned_transform_.fromNed(orientation);
            camera->setCameraOrientation(quat.Rotator());
        },
        true);
}

RobotSim::RayCastResponse
RobotSimApiBase::rayCast(const RobotSim::RayCastRequest& request)
{
    RobotSim::RayCastResponse response;
    response.hits.clear();

    UWorld* world = this->getPawn()->GetWorld();

    float worldToMeters =
        URobotBlueprintLib::GetWorldToMetersScale(this->getPawn());

    FVector start(request.position.x(), request.position.y(),
                  request.position.z());

    FVector end(request.position.x() + request.direction.x(),
                request.position.y() + request.direction.y(),
                request.position.z() + request.direction.z());

    start *= worldToMeters;
    end *= worldToMeters;

    FVector referenceOffset = FVector::ZeroVector;
    FRotator referenceRotator = FRotator::ZeroRotator;

    if (request.reference_frame_link.length() > 0)
    {
        params_.vehicle->GetComponentReferenceTransform(
            FString(request.reference_frame_link.c_str()), referenceOffset,
            referenceRotator);

        start = referenceRotator.RotateVector(start) + referenceOffset;
        end = referenceRotator.RotateVector(end) + referenceOffset;
    }

    FHitResult hitResult(ForceInit);

    bool isHit = true;
    TArray<const AActor*> ignoreActors;
    while (isHit)
    {
        isHit = URobotBlueprintLib::GetObstacle(this->getPawn(), start, end,
                                                hitResult, ignoreActors,
                                                ECC_Visibility, true);

        if (isHit)
        {
            RobotSim::RayCastHit hit;
            if (hitResult.Actor != nullptr)
            {
                hit.collided_actor_name = std::string(
                    TCHAR_TO_UTF8(*hitResult.Actor.Get()->GetName()));
            }

            // Translate hit back into local frame of link provided
            // If no link is provided, this is a noop
            // Also scale back to MKS
            FVector hitPoint = referenceRotator.UnrotateVector(
                hitResult.ImpactPoint - referenceOffset);
            hitPoint /= worldToMeters;

            hit.hit_point =
                RobotSim::Vector3r(hitPoint.X, hitPoint.Y, hitPoint.Z);

            FVector impactNormal = hitResult.ImpactNormal;
            if (!impactNormal.Normalize())
            {
                throw std::runtime_error(
                    "Unable to normalize an impact vector.");
            }

            // Only need to rotate normal
            FVector hitNormal = referenceRotator.UnrotateVector(impactNormal);

            hit.hit_normal =
                RobotSim::Vector3r(hitNormal.X, hitNormal.Y, hitNormal.Z);

            response.hits.emplace_back(hit);

            // quick debugging draw
            /*          if (request.persist_seconds > 0)
                                  {
                                          DrawDebugLine(world, start,
               hitResult.ImpactPoint, FColor(0, 0, 255, 0), false,
               request.persist_seconds, (uint8)'\002', 5); DrawDebugPoint(world,
               hitResult.ImpactPoint, 10, FColor(255, 0, 0, 0), false,
               request.persist_seconds, (uint8)'\002');
                                  }*/

            if (!request.through_blocking)
            {
                return response;
            }
            else
            {
                start = hitResult.ImpactPoint;
                ignoreActors.Add(hitResult.Actor.Get());
            }
        }
        else
        {
            // quick debugging draw
            // if (request.persist_seconds > 0)
            //{
            //    DrawDebugLine(world, start, end, FColor(0, 0, 255, 0), false,
            //    request.persist_seconds, (uint8)'\002', 5);
            //}
        }
    }

    return response;
}

// parameters in NED frame
RobotSimApiBase::Pose RobotSimApiBase::getPose() const
{
    // kinematics_.twist.linear =
    // RobotSim::Vector3r(nedTransform.fromNed(velocity.X),
    // nedTransform.fromNed(velocity.Y), nedTransform.fromNed(velocity.Z));
    auto nedTransform = getNedTransform();
    FVector position = getUUPosition();
    FVector position_scaled(nedTransform.toNed(position.X),
                            nedTransform.toNed(position.Y),
                            nedTransform.toNed(position.Z));
    return toPose(position_scaled, getUUOrientation().Quaternion());
}

RobotSimApiBase::Pose RobotSimApiBase::toPose(const FVector& u_position,
                                              const FQuat& u_quat) const
{
    // const Vector3r& position = ned_transform_.toLocalNed(u_position);
    // const Quaternionr& orientation = ned_transform_.toNed(u_quat);

    Vector3r position(u_position.X, u_position.Y, u_position.Z);
    Quaternionr orientation(u_quat.W, u_quat.X, u_quat.Y, u_quat.Z);

    return Pose(position, orientation);
}

void RobotSimApiBase::setPose(const Pose& pose, bool ignore_collision)
{
    URobotBlueprintLib::RunCommandOnGameThread(
        [this, pose, ignore_collision]() {
            setPoseInternal(pose, ignore_collision);
        },
        true);
}

std::vector<RobotSim::GeoPoint> RobotSimApiBase::xyzToGeoPoints(
    const std::vector<RobotSim::Vector3r>& xyz_points)
{
    std::vector<RobotSim::GeoPoint> result;
    result.reserve(xyz_points.size());

    for (const auto& point : xyz_points)
    {
        result.emplace_back(RobotSim::EarthUtils::nedToGeodetic(
            point, this->home_geo_point_, this->flip_z_for_gps_));
    }

    return result;
}

void RobotSimApiBase::setPoseInternal(const Pose& pose, bool ignore_collision)
{
    // translate to new PawnSimApi position & orientation from NED to NEU
    // FVector position = ned_transform_.fromLocalNed(pose.position);
    FVector position(pose.position.x(), pose.position.y(), pose.position.z());
    state_.current_position = position;

    // quaternion formula comes from http://stackoverflow.com/a/40334755/207661
    // FQuat orientation = ned_transform_.fromNed(pose.orientation);
    FQuat orientation(pose.orientation.x(), pose.orientation.y(),
                      pose.orientation.z(), pose.orientation.w());

    bool enable_teleport = ignore_collision || canTeleportWhileMove();

    // must reset collision before we set pose. Setting pose will immediately
    // call NotifyHit if there was collision if there was no collision than
    // has_collided would remain false, else it will be set so its value can be
    // checked at the start of next tick
    state_.collision_info.has_collided = false;
    state_.was_last_move_teleport = enable_teleport;

    params_.vehicle->TeleportToLocation(position, orientation, enable_teleport);

    if (state_.tracing_enabled &&
        (state_.last_position - position).SizeSquared() > 0.25)
    {
        UKismetSystemLibrary::DrawDebugLine(
            params_.vehicle->GetPawn()->GetWorld(), state_.last_position,
            position, FColor::Purple, -1, 3.0f);
        state_.last_position = position;
    }
    else if (!state_.tracing_enabled)
    {
        state_.last_position = position;
    }
}

void RobotSimApiBase::setDebugPose(const Pose& debug_pose)
{
    state_.current_debug_position =
        ned_transform_.fromLocalNed(debug_pose.position);
    if (state_.tracing_enabled && !VectorMath::hasNan(debug_pose.position))
    {
        FVector debug_position =
            state_.current_debug_position - state_.debug_position_offset;
        if ((state_.last_debug_position - debug_position).SizeSquared() > 0.25)
        {
            UKismetSystemLibrary::DrawDebugLine(
                params_.vehicle->GetPawn()->GetWorld(),
                state_.last_debug_position, debug_position,
                FColor(0xaa, 0x33, 0x11), -1, 10.0F);
            URobotBlueprintLib::LogMessage(FString("Debug Pose: "),
                                           debug_position.ToCompactString(),
                                           LogDebugLevel::Informational);
            state_.last_debug_position = debug_position;
        }
    }
    else if (!state_.tracing_enabled)
    {
        state_.last_debug_position =
            state_.current_debug_position - state_.debug_position_offset;
    }
}

bool RobotSimApiBase::canTeleportWhileMove() const
{
    // allow teleportation
    //  if collisions are not enabled
    //  or we have collided but passthrough is enabled
    //     we will flip-flop was_last_move_teleport flag so on one tick we have
    //     passthrough and other tick we don't without flip flopping, collisions
    //     can't be detected
    return !state_.collisions_enabled ||
           (state_.collision_info.has_collided &&
            !state_.was_last_move_teleport && state_.passthrough_enabled);
}

const RobotSim::Kinematics::State* RobotSimApiBase::getPawnKinematics() const
{
    return &kinematics_;
}

void RobotSimApiBase::updateKinematics(float dt)
{
    const auto last_kinematics = kinematics_;

    kinematics_.pose = getPose();

    // kinematics_.twist.linear =
    // getNedTransform().toLocalNed(getPawn()->GetVelocity());
    auto nedTransform = getNedTransform();
    auto velocity = getPawn()->GetVelocity();
    kinematics_.twist.linear = RobotSim::Vector3r(
        nedTransform.toNed(velocity.X), nedTransform.toNed(velocity.Y),
        nedTransform.toNed(velocity.Z));
    kinematics_.twist.angular = RobotSim::VectorMath::toAngularVelocity(
        kinematics_.pose.orientation, last_kinematics.pose.orientation, dt);

    kinematics_.accelerations.linear =
        (kinematics_.twist.linear - last_kinematics.twist.linear) / dt;
    kinematics_.accelerations.angular =
        (kinematics_.twist.angular - last_kinematics.twist.angular) / dt;

    // TODO: update other fields?
}

// void RobotSimApi::updateRenderedState(float dt)
//{
//	updateKinematics(dt);
//}
//
// void RobotSimApi::updateRendering(float dt)
//{
//	unused(dt);
//	//no default action in this base class
//}

const RobotSim::Kinematics::State*
RobotSimApiBase::getGroundTruthKinematics() const
{
    return &kinematics_;
}
const RobotSim::Environment* RobotSimApiBase::getGroundTruthEnvironment() const
{
    return environment_.get();
}

// std::string RobotSimApi::getRecordFileLine(bool is_header_line) const
//{
//	if (is_header_line) {
//		return "TimeStamp\tPOS_X\tPOS_Y\tPOS_Z\tQ_W\tQ_X\tQ_Y\tQ_Z\t";
//	}
//
//	const RobotSim::Kinematics::State* kinematics =
// getGroundTruthKinematics(); 	uint64_t timestamp_millis =
// static_cast<uint64_t>(RobotSim::ClockFactory::get()->nowNanos() / 1.0E6);
//
//	//TODO: because this bug we are using alternative code with stringstream
//	//https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html
//
//	std::string line;
//	line.append(std::to_string(timestamp_millis)).append("\t")
//		.append(std::to_string(kinematics->pose.position.x())).append("\t")
//		.append(std::to_string(kinematics->pose.position.y())).append("\t")
//		.append(std::to_string(kinematics->pose.position.z())).append("\t")
//		.append(std::to_string(kinematics->pose.orientation.w())).append("\t")
//		.append(std::to_string(kinematics->pose.orientation.x())).append("\t")
//		.append(std::to_string(kinematics->pose.orientation.y())).append("\t")
//		.append(std::to_string(kinematics->pose.orientation.z())).append("\t")
//		;
//
//	return line;
//
//	//std::stringstream ss;
//	//ss << timestamp_millis << "\t";
//	//ss << kinematics.pose.position.x() << "\t" <<
// kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
//	//ss << kinematics.pose.orientation.w() << "\t" <<
// kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() <<
//"\t" << kinematics.pose.orientation.z() << "\t";
//	//ss << "\n";
//	//return ss.str();
//}

void RobotSimApiBase::setDrawShapes(
    std::unordered_map<std::string, RobotSim::DrawableShape>& drawableShapes,
    bool persist_unmentioned)
{
    if (persist_unmentioned)
    {
        for (auto& kvp : drawableShapes)
        {
            this->drawable_shapes_[kvp.first] = kvp.second;
        }
    }
    else
    {
        this->drawable_shapes_ = drawableShapes;
    }

    this->should_refresh_drawable_shapes_ = true;
}

void RobotSimApiBase::drawDrawShapes()
{
    //    FlushPersistentDebugLines(getPawn()->GetWorld());

    bool persistant = true;
    float persistSeconds = -1;
    uint8 priority = (uint8)'\002';

    auto pawn = this->params_.vehicle;
    auto world = getPawn()->GetWorld();
    float worldScale = URobotBlueprintLib::GetWorldToMetersScale(getPawn());

    for (const auto& kvp : this->drawable_shapes_)
    {
        auto shape = kvp.second;
        FVector offsetVector = FVector::ZeroVector;
        FRotator offsetRotator = FRotator::ZeroRotator;

        if (shape.reference_frame_link.length() > 0)
        {
            pawn->GetComponentReferenceTransform(
                FString(shape.reference_frame_link.c_str()), offsetVector,
                offsetRotator);
        }

        // Point
        // First three numbers are the center.
        // next number is the size.
        // Next four numbers are the color in RGBA space.
        if (shape.type == 0)
        {
            FVector location(shape.shape_params[0], shape.shape_params[1],
                             shape.shape_params[2]);
            float size = shape.shape_params[3];
            FColor color(static_cast<uint8>(shape.shape_params[4]),
                         static_cast<uint8>(shape.shape_params[5]),
                         static_cast<uint8>(shape.shape_params[6]),
                         static_cast<uint8>(shape.shape_params[7]));

            FVector finalLocation =
                offsetRotator.RotateVector(location * worldScale) +
                offsetVector;

            //            DrawDebugPoint(world, finalLocation, size, color,
            //            persistant, persistSeconds, priority);
        }
        // Sphere
        // First three numbers are the center.
        // Next number is the radius
        // Next number is the thickness
        // Next number is the number of segments
        // Next four numbers are the color in RGBA space
        else if (shape.type == 1)
        {
            FVector location(shape.shape_params[0], shape.shape_params[1],
                             shape.shape_params[2]);
            float radius = shape.shape_params[3] * worldScale;
            float thickness = shape.shape_params[4];
            int numberOfSegments = static_cast<int>(shape.shape_params[5]);
            FColor color(static_cast<uint8>(shape.shape_params[6]),
                         static_cast<uint8>(shape.shape_params[7]),
                         static_cast<uint8>(shape.shape_params[8]),
                         static_cast<uint8>(shape.shape_params[9]));

            FVector finalLocation =
                offsetRotator.RotateVector(location * worldScale) +
                offsetVector;

            //            DrawDebugSphere(world, finalLocation, radius,
            //            numberOfSegments, color, persistant, persistSeconds,
            //            priority, thickness);
        }
        // Circle
        // First three numbers are the center
        // Next three numbers are the normal
        // Next number is the radius
        // Next number is the thickness
        // Next number is the number of segments
        // Next four numbers are the color in RGBA space
        else if (shape.type == 2)
        {
            FVector location(shape.shape_params[0], shape.shape_params[1],
                             shape.shape_params[2]);
            FVector normal(shape.shape_params[3], shape.shape_params[4],
                           shape.shape_params[5]);
            float radius = shape.shape_params[6] * worldScale;
            float thickness = shape.shape_params[7];
            int numberOfSegments = static_cast<int>(shape.shape_params[8]);
            FColor color(static_cast<uint8>(shape.shape_params[9]),
                         static_cast<uint8>(shape.shape_params[10]),
                         static_cast<uint8>(shape.shape_params[11]),
                         static_cast<uint8>(shape.shape_params[12]));

            FVector finalLocation =
                offsetRotator.RotateVector(location * worldScale) +
                offsetVector;

            FVector localY(0, 1, 0);
            FVector localZ(0, 0, 1);

            FRotator normalRotator = normal.Rotation();
            FRotator globalRotator = normalRotator + offsetRotator;

            FVector globalY = globalRotator.RotateVector(localY);
            FVector globalZ = globalRotator.RotateVector(localZ);

            //            DrawDebugCircle(world, finalLocation, radius,
            //            numberOfSegments, color, persistant, persistSeconds,
            //            priority, thickness, globalY, globalZ, false);
        }
        // Box
        // First three numbers are the center
        // Next three numbers are the extents
        // Next number is the thickness
        // Next four are the color in RGBA space
        else if (shape.type == 3)
        {
            FVector location(shape.shape_params[0], shape.shape_params[1],
                             shape.shape_params[2]);
            FVector boxExtents(shape.shape_params[3], shape.shape_params[4],
                               shape.shape_params[5]);
            float thickness = shape.shape_params[6];
            FColor color(static_cast<uint8>(shape.shape_params[7]),
                         static_cast<uint8>(shape.shape_params[8]),
                         static_cast<uint8>(shape.shape_params[9]),
                         static_cast<uint8>(shape.shape_params[10]));

            FVector finalLocation =
                offsetRotator.RotateVector(location * worldScale) +
                offsetVector;

            boxExtents *= worldScale;

            //            DrawDebugBox(world, finalLocation, boxExtents, color,
            //            persistant, persistSeconds, priority, thickness);
        }
        // Line
        // First three numbers are the first point
        // Second three numbers are the second point
        // Next number is the thickness
        // Next four are the color in RGBA space
        else if (shape.type == 4)
        {
            FVector firstPoint(shape.shape_params[0], shape.shape_params[1],
                               shape.shape_params[2]);
            FVector secondPoint(shape.shape_params[3], shape.shape_params[4],
                                shape.shape_params[5]);
            float thickness = shape.shape_params[6];
            FColor color(static_cast<uint8>(shape.shape_params[7]),
                         static_cast<uint8>(shape.shape_params[8]),
                         static_cast<uint8>(shape.shape_params[9]),
                         static_cast<uint8>(shape.shape_params[10]));

            FVector finalFirstPoint =
                offsetRotator.RotateVector(firstPoint * worldScale) +
                offsetVector;
            FVector finalSecondPoint =
                offsetRotator.RotateVector(secondPoint * worldScale) +
                offsetVector;

            //            DrawDebugLine(world, finalFirstPoint,
            //            finalSecondPoint, color, persistant, persistSeconds,
            //            priority, thickness);
        }
    }
}

RobotSim::RobotApiBase* RobotSimApiBase::getVehicleApiBase() const
{
    return nullptr;
}