#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Particles/ParticleSystemComponent.h"

#include <vector>
#include <memory>
#include "common_utils/Common.hpp"
#include "common_utils/Signal.hpp"
#include "common_utils/CommonStructs.hpp"
#include "PIPCamera.h"
#include "physics/Kinematics.hpp"
#include "NedTransform.h"
#include "common_utils/RobotSimSettings.hpp"
//#include "SimJoyStick.h"
#include "common_utils/RobotApiBase.hpp"
#include "common_utils/UniqueValueMap.hpp"
#include "RobotBase.h"
#include "PawnEvents.h"
#include "common_utils/UpdatableObject.hpp"
#include "common_utils/ImageCaptureBase.hpp"
#include "physics/Environment.hpp"
//#include "UnrealSensors/UnrealSensorFactory.h"
//
class RobotSimApiBase : public RobotSim::UpdatableObject
{
public: // types
    typedef RobotSim::RobotSimSettings RobotSimSettings;
    typedef RobotSim::GeoPoint GeoPoint;
    typedef RobotSim::Vector3r Vector3r;
    typedef RobotSim::Pose Pose;
    typedef RobotSim::Quaternionr Quaternionr;
    typedef RobotSim::CollisionInfo CollisionInfo;
    typedef RobotSim::VectorMath VectorMath;
    typedef RobotSim::real_T real_T;
    typedef RobotSim::Utils Utils;
    typedef RobotSim::RobotSimSettings::VehicleSetting VehicleSetting;
    typedef RobotSim::ImageCaptureBase ImageCaptureBase;

    struct Params
    {
        RobotBase* vehicle;
        const NedTransform* global_transform;
        PawnEvents* pawn_events;
        common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
        UClass* pip_camera_class;
        UParticleSystem* collision_display_template;
        RobotSim::GeoPoint home_geopoint;
        std::string vehicle_name;

        Params()
        {
        }

        Params(RobotBase* pawn_val,
               const NedTransform* global_transform_val,
               PawnEvents* pawn_events_val,
               const common_utils::UniqueValueMap<std::string, APIPCamera*>
                   cameras_val,
               UClass* pip_camera_class_val,
               UParticleSystem* collision_display_template_val,
               const RobotSim::GeoPoint home_geopoint_val,
               std::string vehicle_name_val)
        {
            vehicle = pawn_val;
            global_transform = global_transform_val;
            pawn_events = pawn_events_val;
            cameras = cameras_val;
            pip_camera_class = pip_camera_class_val;
            collision_display_template = collision_display_template_val;
            home_geopoint = home_geopoint_val;
            vehicle_name = vehicle_name_val;
        }
    };

public: // implementation of VehicleSimApiBase
    void setCameraPose(const RobotSim::CameraPose camera_pose);
    Pose getPose() const;
    void setPose(const Pose& pose, bool ignore_collision);
    std::vector<GeoPoint> xyzToGeoPoints(const std::vector<Vector3r>& xyz);
    RobotSim::CameraInfo getCameraInfo(const std::string& camera_name) const;
    void setCameraOrientation(const std::string& camera_name,
                              const Quaternionr& orientation);
    RobotSim::RayCastResponse rayCast(const RobotSim::RayCastRequest& request);
    CollisionInfo getCollisionInfo() const;

    std::string getVehicleName() const
    {
        return params_.vehicle_name;
    }
    void toggleTrace();

    const RobotSim::Kinematics::State* getGroundTruthKinematics() const;
    const RobotSim::Environment* getGroundTruthEnvironment() const;

    void setDrawShapes(std::unordered_map<std::string, RobotSim::DrawableShape>&
                           drawableShapes,
                       bool persistUnmentioned);
    const RobotSimSettings::VehicleSetting* getVehicleSetting() const
    {
        return RobotSimSettings::singleton().getVehicleSetting(
            getVehicleName());
    }

protected: // additional interface for derived class
    virtual void pawnTick(float dt);
    const RobotSim::Kinematics::State* getPawnKinematics() const;
    void setPoseInternal(const Pose& pose, bool ignore_collision);
    virtual RobotSim::RobotApiBase* getVehicleApiBase() const;

public: // Unreal specific methods
    // returns one of the cameras attached to the pawn
    const TArray<APIPCamera*> getAllCameras() const;
    const APIPCamera* getCamera(const std::string& camera_name) const;
    APIPCamera* getCamera(const std::string& camera_name);
    int getCameraCount();

    // if enabled, this would show some flares
    void displayCollisionEffect(FVector hit_location, const FHitResult& hit);

    // return the attached pawn
    APawn* getPawn();

    // get/set pose
    // parameters in NED frame
    void setDebugPose(const Pose& debug_pose);

    FVector getUUPosition() const;
    FRotator getUUOrientation() const;

    const NedTransform& getNedTransform() const;

    void possess();
    // void setRCForceFeedback(float rumble_strength, float auto_center);

private: // methods
    bool canTeleportWhileMove() const;
    void allowPassthroughToggleInput();
    void detectUsbRc();
    void setupCamerasFromSettings(
        const common_utils::UniqueValueMap<std::string, APIPCamera*>& cameras);
    void createCamerasFromSettings();
    // on collision, pawns should update this
    void onCollision(class UPrimitiveComponent* MyComp,
                     class AActor* Other,
                     class UPrimitiveComponent* OtherComp,
                     bool bSelfMoved,
                     FVector HitLocation,
                     FVector HitNormal,
                     FVector NormalImpulse,
                     const FHitResult& Hit);

    // these methods are for future usage
    void plot(std::istream& s, FColor color, const Vector3r& offset);
    RobotSimApiBase::Pose toPose(const FVector& u_position,
                                 const FQuat& u_quat) const;
    void updateKinematics(float dt);
    void setStartPosition(const FVector& position, const FRotator& rotator);
    void drawDrawShapes();
    void serviceMoveCameraRequests();

private: // vars
    Params params_;
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras_;
    RobotSim::GeoPoint home_geo_point_;

    std::string vehicle_name_;
    NedTransform ned_transform_;

    FVector ground_trace_end_;
    FVector ground_margin_;
    std::string log_line_;

    bool flip_z_for_gps_;

    struct State
    {
        FVector start_location;
        FRotator start_rotation;
        FVector last_position;
        FVector last_debug_position;
        FVector current_position;
        FVector current_debug_position;
        FVector debug_position_offset;
        bool tracing_enabled;
        bool collisions_enabled;
        bool passthrough_enabled;
        bool was_last_move_teleport;
        CollisionInfo collision_info;

        FVector mesh_origin;
        FVector mesh_bounds;
        FVector ground_offset;
        FVector transformation_offset;
    };

    struct MoveCameraRequest
    {
        std::string camera_name;
        FVector transformVec;
        FRotator rotator;
    };

    TQueue<MoveCameraRequest> move_camera_requests_;

    State state_, initial_state_;

    RobotSim::Kinematics::State kinematics_;
    std::unique_ptr<RobotSim::Environment> environment_;

    std::unordered_map<std::string, RobotSim::DrawableShape> drawable_shapes_;

    bool should_refresh_drawable_shapes_ = false;

public:
    RobotSimApiBase(Params params);

    virtual void reset() override;
    virtual void update() override;

    virtual std::string getRecordFileLine(bool is_header_line) const;

    virtual void updateRenderedState(float dt);
    virtual void updateRendering(float dt);
};
