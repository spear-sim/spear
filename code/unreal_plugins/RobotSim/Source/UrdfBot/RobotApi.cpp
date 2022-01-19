
#include "RobotApi.h"
#include <string>

#include "UrdfBot/UrdfBotPawn.h"

// UrdfBotApi::UrdfBotApi(AUrdfBotPawn* pawn, const RobotSim::Kinematics::State*
// pawn_kinematics, const RobotSim::GeoPoint& home_geopoint,
//    std::function<const RobotSim::Kinematics::State*(std::string)>
//    state_provider_fxn, const RobotSim::RobotSimSettings::VehicleSetting*
//    vehicle_setting, std::shared_ptr<RobotSim::SensorFactory> sensor_factory,
//    const RobotSim::Environment& environment) :
//    UrdfBotApiBase(vehicle_setting, sensor_factory, state_provider_fxn,
//    environment),
//        pawn_(pawn), pawn_kinematics_(pawn_kinematics),
//        home_geopoint_(home_geopoint)
//{
//}
RobotApi::RobotApi(
    AUrdfBotPawn* pawn,
    const RobotSim::Kinematics::State* pawn_kinematics,
    const RobotSim::GeoPoint& home_geopoint,
    std::function<const RobotSim::Kinematics::State*(std::string)>
        state_provider_fxn,
    const RobotSim::RobotSimSettings::VehicleSetting* vehicle_setting,
    const RobotSim::Environment& environment)
    : pawn_(pawn), pawn_kinematics_(pawn_kinematics),
      home_geopoint_(home_geopoint)
{
}
RobotApi::~RobotApi()
{
}

void RobotApi::addLinearForce(const RobotApi::AddLinearForce& addedForce,
                              const std::string& vehicle_name)
{
    // TODO: For now, only use one vehicle_name.
    TMap<FString, AUrdfLink*> components = this->pawn_->GetLinkComponents();
    FString linkName = FString(addedForce.link_name.c_str());
    if (components.Contains(linkName))
    {
        auto forceSpecification = new UrdfLinearForceSpecification(addedForce);

        if (!forceSpecification)
            throw std::runtime_error(
                "OOM when attempting to allocate linear force specification.");

        components[linkName]->AddForceSpecification(forceSpecification);
        this->force_link_mapping_.Add(FString(addedForce.force_name.c_str()),
                                      components[linkName]);
    }
}

void RobotApi::addAngularForce(const RobotApi::AddAngularForce& addedForce,
                               const std::string& vehicle_name)
{
    // TODO: For now, only use one vehicle_name.
    TMap<FString, AUrdfLink*> components = this->pawn_->GetLinkComponents();
    FString linkName = FString(addedForce.link_name.c_str());
    if (components.Contains(linkName))
    {
        auto forceSpecification = new UrdfAngularForceSpecification(addedForce);

        if (!forceSpecification)
            throw std::runtime_error(
                "OOM when attempting to allocate angular force specification.");

        components[linkName]->AddForceSpecification(forceSpecification);
        this->force_link_mapping_.Add(FString(addedForce.force_name.c_str()),
                                      components[linkName]);
    }
}

void RobotApi::updateForceMagnitude(const UpdateForceMagnitude& updatedForce,
                                    const std::string& vehicle_name)
{
    FString forceName = FString(updatedForce.force_name.c_str());

    if (this->force_link_mapping_.Contains(forceName))
    {
        AUrdfLink* link = this->force_link_mapping_[forceName];
        link->SetForceMagnitude(forceName, updatedForce.magnitude);
    }
}

void RobotApi::updateControlledMotionComponentControlSignal(
    const UrdfBotControlledMotionComponentControlSignal& updated_control_signal,
    const std::string& vehicle_name)
{
    TMap<FString, ControlledMotionComponent*> components =
        this->pawn_->GetControlledMotionComponents();
    FString componentName =
        FString(updated_control_signal.component_name.c_str());
    if (components.Contains(componentName))
    {
        ControlledMotionComponent* component = components[componentName];
        TMap<FString, float> controlSignalValues;
        for (auto const& kvp : updated_control_signal.control_signal_values)
        {
            controlSignalValues.Add(FString(kvp.first.c_str()), kvp.second);
        }
        component->SetControl(controlSignalValues);
    }
}

void RobotApi::setAbsoluteJointPose(
    const UrdfBotControlledMotionComponentControlSignal& updated_control_signal,
    const std::string& vehicle_name)
{
    TMap<FString, ControlledMotionComponent*> components =
        this->pawn_->GetControlledMotionComponents();
    FString componentName =
        FString(updated_control_signal.component_name.c_str());
    if (components.Contains(componentName))
    {
        ControlledMotionComponent* component = components[componentName];
        TMap<FString, float> controlSignalValues;
        for (auto const& kvp : updated_control_signal.control_signal_values)
        {
            controlSignalValues.Add(FString(kvp.first.c_str()), kvp.second);
        }
        component->ControlAbsoluteJointPose(controlSignalValues);
    }
}

void RobotApi::setRelativeJointPose(
    const UrdfBotControlledMotionComponentControlSignal& updated_control_signal,
    const std::string& vehicle_name)
{
    TMap<FString, ControlledMotionComponent*> components =
        this->pawn_->GetControlledMotionComponents();
    FString componentName =
        FString(updated_control_signal.component_name.c_str());
    if (components.Contains(componentName))
    {
        ControlledMotionComponent* component = components[componentName];
        TMap<FString, float> controlSignalValues;
        for (auto const& kvp : updated_control_signal.control_signal_values)
        {
            controlSignalValues.Add(FString(kvp.first.c_str()), kvp.second);
        }
        component->ControlRelativeJointPose(controlSignalValues);
    }
}

void RobotApi::setBotInstantSpeed(
    const UrdfBotControlledMotionComponentControlSignal& updated_control_signal)
{
    TMap<FString, ControlledMotionComponent*> components =
        this->pawn_->GetBotMotionComponents();
    FString componentName =
        FString(updated_control_signal.component_name.c_str());
    if (components.Contains(componentName))
    {
        ControlledMotionComponent* component = components[componentName];
        float actual_speed = updated_control_signal.control_signal_values.at(
            std::string("Value"));
        component->SetMotionSpeed(actual_speed);
    }
}

RobotSim::Momentums RobotApi::getBotInstantSpeed() const
{
    /*
            此操作用于计算获取各个motor的运动数据， 适用于ros控制。
    */
    // std::map<std::string, float> motion_component_speed;
    // TMap<FString, ControlledMotionComponent*>  motionComponents =
    // this->pawn_->GetBotMotionComponents(); for (auto const& component :
    // motionComponents)
    //{
    //	motion_component_speed[std::string(TCHAR_TO_UTF8(*component.Key))] =
    // component.Value->GetMotorSpeed();
    //}
    // return motion_component_speed;

    RobotSim::Momentums SpeedMomentums;
    AUrdfLink* rootLink = this->pawn_->GetRootLinkComponent();
    if (rootLink)
    {
        FVector linearV = rootLink->GetPhysicsAngularVelocityInRadians();
        FVector angularV = rootLink->GetPhysicsLinearVelocity();

        RobotSim::Vector3r linearV3r(linearV.X, linearV.Y, linearV.Z);
        RobotSim::Vector3r angularV3r(angularV.X, angularV.Y, angularV.Z);
        SpeedMomentums.linear = linearV3r;
        SpeedMomentums.angular = angularV3r;
    }
    return SpeedMomentums;
}

RobotSim::Momentums RobotApi::getLinkSpeed(const FString& linkName) const
{
    RobotSim::Momentums speedMomentums;
    AUrdfLink* botLink = this->pawn_->GetLink(linkName);
    if (botLink)
    {
        FVector linearV =
            botLink->GetPhysicsAngularVelocityInRadians(); /* 每秒转动的弧度 */
        FVector angularV = botLink->GetPhysicsLinearVelocity();

        RobotSim::Vector3r linearV3r(linearV.X, linearV.Y, linearV.Z);
        RobotSim::Vector3r angularV3r(angularV.X, angularV.Y, angularV.Z);
        speedMomentums.linear = linearV3r;
        speedMomentums.angular = angularV3r;
    }
    return speedMomentums;
}

void RobotApi::setBotWheelRadius(float wheel_radius)
{
    TMap<FString, ControlledMotionComponent*> motionComponents =
        this->pawn_->GetBotMotionComponents();

    for (auto const& component : motionComponents)
    {
        component.Value->SetWheelRadius(wheel_radius);
    }
}

const RobotApi::UrdfBotState
RobotApi::getBotState(const std::string& vehicle_name)
{
    RobotApi::UrdfBotState botState;
    TMap<FString, AUrdfLink*> linkComponents = this->pawn_->GetLinkComponents();
    TMap<FString, ControlledMotionComponent*> motionComponents =
        this->pawn_->GetControlledMotionComponents();
    AUrdfLink* rootLinkComponent = this->pawn_->GetRootLinkComponent();

    FVector rootLinkLocation = rootLinkComponent->GetActorLocation();
    FRotator rootLinkRotation = rootLinkComponent->GetActorRotation();
    FQuat rootLinkRotationQuat = rootLinkRotation.Quaternion();

    FVector rootLinkVelocity = rootLinkComponent->GetPhysicsLinearVelocity();
    FVector rootLinkAngularVelocity =
        rootLinkComponent->GetPhysicsAngularVelocityInRadians();

    // pawn_kinematics_ is updated in PawnSim.cpp
    // botState.kinematics_estimated = *this->pawn_kinematics_;
    botState.kinematics_estimated = rootLinkComponent->GetKinematics();

    for (auto& kvp : linkComponents)
    {
        // TODO: add, acceleration
        LinkInformation currentLinkInformation;
        AUrdfLink* currentLink = kvp.Value;
        RobotSim::Pose relativePose;
        RobotSim::Twist relativeTwist;

        FVector relativeLocation =
            currentLink->GetActorLocation() - rootLinkLocation;
        FQuat relativeRotation =
            (currentLink->GetActorRotation() - rootLinkRotation).Quaternion();

        FVector relativeVelocity =
            currentLink->GetPhysicsLinearVelocity() - rootLinkVelocity;
        FVector relativeAngularVelocity =
            currentLink->GetPhysicsAngularVelocityInRadians() -
            rootLinkAngularVelocity;

        relativePose.position = RobotSim::Vector3r(
            relativeLocation[0], relativeLocation[1], relativeLocation[2]);
        relativePose.orientation =
            RobotSim::Quaternionr(relativeRotation.W, relativeRotation.X,
                                  relativeRotation.Y, relativeRotation.Z);

        relativeTwist.linear = RobotSim::Vector3r(
            relativeVelocity[0], relativeVelocity[1], relativeVelocity[2]);
        relativeTwist.angular = RobotSim::Vector3r(relativeAngularVelocity[0],
                                                   relativeAngularVelocity[1],
                                                   relativeAngularVelocity[2]);

        currentLinkInformation.link_name = TCHAR_TO_UTF8(*(kvp.Key));
        currentLinkInformation.link_relative_pose = relativePose;
        currentLinkInformation.link_relative_twist = relativeTwist;

        botState.link_information.emplace_back(currentLinkInformation);
    }

    for (auto const& kvp : motionComponents)
    {
        TMap<FString, FString> state = kvp.Value->GetState();

        std::map<std::string, std::string> state_converted;
        for (const auto& state_kvp : state)
        {
            state_converted[std::string(TCHAR_TO_UTF8(*(state_kvp.Key)))] =
                std::string(TCHAR_TO_UTF8(*(state_kvp.Value)));
        }

        std::string component_name = std::string(TCHAR_TO_UTF8(*(kvp.Key)));

        botState.controlled_motion_component_states[component_name] =
            state_converted;
    }

    return botState;
}

std::vector<RobotApi::BotJoint> RobotApi::getAllJoints()
{
    std::vector<RobotApi::BotJoint> arrayResJoints;
    RobotApi::BotJoint botJoint;
    uint32 index = 0;
    for (auto eachJoint : this->pawn_->getConstraints())
    {
        TTuple<UrdfJointType, UPhysicsConstraintComponent*> joint =
            eachJoint.Value;
        botJoint.mJointName = TCHAR_TO_UTF8(*eachJoint.Key);
        botJoint.mJointType = joint.Key;
        botJoint.mJointID = index++;
        arrayResJoints.push_back(botJoint);
    }
    return arrayResJoints;
}

void RobotApi::setJointTorque(FString jointName, FVector torque)
{
    // todo  this
    this->pawn_->setJointTorque(jointName, torque);
}

void RobotApi::setDrive(FString jointName, float stiffness, float damping)
{
    this->pawn_->setDrive(jointName, stiffness, damping);
}

void RobotApi::setDriveTarget(FString jointName, float target)
{
    this->pawn_->setDriveTarget(jointName, target);
}

void RobotApi::setQueueTorque(const RobotSim::QueueVector3r& queueTorque)
{
    // TMap<FString, ControlledMotionComponent*>&    motionComponents =
    // this->pawn_->GetControlledMotionComponents();

    ////motionComponents
    // uint32   queueSize = queueTorque.mVector3rArray.size();
    // const std::vector<RobotSim::Vector3r>&   vecArray =
    // queueTorque.mVector3rArray; uint32   mapSize = motionComponents.Num(); if
    // (queueSize == mapSize)
    //{
    //	uint32 index = 0;
    //	for (const auto element : motionComponents)
    //	{
    //		RobotSim::Vector3r  vec3r = vecArray[index++];
    //		element.Value->SetConstTorque(FVector(vec3r.x(), vec3r.y(),
    // vec3r.z()));
    //	}
    //}
}

RobotSim::Pose RobotApi::getBotPose() const
{
    AUrdfLink* rootLink = this->pawn_->GetRootLinkComponent();
    RobotSim::Pose linkpose;
    if (rootLink != nullptr)
    {
        linkpose = rootLink->GetPose();
    }
    return linkpose;
}

RobotSim::Pose RobotApi::getLinkPose(const FString& linkName) const
{
    RobotSim::Pose linkPose;
    // AUrdfBotPawn*  botPawn = this->ge
    AUrdfLink* linkPtr = this->pawn_->GetLink(linkName);
    if (linkPtr)
    {
        linkPose = linkPtr->GetPose();
    }
    return linkPose;
}

float RobotApi::getJointPose(const FString& jointName) const
{
    float actualPose = 0.0;
    /*TMap<FString, ControlledMotionComponent*>&    motionComponents =
    this->pawn_->GetControlledMotionComponents(); if
    (motionComponents.Contains(jointName))
    {
            ControlledMotionComponent*  jointComponent =
    motionComponents[jointName]; TMap<FString, FString> jointState =
    jointComponent->GetState(); FString  actualPoint =
    jointState["ActualPoint"]; actualPose = FCString::Atof(*actualPoint);
    }*/

    return actualPose;
}

float RobotApi::getJointVelocity(const FString& jointName) const
{
    float actualVelocity = 0.0;
    // TMap<FString, ControlledMotionComponent*>&    motionComponents =
    // this->pawn_->GetControlledMotionComponents(); if
    // (motionComponents.Contains(jointName))
    //{
    //	ControlledMotionComponent*  jointComponent =
    // motionComponents[jointName]; 	TMap<FString, FString> jointState =
    // jointComponent->GetState(); 	FString  actualPoint =
    // jointState["ActualSpeed"]; 	actualVelocity =
    // FCString::Atof(*actualPoint);
    //}

    return actualVelocity;
}

void RobotApi::reset()
{
    RobotSim::RobotApiBase::reset();

    // ToDo:  Reset all forces on object to  0.
    for (auto& linkKvp : this->pawn_->GetLinkComponents())
    {
        linkKvp.Value->ResetForceMagnitudes();
    }
}

void RobotApi::update()
{
    RobotSim::RobotApiBase::update();
}

RobotSim::GeoPoint RobotApi::getHomeGeoPoint() const
{
    return this->home_geopoint_;
}

void RobotApi::enableApiControl(bool is_enabled)
{
    this->api_control_enabled_ = is_enabled;
}

bool RobotApi::isApiControlEnabled() const
{
    return this->api_control_enabled_;
}

// TODO: Figure out what this does.
bool RobotApi::armDisarm(bool arm)
{
    unused(arm);
    return true;
}

void RobotApi::setAttachment()
{
    this->pawn_->onGrabObject();
}
