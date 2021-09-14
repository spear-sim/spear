#pragma once
#include "common_utils/VectorMath.hpp"
#include "common_utils/VehicleApiBase.hpp"

#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "UrdfBot/UrdfParser/UrdfJointSpecification.h"
#include "UrdfBot/ControlledMotionComponent/ControlledMotionComponent.h"


class AUrdfBotPawn;

class RobotApi : public RobotSim::VehicleApiBase {

public:
	struct AddLinearForce {
		std::string force_name;
		std::string link_name;
		RobotSim::Vector3r application_point;
		RobotSim::Vector3r axis;

		AddLinearForce() {};

		AddLinearForce(std::string force_name, std::string link_name, RobotSim::Vector3r application_point, RobotSim::Vector3r axis)
			: force_name(force_name), link_name(link_name), application_point(application_point), axis(axis) {};
	};

	struct AddAngularForce {
		std::string force_name;
		std::string link_name;
		RobotSim::Vector3r axis;

		AddAngularForce() {};

		AddAngularForce(std::string force_name, std::string link_name, RobotSim::Vector3r axis)
			: force_name(force_name), link_name(link_name), axis(axis) {};
	};

	struct UpdateForceMagnitude {
		std::string force_name;
		float magnitude;

		UpdateForceMagnitude() {};

		UpdateForceMagnitude(std::string force_name, float magnitude)
			: force_name(force_name), magnitude(magnitude) {};
	};

	struct LinkInformation {
		std::string link_name;
		RobotSim::Pose link_relative_pose;
		RobotSim::Twist link_relative_twist;

		LinkInformation() {};

		LinkInformation(std::string link_name, RobotSim::Pose link_relative_pose, RobotSim::Twist link_relative_twist)
			: link_name(link_name), link_relative_pose(link_relative_pose), link_relative_twist(link_relative_twist) {};
	};

	struct UrdfBotState {
		std::vector<LinkInformation> link_information;
		RobotSim::Kinematics::State kinematics_estimated;
		std::map<std::string, std::map<std::string, std::string>> controlled_motion_component_states;

		UrdfBotState() {};

		UrdfBotState(std::vector<LinkInformation> &link_information, RobotSim::Kinematics::State kinematics_estimated, std::map<std::string, std::map<std::string, std::string>> controlled_motion_component_states)
			: link_information(link_information), kinematics_estimated(kinematics_estimated), controlled_motion_component_states(controlled_motion_component_states)
		{
		}
	};

	struct BotMotionComponentSpeed
	{
		std::map<std::string, float> motion_component_speed;

		BotMotionComponentSpeed() {};
		BotMotionComponentSpeed(std::map<std::string, float> control_motion_speed)
			:motion_component_speed(control_motion_speed)
		{
		}
	};

	struct UrdfBotControlledMotionComponentControlSignal {
		std::string component_name;
		std::map<std::string, float> control_signal_values;

		UrdfBotControlledMotionComponentControlSignal() {};

		UrdfBotControlledMotionComponentControlSignal(std::string component_name, std::map<std::string, float> control_signal_values)
			: component_name(component_name), control_signal_values(control_signal_values)
		{
		}
	};

	struct JointLimit
	{
		float	lower;
		float	upper;
		float	effort;
		float	velocity;
		JointLimit() {};
		JointLimit(float lo, float up, float ef, float ve) : lower(lo), upper(up), effort(ef), velocity(ve) {};
	};

	struct BotJoint
	{
		std::string		mJointName;
		UrdfJointType	mJointType;
		uint32			mJointID;
		float			mFriction;
		float			mStiffness;
		float			mDamping;
		JointLimit		mJointLimit;
		BotJoint() {};
		BotJoint(std::string joint_name, UrdfJointType joint_type, uint32 joint_id, float friction, float stiffness, float damping, JointLimit joint_limit) :
			mJointName(joint_name), mJointType(joint_type), mJointID(joint_id), mFriction(friction), mStiffness(stiffness), mDamping(damping), mJointLimit(joint_limit)
		{
		}
	};

	struct BotAllJoints
	{
		std::vector<RobotApi::BotJoint>   mAllJoints;
		BotAllJoints() {};
	};

	RobotApi(AUrdfBotPawn* pawn, const RobotSim::Kinematics::State* pawn_kinematics, const RobotSim::GeoPoint& home_geopoint,
		std::function<const RobotSim::Kinematics::State*(std::string)> state_provider_fxn, const RobotSim::RobotSimSettings::VehicleSetting* vehicle_setting,
		const RobotSim::Environment& environment);
	virtual ~RobotApi();

	void addLinearForce(const AddLinearForce& added_force, const std::string& vehicle_name) ;
	void addAngularForce(const AddAngularForce& added_force, const std::string& vehicle_name) ;
	void updateForceMagnitude(const UpdateForceMagnitude& updated_force, const std::string& vehicle_name) ;
	 void updateControlledMotionComponentControlSignal(const UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name) ;
	 void setAbsoluteJointPose(const UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name) ;
	 void setRelativeJointPose(const UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name) ;
	 void setBotInstantSpeed(const UrdfBotControlledMotionComponentControlSignal& updated_control_signal) ;
	 RobotSim::Momentums getBotInstantSpeed() const ;
	 RobotSim::Momentums  getLinkSpeed(const FString& linkName) const;
	 void  setBotWheelRadius(float wheel_radius) ;
	 const UrdfBotState getBotState(const std::string& vehicle_name) ;
	 std::vector<BotJoint>  getAllJoints()  ;
	 void setJointTorque(FString jointName, FVector torque) ;
	 void setDrive(FString jointName, float stiffness, float damping) ;
	 void setDriveTarget(FString jointName, float target) ;
	 void setQueueTorque(const RobotSim::QueueVector3r& queueTorque) ;
	 RobotSim::Pose getBotPose() const ;
	 RobotSim::Pose getLinkPose(const FString& linkName) const ;
	 float getJointPose(const FString& jointName) const ;
	 float getJointVelocity(const FString& jointName) const ;


public:
	virtual void reset() override;
	virtual void update() override;
	virtual RobotSim::GeoPoint getHomeGeoPoint() const override;
	virtual void enableApiControl(bool is_enabled) override;
	virtual bool isApiControlEnabled() const override;
	virtual bool armDisarm(bool arm) override;
	void setAttachment() ;

private:
	bool api_control_enabled_ = false;
	AUrdfBotPawn* pawn_;
	const RobotSim::Kinematics::State* pawn_kinematics_;
	RobotSim::GeoPoint home_geopoint_;

	TMap<FString, AUrdfLink*> force_link_mapping_;

};