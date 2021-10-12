#include "InverseKinematicApi.h"

InverseKinematicComponent::InverseKinematicComponent()
{
}

void InverseKinematicComponent::ImportModel(std::string FilePath,
                                            float ScaleFactor)
{
    this->invScaleFactor_ = 1.0f / ScaleFactor;
    Addons::URDFReadFromFile(FilePath.data(), model, false);
}

void InverseKinematicComponent::SetEndEffector(FString endEffectorName)
{
    this->eefName_ = std::string(TCHAR_TO_UTF8(*endEffectorName));
    int linkId = this->eefId_ =
        this->model->GetBodyId(TCHAR_TO_UTF8(*endEffectorName));

    while (linkId != 0)
    {
        linkId = model->GetParentBodyId(linkId);
        if (linkId > 0)
        {
            this->armJointIds.Add(linkId - 1);
        }
    }

    URobotBlueprintLib::LogMessage(FString("hello world"),
                                   FString("SetEndEffector"),
                                   LogDebugLevel::Failure, 30);
}

bool InverseKinematicComponent::CalculateIK(TMap<FString, float> QInitMap,
                                            FString endEffectorName,
                                            FVector targetPos,
                                            FQuat targetOri,
                                            TMap<FString, float>& resultMap)
{
    int body_id = model->GetBodyId(TCHAR_TO_UTF8(*endEffectorName));

    VectorNd Qinit = VectorNd::Zero(model->q_size);
    for (int& jointId : this->armJointIds)
    {
        FString linkName(model->GetBodyName(jointId + 1).c_str());
        if (QInitMap.Contains(linkName))
        {
            Qinit(jointId) = QInitMap[linkName];
        }
    }

    FMatrix tempOriMat = FRotationMatrix::Make(targetOri);
    Matrix3d targetOriMat(Matrix3d::Identity(3, 3));
    targetOriMat << tempOriMat.M[0][0], tempOriMat.M[0][1], tempOriMat.M[0][2],
        tempOriMat.M[1][0], tempOriMat.M[1][1], tempOriMat.M[1][2],
        tempOriMat.M[2][0], tempOriMat.M[2][1], tempOriMat.M[2][2];

    // Calculate IK for given pose
    VectorNd Qres(Qinit);
    bool result = this->CalculateIKWithControlledTorso(
        Qinit, Qres, body_id,
        this->invScaleFactor_ * Vector3d(targetPos.X, targetPos.Y, targetPos.Z),
        targetOriMat);
    if (result)
    {
        // return target q list as map
        for (int& jointId : this->armJointIds)
        {
            resultMap.Add(FString(model->GetBodyName(jointId + 1).c_str()),
                          Qres(jointId));
        }
        // scale prismatic joint
        //resultMap["torso_lift_link"] =
        //    resultMap["torso_lift_link"] / this->invScaleFactor_;
    }
    return result;
}

bool InverseKinematicComponent::CalculateIKWithControlledTorso(
    const Math::VectorNd& Qinit,
    Math::VectorNd& Qres,
    int body_id,
    Vector3d targetPos,
    Matrix3d targetOri)
{
    InverseKinematicsConstraintSet cs;
    // add end effector target pose
    cs.AddFullConstraint(body_id, Vector3d(0., 0., 0.), targetPos, targetOri);
    // limit torso_lift_link at current position
    //cs.AddPointConstraint(
    //    model->GetBodyId("torso_lift_link"), Vector3d(0., 0., 0.),
    //    Vector3d(-0.086875, 0., this->invScaleFactor_ * Qinit(2) + 0.37743));

    bool result = RigidBodyDynamics::InverseKinematics(*model, Qinit, cs, Qres);

    if (!result)
    {
        // try IK with unlimited torso_lift_link
        InverseKinematicsConstraintSet cs2;
        cs2.AddFullConstraint(body_id, Vector3d(0., 0., 0.), targetPos,
                              targetOri);
        result = RigidBodyDynamics::InverseKinematics(*model, Qinit, cs2, Qres);
        if (!result)
        {
            // unreachable regardless torso position
            return false;
        }
        float qTorsoLiftJoint = Qres(2);
        if (qTorsoLiftJoint > 0.38615)
        {
            // possible with torso beyond upper limit, try again with max torso
            // pose
            cs2.AddPointConstraint(model->GetBodyId("torso_lift_link"),
                                   Vector3d(0., 0., 0.),
                                   Vector3d(-0.086875, 0., 0.37743 + 0.38615));
            return RigidBodyDynamics::InverseKinematics(*model, Qinit, cs2,
                                                        Qres);
        }
        else if (qTorsoLiftJoint < 0)
        {
            // possible with torso beyond lower limit, try again with min torso
            // pose
            cs2.AddPointConstraint(model->GetBodyId("torso_lift_link"),
                                   Vector3d(0., 0., 0.),
                                   Vector3d(-0.086875, 0., 0.37743));
            return RigidBodyDynamics::InverseKinematics(*model, Qinit, cs2,
                                                        Qres);
        }
        else
        {
            // reachable with reasonable torso pose
            return true;
        }
    }
    else
    {
        // reachable at current torso pose
        return true;
    }
}

Matrix44 InverseKinematicComponent::ForwardKinematic(Math::VectorNd Q,
                                                     int body_id)
{
    Vector3d pos =
        CalcBodyToBaseCoordinates(*model, Q, body_id, Vector3d(0., 0., 0.));
    Matrix3d ori = CalcBodyWorldOrientation(*model, Q, body_id);

    Matrix44 pose(MatrixNd::Identity(4, 4));
    pose.block<3, 3>(0, 0) = ori;
    pose.block<3, 1>(0, 3) = pos;
    return pose;
}
