import numpy as np
import spear
from openbot_gym.openbot_agent import OpenBotAgent

# Autopilot class containing the vehicle's low level controller 
class OpenBotAutopilot:

    def __init__(self, config):
    self.kp_lin = config.AUTOPILOT.PROPORTIONAL_GAIN_DIST
    self.kd_lin = config.AUTOPILOT.DERIVATIVE_GAIN_DIST
    self.kp_ang = config.AUTOPILOT.PROPORTIONAL_GAIN_HEADING
    self.kd_ang = config.AUTOPILOT.DERIVATIVE_GAIN_HEADING
    self.acceptance_radius = config.AUTOPILOT.ACCEPTANCE_RADIUS
    self.forward_min_angle = config.AUTOPILOT.FORWARD_MIN_ANGLE
    self.control_saturation = config.AUTOPILOT.CONTROL_SATURATION
    self.dt = config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS
    
    # perform sanity checks
    assert self.kp_lin > 0.0
    assert self.kd_lin > 0.0
    assert self.kp_ang > 0.0
    assert self.kd_ang > 0.0
    assert self.acceptance_radius > 0.0
    assert self.forward_min_angle > 0.0
    assert self.control_saturation > 0.0
    assert self.dt > 0.0 

    def update(self, desired_position_xy, actual_pose_yaw_xy):
        target_location_reached = False
        forward_ctrl = 0.0
        right_ctrl = 0.0
        action = np.array([0.0,0.0], dtype=np.float32)
        delta_yaw = 0.0
        forward = np.array([1,0]) # Front axis is the X axis.
        forward_rotated = np.array([0,0])

        # target error vector (global coordinate system):
        relative_position_to_target = desired_position_xy - np.array([actual_pose_yaw_xy[1],actual_pose_yaw_xy[2]])
    
        # compute Euclidean distance to target in [m]:
        dist = np.linalg.norm(relative_position_to_target)*0.01

        # if the robot close enough to its target, consider the control task as done
        if dist < self.acceptance_radius :
            target_location_reached = True;

        # otherwise compute the autopilot command:
        else:
    
            # compute robot forward axis of the robot (in the global coordinate system):
            yaw_vehicle = actual_pose_yaw_xy[0];
            rot = np.array([[np.cos(yaw_vehicle), -np.sin(yaw_vehicle)], [np.sin(yaw_vehicle), np.cos(yaw_vehicle)]])

            forward_rotated = np.dot(rot, forward)

            # compute relative yaw to target:
            delta_yaw = np.arctan2(forward_rotated[1], forward_rotated[0]) - np.arctan2(relative_position_to_target[1], relative_position_to_target[0])

            # fit to range [-pi, pi]:
            if delta_yaw > np.pi:
                delta_yaw -= 2 * np.pi
            elif delta_yaw <= -np.pi:
                delta_yaw += 2 * np.pi

            # numerical diff:
            dXY = np.array([pos_x, pos_y]) - desired_position_xy
            linVelNorm = np.linalg.norm(dXY/dt)
            yawVel = dYaw/dt
            linVel = linVelNorm * 0.036; # in [m/s]

            right_ctrl = -self.kp_ang * delta_yaw - self.kd_ang * yawVel;
            right_ctrl = np.clip(right_ctrl, -self.control_saturation, self.control_saturation)

            if abs(delta_yaw) < forwardMinAngle :
            forward_ctrl = self.kp_lin * dist - self.kd_lin * linVel
            forward_ctrl = np.clip(forward_ctrl, -self.control_saturation, self.control_saturation)
            forward_ctrl *= abs(cos(delta_yaw)); # full throttle if the vehicle facing the objective. Otherwise give more priority to the yaw command.

            # compute action:
            leftWheelCommand = forward_ctrl + right_ctrl
            leftWheelCommand = np.clip(leftWheelCommand, -self.control_saturation, self.control_saturation)
    
            rightWheelCommand = forward_ctrl - right_ctrl
            rightWheelCommand = np.clip(rightWheelCommand, -self.control_saturation, self.control_saturation)
    
            action = np.array([leftWheelCommand,rightWheelCommand], dtype=np.float32)

        return action, target_location_reached
