import numpy as np

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
        self.xy_position_old = np.array([0.0, 0.0], dtype=np.float32)
        self.yaw_old = 0.0
        self.first_update = True

        # perform sanity checks
        assert self.kp_lin >= 0.0
        assert self.kd_lin >= 0.0
        assert self.kp_ang >= 0.0
        assert self.kd_ang >= 0.0
        assert self.acceptance_radius >= 0.0
        assert self.forward_min_angle >= 0.0
        assert self.control_saturation >= 0.0
        assert self.dt > 0.0 

    def update(self, desired_position_xy, actual_pose_yaw_xy):

        # avoid discontinuities on the derivative
        if self.first_update: 
            self.xy_position_old = np.array([actual_pose_yaw_xy[1],actual_pose_yaw_xy[2]], dtype=np.float32)
            self.yaw_old = actual_pose_yaw_xy[0]
            self.first_update = False

        target_location_reached = False

        # target error vector (global coordinate system):
        xy_position = np.array([actual_pose_yaw_xy[1],actual_pose_yaw_xy[2]], dtype=np.float32)
        position_error = desired_position_xy - xy_position
    
        # compute Euclidean distance to target in [m]:
        position_error_norm = np.linalg.norm(position_error)*0.01

        # if the robot close enough to its target, consider the control task as done
        if position_error_norm < self.acceptance_radius :
            target_location_reached = True;
        # otherwise compute the autopilot command:
        else:
            # compute robot forward axis (in the global coordinate system):
            yaw_vehicle = actual_pose_yaw_xy[0];
            forward = np.array([1,0]) # front axis is the X axis.
            rot = np.array([[np.cos(yaw_vehicle), -np.sin(yaw_vehicle)], [np.sin(yaw_vehicle), np.cos(yaw_vehicle)]])
            forward_rotated = np.dot(rot, forward)

            # compute yaw error:
            yaw_error = np.arctan2(forward_rotated[1], forward_rotated[0]) - np.arctan2(position_error[1], position_error[0])

            # fit to range [-pi, pi]:
            if yaw_error > np.pi:
                yaw_error -= 2 * np.pi
            elif yaw_error <= -np.pi:
                yaw_error += 2 * np.pi

            # velocity computation
            lin_vel_norm = np.linalg.norm((xy_position-self.xy_position_old)/self.dt) * 0.036 
            self.xy_position_old = xy_position
            ang_vel_norm = yaw_error/self.dt # in [?]

            # compute angular component of motion 
            right_ctrl = -self.kp_ang * yaw_error - self.kd_ang * ang_vel_norm;
            right_ctrl = np.clip(right_ctrl, -self.control_saturation, self.control_saturation)

            # only compute the linear component of motion if the vehicle is facing its target (modulo forward_min_angle)
            forward_ctrl = 0.0
            if abs(yaw_error) <= self.forward_min_angle:
                forward_ctrl += self.kp_lin * position_error_norm - self.kd_lin * lin_vel_norm
                forward_ctrl = np.clip(forward_ctrl, -self.control_saturation, self.control_saturation)
                forward_ctrl *= abs(np.cos(yaw_error)); # full throttle if the vehicle facing the objective. Otherwise give more priority to the yaw command.

            # compute action for each wheel as the sum of the linear and angular control inputs
            left_wheel_command = forward_ctrl + right_ctrl
            left_wheel_command = np.clip(left_wheel_command, -self.control_saturation, self.control_saturation)
            right_wheel_command = forward_ctrl - right_ctrl
            right_wheel_command = np.clip(right_wheel_command, -self.control_saturation, self.control_saturation)
    
            action = np.array([left_wheel_command,right_wheel_command], dtype=np.float32)

        return action, target_location_reached
