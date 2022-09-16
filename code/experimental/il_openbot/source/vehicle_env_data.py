import math
import numpy as np
import csv
import datetime
from vehicle_env import VehicleEnv


class VehicleEnv_Data(VehicleEnv):
    def __init__(self, **kwargs):
        super().__init__(log_obs_dim=11,
                         **kwargs)

    def _fill_database(self, executed_steps):
        print("Filling database...")
        # Low-level commands sent to the motors
        f_ctrl = open(self.datafolder_name + "sensor_data/ctrlLog.txt", 'w',
                      encoding="utf-8")

        # High level commands
        f_goal = open(self.datafolder_name + "sensor_data/goalLog.txt", 'w',
                      encoding="utf-8")

        # Reference of the images correespoinding to each control input
        f_rgb = open(self.datafolder_name + "sensor_data/rgbFrames.txt", 'w',
                     encoding="utf-8")

        # Raw pose data (for debug purposes and (also) to prevent one from
        # having to re-run the data collection in case of a deg2rad issue...)
        f_pose = open(self.datafolder_name + "sensor_data/poseData.txt", 'w',
                      encoding="utf-8")

        writer_ctrl = csv.writer(f_ctrl, delimiter=",")
        writer_ctrl.writerow(
            ('timestamp[ns]', 'leftCtrl', 'rightCtrl'))

        writer_pose = csv.writer(f_pose, delimiter=",")
        writer_pose.writerow(
            ('timestamp[ns]', 'posX', 'posY', 'posZ',
             'rollAngle', 'pitchAngle', 'yawAngle'))

        writer_goal = csv.writer(f_goal, delimiter=",")
        writer_goal.writerow(
            ('timestamp[ns]', 'dist', 'sin_yaw', 'cos_yaw'))

        writer_rgb = csv.writer(f_rgb, delimiter=",")
        writer_rgb.writerow(('timestamp[ns]', 'frame'))

        # use the vehicle last location as goal
        goal_location = np.array(
            [self.array_obs[executed_steps-1][2],
             self.array_obs[executed_steps-1][3]])

        for i in range(executed_steps):

            # Target error vector (global coordinate system):
            relative_position_to_target = goal_location - \
                np.array([self.array_obs[i][2], self.array_obs[i][3]])

            # Compute Euclidean distance to target:
            dist = np.linalg.norm(relative_position_to_target)

            # Compute robot forward axis (global coordinate system):
            actual_forward_angle = self._get_actual_forward_angle(
                yaw_vehicle=self.array_obs[i][7])

            desired_forward_angle = self._get_desired_forward_angle(
                relative_position_to_target)

            # Compute delta yaw:
            delta_yaw = self._get_delta_yaw(
                actual_forward_angle=actual_forward_angle,
                desired_forward_angle=desired_forward_angle)

            # Check the actual OpenBot code:
            # https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
            sin_yaw = math.sin(delta_yaw)
            cos_yaw = math.cos(delta_yaw)

            # Write pose data
            writer_pose.writerow((int(self.array_obs[i][10]),
                                  self.array_obs[i][2],
                                  self.array_obs[i][3],
                                  self.array_obs[i][4],
                                  self.array_obs[i][5],
                                  self.array_obs[i][6],
                                  self.array_obs[i][7]))

            # Write the low-level control observation into a file:
            writer_ctrl.writerow((int(self.array_obs[i][10]),
                                  self.array_obs[i][0],
                                  self.array_obs[i][1]))

            # Write the corresponding image index into a file:
            writer_rgb.writerow((int(self.array_obs[i][10]), i))

            # Write the corresponding high level command into a file:
            # For imitation learning, use the latest position as a goal
            writer_goal.writerow(
                (int(self.array_obs[i][10]), dist/100, sin_yaw, cos_yaw))

        f_ctrl.close()
        f_pose.close()
        f_goal.close()
        f_rgb.close()

    def _fill_array_obs(self, step_count):
        obs = self.get_observation()

        # Fill an array with the different observations:
        self.array_obs[step_count][0] = self.speed_multiplier * \
            obs["control_data"][0]  # ctrl left
        self.array_obs[step_count][1] = self.speed_multiplier * \
            obs["control_data"][1]  # ctrl right

        # agent pos X wrt. world
        self.array_obs[step_count][2] = obs["state_data"][0]
        # agent pos Y wrt. world
        self.array_obs[step_count][3] = obs["state_data"][1]
        # agent pos Z wrt. world
        self.array_obs[step_count][4] = obs["state_data"][2]
        # agent Roll wrt. world
        self.array_obs[step_count][5] = obs["state_data"][3]
        # agent Pitch wrt. world
        self.array_obs[step_count][6] = obs["state_data"][4]
        # agent Yaw wrt. world
        self.array_obs[step_count][7] = obs["state_data"][5]

        # desired (waypoint) agent pos X wrt. world
        self.array_obs[step_count][8] = self.desired_position_xy[0]
        # desired (waypoint) agent pos Y wrt. world
        self.array_obs[step_count][9] = self.desired_position_xy[1]

        timestamp = 10000*datetime.datetime.now().timestamp()
        self.array_obs[step_count][10] = timestamp

    def _get_action(self):
        if self._target_waypoint_reached():
            # If reached target location, do nothing
            action = np.array([0.0, 0.0])
            return action

        lin_vel, yaw_vel = self._get_velocities()

        forward_ctrl = 0.0
        delta_yaw = self._get_delta_yaw()

        # Compute forward PID command:
        if abs(delta_yaw) < self._config.ROBOT_SIM.FORWARD_MIN_ANGLE:
            kp_lin = self._config.ROBOT_SIM.PROPORTIONAL_GAIN_DIST
            kd_lin = self._config.ROBOT_SIM.DERIVATIVE_GAIN_DIST
            # Full throttle if the vehicle facing the objective.
            # Otherwise give more priority to the yaw command.
            dist = self._get_dist_to_goal_xy()
            forward_ctrl = kp_lin * dist - kd_lin * lin_vel
            forward_ctrl = self._clamp(forward_ctrl)
            forward_ctrl *= abs(math.cos(delta_yaw))

        # Compute yaw PID command:
        kp_ang = self._config.ROBOT_SIM.PROPORTIONAL_GAIN_HEADING
        kd_ang = self._config.ROBOT_SIM.DERIVATIVE_GAIN_HEADING
        yaw_ctrl = -kp_ang * delta_yaw - kd_ang * yaw_vel
        yaw_ctrl = self._clamp(yaw_ctrl)

        # Compute action:
        left_wheel_command = forward_ctrl + yaw_ctrl
        left_wheel_command = self._clamp(left_wheel_command)

        right_wheel_command = forward_ctrl - yaw_ctrl
        right_wheel_command = self._clamp(right_wheel_command)

        action = np.array([left_wheel_command, right_wheel_command])

        return action
