#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import tflite_runtime.interpreter as tflite

from utils import get_compass_observation, get_relative_target_pose


class OpenBotPIDPolicy():

    def __init__(self, config): 

        self._config = config
        self._trajectory = np.empty((0, 3), dtype=float)
        self._index_waypoint = 0
        self._position_xy_old = np.zeros(2, dtype=np.float32)
        self._yaw_old = 0.0

        assert self._config.IMITATION_LEARNING_OPENBOT.ACCEPTANCE_RADIUS >= 0.0
        assert self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION >= 0.0
        assert self._config.IMITATION_LEARNING_OPENBOT.PID.PROPORTIONAL_GAIN_DIST >= 0.0
        assert self._config.IMITATION_LEARNING_OPENBOT.PID.DERIVATIVE_GAIN_DIST >= 0.0
        assert self._config.IMITATION_LEARNING_OPENBOT.PID.PROPORTIONAL_GAIN_HEADING >= 0.0
        assert self._config.IMITATION_LEARNING_OPENBOT.PID.DERIVATIVE_GAIN_HEADING >= 0.0
        assert self._config.IMITATION_LEARNING_OPENBOT.PID.FORWARD_MIN_ANGLE >= 0.0
        assert self._config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS > 0.0 
    
    def reset(self, obs, env_step_info):
        
        self._trajectory = env_step_info["agent_step_info"]["trajectory_data"]
        assert len(self._trajectory) >= 1
        self._index_waypoint = 1 # initialized to 1 as waypoint with index 0 refers to the agent initial position
        self._position_xy_old = obs["state_data"][0:2]
        self._yaw_old = obs["state_data"][4]

    def step(self, obs):

        # xy position of the next waypoint in world frame:
        position_xy_desired = self._trajectory[self._index_waypoint][0:2] # [x_des, y_des]

        # compute the relative agent-target pose from the raw observation dictionary
        xy_position_error, yaw_error = get_relative_target_pose(position_xy_desired, obs["state_data"][0:2], obs["state_data"][4])

        # compute Euclidean distance to target in [m]:
        xy_position_error_norm = np.linalg.norm(xy_position_error) * 0.01

        # velocity computation
        lin_vel_norm = np.linalg.norm((obs["state_data"][0:2] - self._position_xy_old) / self._config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS) * 0.01 
        ang_vel_norm = (obs["state_data"][4] - self._yaw_old)/self._config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS
        
        # compute angular component of motion 
        right_ctrl = -self._config.IMITATION_LEARNING_OPENBOT.PID.PROPORTIONAL_GAIN_HEADING * yaw_error - self._config.IMITATION_LEARNING_OPENBOT.PID.DERIVATIVE_GAIN_HEADING * ang_vel_norm;
        right_ctrl = np.clip(right_ctrl, -self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION, self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION)

        # only compute the linear component of motion if the vehicle is facing its target (modulo config.IMITATION_LEARNING_OPENBOT.PID.FORWARD_MIN_ANGLE)
        forward_ctrl = 0.0
        if abs(yaw_error) <= self._config.IMITATION_LEARNING_OPENBOT.PID.FORWARD_MIN_ANGLE:
            forward_ctrl += self._config.IMITATION_LEARNING_OPENBOT.PID.PROPORTIONAL_GAIN_DIST * xy_position_error_norm - self._config.IMITATION_LEARNING_OPENBOT.PID.DERIVATIVE_GAIN_DIST * lin_vel_norm
            forward_ctrl = np.clip(forward_ctrl, -self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION, self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION)
            forward_ctrl *= abs(np.cos(yaw_error)); # full throttle if the vehicle facing the objective. Otherwise give more priority to the yaw command.

        # compute action for each wheel as the sum of the linear and angular control inputs
        left_wheel_command = forward_ctrl + right_ctrl
        left_wheel_command = np.clip(left_wheel_command, -self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION, self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION)
        right_wheel_command = forward_ctrl - right_ctrl
        right_wheel_command = np.clip(right_wheel_command, -self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION, self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION)
        action = np.array([left_wheel_command,right_wheel_command], dtype=np.float32)

        # update member variables
        self._position_xy_old = obs["state_data"][0:2]
        self._yaw_old = obs["state_data"][4]
        
        # is the current waypoint close enough to be considered as "reached" ?
        waypoint_reached = (xy_position_error_norm <= self._config.IMITATION_LEARNING_OPENBOT.ACCEPTANCE_RADIUS)

        # if a waypoint of the trajectory is "reached", based on the autopilot's config.IMITATION_LEARNING_OPENBOT.ACCEPTANCE_RADIUS condition 
        if waypoint_reached: 
            num_waypoints = len(self._trajectory) - 1 # discarding the initial position
            if self._index_waypoint < num_waypoints:  # if this waypoint is not the final "goal"
                spear.log(f"Waypoint {self._index_waypoint} of {num_waypoints} reached.")
                self._index_waypoint += 1 # set the next way point as the current target to be tracked by the agent
                goal_reached = False 
            else: # if this waypoint is the final "goal"
                goal_reached = True 
        else:
            goal_reached = False 

        # compute step_info object
        step_info = {"current_waypoint": self._trajectory[self._index_waypoint], "waypoint_reached": waypoint_reached, "goal_reached": goal_reached}

        return action, step_info


class OpenBotPilotNetPolicy():

    def __init__(self, config):

        self._config = config

        assert self._config.IMITATION_LEARNING_OPENBOT.ACCEPTANCE_RADIUS >= 0.0
        assert self._config.IMITATION_LEARNING_OPENBOT.CONTROL_SATURATION >= 0.0

        # load the control policy
        self._interpreter = tflite.Interpreter(config.IMITATION_LEARNING_OPENBOT.PILOT_NET.PATH)
        self._interpreter.allocate_tensors()
        
        # get input and output tensor details
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        
        # the policy takes two inputs: normalized rgb image and a 3D compass observation.
        spear.log(f"Input details of the control policy: {self._input_details}")

        # the policy gives a 2D output consisting of the duty cycles to be sent to the left (resp. right) motors on a [0, 1] scale
        spear.log(f"Output details of the control policy: {self._output_details}")

    def step(self, obs, position_xy_desired):
    
        for input_detail in self._input_details:

            if "img_input" in input_detail["name"]:

                # preprocess (normalize + crop) visual observations:
                target_height = input_detail["shape"][1] 
                target_width = input_detail["shape"][2] 
                image_height = obs["camera.final_color"].shape[0]
                image_width = obs["camera.final_color"].shape[1]
                assert image_height-target_height >= 0.0
                assert image_width-target_width >= 0.0
                img_input = np.float32(obs["camera.final_color"][image_height-target_height:image_height, image_width-target_width:image_width])/255.0 # crop and normalization in the [0.0, 1.0] range
                self._interpreter.set_tensor(input_detail["index"], img_input[np.newaxis])

            elif "cmd_input" in input_detail["name"] or "goal_input" in input_detail["name"] :
                
                # get the updated compass observation
                compass_observation = get_compass_observation(position_xy_desired, obs["state_data"][0:2], obs["state_data"][4])
                self._interpreter.set_tensor(input_detail["index"], compass_observation[np.newaxis])   

            else:
                assert False
        
        # run inference
        self._interpreter.invoke()

        # generate an action from inference result
        tflite_output = np.clip(self._interpreter.get_tensor(self._output_details[0]["index"]).astype(np.float32), -1.0, 1.0)
        action = np.array([tflite_output[0][0],tflite_output[0][1]], dtype=np.float32)

        # compute the relative agent-target pose from the raw observation dictionary
        xy_position_error, _ = get_relative_target_pose(position_xy_desired, obs["state_data"][0:2], obs["state_data"][4])

        # compute Euclidean distance to target in [m]:
        xy_position_error_norm = np.linalg.norm(xy_position_error) * 0.01

        # is the current waypoint close enough to be considered as "reached" ?
        step_info = {"goal_reached" : (xy_position_error_norm <= self._config.IMITATION_LEARNING_OPENBOT.ACCEPTANCE_RADIUS)}
    
        return action, step_info
