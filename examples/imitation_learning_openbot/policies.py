#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear
import tensorflow as tf

class OpenBotPathFollowingPolicy():

    def __init__(self, config): 

        self._config = config

        # Our convention in this example is to store all data that comes directly from Unreal in the native format
        # exported by Unreal, i.e., centimeters and degrees. We eventually need to convert some of this data to
        # meters and radians, but we only do so in local temporary variables.
        self._path = None
        self._waypoint_index = None
        self._location_xy_prev = None
        self._rotation_yaw_prev = None
    
    def reset(self, obs, path):
        
        assert path.shape[0] >= 2 # path must contain at least an initial and goal location

        self._path = path
        self._waypoint_index = 1 # initialized to 1 because waypoint 0 is the agent's initial position
        self._location_xy_prev = obs["location"][0:2]
        self._rotation_yaw_prev = obs["rotation"][1]

    def step(self, obs):

        cm_to_m = 0.01

        # compute the location (x,y) error and heading (yaw) error
        location_xy_current = obs["location"][0:2] * cm_to_m
        location_xy_desired = self._path[self._waypoint_index][0:2] * cm_to_m
        location_xy_error   = np.linalg.norm(location_xy_desired - location_xy_current)

        rotation_yaw_current = np.deg2rad(obs["rotation"][1])
        heading_xy_current   = np.array([np.cos(rotation_yaw_current), np.sin(rotation_yaw_current)])
        heading_xy_desired   = (location_xy_desired - location_xy_current) / np.linalg.norm(location_xy_desired - location_xy_current)
        rotation_yaw_desired = np.arctan2(heading_xy_desired[1], heading_xy_desired[0])
        rotation_yaw_error   = rotation_yaw_desired - np.arctan2(heading_xy_current[1], heading_xy_current[0])

        if rotation_yaw_error < -np.pi:
            rotation_yaw_error += 2*np.pi
        if rotation_yaw_error > np.pi:
            rotation_yaw_error -= 2*np.pi

        location_xy_prev  = self._location_xy_prev * cm_to_m
        rotation_yaw_prev = np.deg2rad(self._rotation_yaw_prev)

        location_xy_speed  = np.linalg.norm(location_xy_current - location_xy_prev) / self._config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME
        rotation_yaw_speed = np.linalg.norm(rotation_yaw_current - rotation_yaw_prev) / self._config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME

        # multiply by -1.0 because otherwise a positive rotation error would cause us to turn right, but it should cause us to turn left
        control_right = -1.0 * \
            (rotation_yaw_error * self._config.IMITATION_LEARNING_OPENBOT.ROTATION_YAW_PROPORTIONAL_GAIN) + \
            (rotation_yaw_speed * self._config.IMITATION_LEARNING_OPENBOT.ROTATION_YAW_DERIVATIVE_GAIN)

        # multiply by an exponential decay term that prioritizes rotation errors, use LOCATION_XY_HALF_LIFE to control how aggressively to do so
        control_forward = np.power(2.0, -abs(rotation_yaw_error) / self._config.IMITATION_LEARNING_OPENBOT.LOCATION_XY_HALF_LIFE) * \
            ((location_xy_error * self._config.IMITATION_LEARNING_OPENBOT.LOCATION_XY_PROPORTIONAL_GAIN) + \
             (location_xy_speed * self._config.IMITATION_LEARNING_OPENBOT.LOCATION_XY_DERIVATIVE_GAIN))
 
        action = np.array([control_forward - control_right, control_forward + control_right], dtype=np.float64)
        action = np.clip(action, -self._config.IMITATION_LEARNING_OPENBOT.MAX_ACTION, self._config.IMITATION_LEARNING_OPENBOT.MAX_ACTION)

        # update member variables
        self._location_xy_prev = obs["location"][0:2]
        self._rotation_yaw_prev = obs["rotation"][1]

        # update waypoint state
        waypoint_reached = location_xy_error <= self._config.IMITATION_LEARNING_OPENBOT.WAYPOINT_REACHED_RADIUS

        if waypoint_reached:
            num_waypoints = self._path.shape[0] - 1 # don't count the initial position
            spear.log(f"    Waypoint {self._waypoint_index} of {num_waypoints} reached.")
            if self._waypoint_index < num_waypoints:                
                self._waypoint_index += 1

        step_info = {"rotation_yaw_desired": np.rad2deg(rotation_yaw_desired), "waypoint": self._path[self._waypoint_index]}

        return action, step_info


class OpenBotPilotNetPolicy():

    def __init__(self, config):

        self._config = config
        self._goal_location = None
        self._img_input_height = None
        self._img_input_width = None

        # load the control policy
        self._interpreter = tf.lite.Interpreter(self._config.IMITATION_LEARNING_OPENBOT.PILOT_NET_POLICY_PATH)
        self._interpreter.allocate_tensors()

        # get input and output tensor details
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()

        for input_detail in self._input_details:
            if "img_input" in input_detail["name"]:
                self._img_input_height = input_detail["shape"][1]
                self._img_input_width =input_detail["shape"][2]

        assert self._img_input_height and self._img_input_width

        # the policy takes two inputs: normalized rgb image and a 3D compass observation
        spear.log(f"Input details of the control policy: {self._input_details}")

        # the policy produces a 2D output consisting of the duty cycles to be sent to the left and right motors on a [0, 1] scale
        spear.log(f"Output details of the control policy: {self._output_details}")

    def reset(self, goal_location):
        assert goal_location.shape[1] == 3
        self._goal_location = goal_location

    def step(self, obs):

        # compute an image observation
        obs_height = obs["camera.final_color"].shape[0]
        obs_width  = obs["camera.final_color"].shape[1]

        assert obs_height >= self._img_input_height
        assert obs_width >= self._img_input_width

        img_input = obs["camera.final_color"][np.newaxis, obs_height-self._img_input_height:, obs_width-self._img_input_width:, :-1].astype(np.float32) / 255.0

        # compute a goal observation
        cm_to_m = 0.01

        location_xy_current = obs["location"][0:2] * cm_to_m
        location_xy_desired = self._goal_location[0, 0:2] * cm_to_m
        location_xy_error   = np.linalg.norm(location_xy_desired - location_xy_current)

        rotation_yaw_current = np.deg2rad(obs["rotation"][1])
        heading_xy_current   = np.array([np.cos(rotation_yaw_current), np.sin(rotation_yaw_current)])
        heading_xy_desired   = (location_xy_desired - location_xy_current) / np.linalg.norm(location_xy_desired - location_xy_current)
        rotation_yaw_error   = np.arctan2(heading_xy_desired[1], heading_xy_desired[0]) - np.arctan2(heading_xy_current[1], heading_xy_current[0])

        if rotation_yaw_error < -np.pi:
            rotation_yaw_error += 2*np.pi
        if rotation_yaw_error > np.pi:
            rotation_yaw_error -= 2*np.pi

        goal_input = np.array([[location_xy_error, np.sin(rotation_yaw_error), np.cos(rotation_yaw_error)]], dtype=np.float32)

        set_img_input = False
        set_goal_input = False
        for input_detail in self._input_details:
            if "img_input" in input_detail["name"]:
                self._interpreter.set_tensor(input_detail["index"], img_input)
                set_img_input = True
            elif "goal_input" in input_detail["name"]:
                self._interpreter.set_tensor(input_detail["index"], goal_input)
                set_goal_input = True
            else:
                assert False

        assert set_img_input and set_goal_input

        # run inference
        self._interpreter.invoke()

        # generate an action from inference result
        tf_output = self._interpreter.get_tensor(self._output_details[0]["index"]).astype(np.float64)
        action = np.clip(tf_output[0], -1.0, 1.0)

        return action
