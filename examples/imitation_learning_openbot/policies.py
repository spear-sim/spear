import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import time

from utils import get_compass_observation, get_relative_target_pose

# Autopilot class containing the vehicle's low level controller 
class OpenBotDrivingPolicy:

    def __init__(self, config):
        self.acceptance_radius = config.DRIVING_POLICY.ACCEPTANCE_RADIUS
        self.control_saturation = config.DRIVING_POLICY.CONTROL_SATURATION
        self.trajectory = np.empty((0, 3), dtype=float)
        self.position_xy_desired = np.empty((0, 2), dtype=float)
        self.position_xy_current = np.empty((0, 2), dtype=float)
        self.yaw_current = 0.0
        self.xy_position_error  = np.empty((0, 2), dtype=float)
        self.yaw_error = 0.0
        self.index_waypoint = 0
        self.num_waypoints = 0
        self.policy_step_info = {"current_waypoint" : np.empty((0, 2), dtype=float), "goal_reached" : False}
        
        # perform sanity checks
        assert self.acceptance_radius >= 0.0
        assert self.control_saturation >= 0.0


    def set_trajectory(self, trajectory):
        self.trajectory = trajectory
        self.num_waypoints = len(trajectory) - 1 # discarding the initial position
        self.policy_step_info["goal_reached"] = False 
        assert self.num_waypoints >= 0


    def update(self, obs):
        # xy position of the next waypoint in world frame:
        self.position_xy_desired = np.array([self.trajectory[self.index_waypoint][0], self.trajectory[self.index_waypoint][1]], dtype=np.float32) # [x_des, y_des]
        self.policy_step_info["current_waypoint"] = self.trajectory[self.index_waypoint]

        # current position and heading of the vehicle in world frame
        self.position_xy_current = np.array([obs["state_data"][0], obs["state_data"][1]], dtype=np.float32)
        self.yaw_current = obs["state_data"][4]
        
        # compute the relative agent-target pose from the raw observation dictionary
        self.xy_position_error, self.yaw_error = get_relative_target_pose(self.position_xy_desired, self.position_xy_current, self.yaw_current)

        # compute Euclidean distance to target in [m]:
        self.xy_position_error_norm = np.linalg.norm(self.xy_position_error) * 0.01
        
        # is the current waypoint close enough to be considered as "reached" ?
        waypoint_reached = (self.xy_position_error_norm <= self.acceptance_radius)

        # if a waypoint of the trajectory is "reached", based on the autopilot's acceptance_radius condition 
        if waypoint_reached: 
            if self.index_waypoint < self.num_waypoints: # if this waypoint is not the final "goal"
                print(f"Waypoint {self.index_waypoint} over {self.num_waypoints} reached !")
                self.index_waypoint += 1 # set the next way point as the current target to be tracked by the agent
                self.policy_step_info["goal_reached"] = False 
            else: # if this waypoint is the final "goal"
                self.policy_step_info["goal_reached"] = True 
        else:
            self.policy_step_info["goal_reached"] = False 

class OpenBotPIDPolicy(OpenBotDrivingPolicy):

    def __init__(self, config): 
        super().__init__(config)
        self.kp_lin = config.DRIVING_POLICY.PID.PROPORTIONAL_GAIN_DIST
        self.kd_lin = config.DRIVING_POLICY.PID.DERIVATIVE_GAIN_DIST
        self.kp_ang = config.DRIVING_POLICY.PID.PROPORTIONAL_GAIN_HEADING
        self.kd_ang = config.DRIVING_POLICY.PID.DERIVATIVE_GAIN_HEADING
        self.forward_min_angle = config.DRIVING_POLICY.PID.FORWARD_MIN_ANGLE
        self.dt = config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS
        self.position_xy_old = np.zeros(2, dtype=np.float32)
        self.yaw_old = 0.0
        self.first_update = True

        # perform sanity checks
        assert self.kp_lin >= 0.0
        assert self.kd_lin >= 0.0
        assert self.kp_ang >= 0.0
        assert self.kd_ang >= 0.0
        assert self.forward_min_angle >= 0.0
        assert self.dt > 0.0 
    

    def set_trajectory(self, trajectory):
        super().set_trajectory(trajectory) 
        self.index_waypoint = 1 # initialized to 1 as waypoint with index 0 refers to the agent initial position


    def update(self, obs):

        # update waypoint and check location reached
        super().update(obs)

        # avoid discontinuities on the derivative
        if self.first_update: 
            self.position_xy_old = self.position_xy_current
            self.yaw_old = self.yaw_current
            self.first_update = False

        # velocity computation
        lin_vel_norm = np.linalg.norm((self.position_xy_current - self.position_xy_old) / self.dt) * 0.01 
        self.position_xy_old = self.position_xy_current
        ang_vel_norm = (self.yaw_current - self.yaw_old)/self.dt 
        self.yaw_old = self.yaw_current

        # compute angular component of motion 
        right_ctrl = -self.kp_ang * self.yaw_error - self.kd_ang * ang_vel_norm;
        right_ctrl = np.clip(right_ctrl, -self.control_saturation, self.control_saturation)

        # only compute the linear component of motion if the vehicle is facing its target (modulo forward_min_angle)
        forward_ctrl = 0.0
        if abs(self.yaw_error) <= self.forward_min_angle:
            forward_ctrl += self.kp_lin * self.xy_position_error_norm - self.kd_lin * lin_vel_norm
            forward_ctrl = np.clip(forward_ctrl, -self.control_saturation, self.control_saturation)
            forward_ctrl *= abs(np.cos(self.yaw_error)); # full throttle if the vehicle facing the objective. Otherwise give more priority to the yaw command.

        # compute action for each wheel as the sum of the linear and angular control inputs
        left_wheel_command = forward_ctrl + right_ctrl
        left_wheel_command = np.clip(left_wheel_command, -self.control_saturation, self.control_saturation)
        right_wheel_command = forward_ctrl - right_ctrl
        right_wheel_command = np.clip(right_wheel_command, -self.control_saturation, self.control_saturation)
        action = np.array([left_wheel_command,right_wheel_command], dtype=np.float32)
            
        return action, self.policy_step_info
        
class OpenBotPilotNetPolicy(OpenBotDrivingPolicy):

    def __init__(self, config):
        super().__init__(config)

        # load the control policy
        self.interpreter = tflite.Interpreter(config.DRIVING_POLICY.PILOT_NET.PATH)
        self.interpreter.allocate_tensors()
        
        # get input and output tensor details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        # the policy takes two inputs: normalized rgb image and a 3D compass observation.
        print(f"Input details of the control policy: {self.input_details}")
        # the policy gives a 2D output consisting of the duty cycles to be sent to the left (resp. right) motors on a [0, 1] scale
        print(f"Output details of the control policy: {self.output_details}")
        
        self.img_input = np.zeros(self.input_details[0]["shape"], dtype=np.float32)
        self.cmd_input = np.zeros(self.input_details[1]["shape"], dtype=np.float32)

    
    def set_trajectory(self, trajectory):
        super().set_trajectory(trajectory) 
        self.index_waypoint = self.num_waypoints # initialized to final goal position


    def update(self, obs):

        # update waypoint and check location reached
        super().update(obs)
    
        # get the updated compass observation
        compass_observation = get_compass_observation(self.position_xy_desired, self.position_xy_current, self.yaw_current)
        
        # fill command tensor
        self.cmd_input[0][0] = compass_observation[0]
        self.cmd_input[0][1] = compass_observation[1]
        self.cmd_input[0][2] = compass_observation[2]
        self.interpreter.set_tensor(self.input_details[1]["index"], self.cmd_input)
    
        # preprocess (normalize + crop) visual observations:
        target_height = self.input_details[0]["shape"][1] 
        target_width = self.input_details[0]["shape"][2] 
        image_height = obs["camera_final_color"].shape[0]
        image_width = obs["camera_final_color"].shape[1]
        assert image_height-target_height >= 0.0
        assert image_width-target_width >= 0.0
        self.img_input = np.float32(obs["camera_final_color"][image_height-target_height:image_height, image_width-target_width:image_width])/255.0 # crop and normalization in the [0.0, 1.0] range
        
        # fill image tensor
        self.interpreter.set_tensor(self.input_details[0]["index"], np.expand_dims(self.img_input, axis=0))
            
        # run inference
        start_time = time.time()
        self.interpreter.invoke()
        stop_time = time.time()
        execution_time = (stop_time - start_time) * 1000
        print('Infererence time: {:.3f}ms'.format(execution_time))

        # generate an action from inference result
        tflite_output = np.clip(self.interpreter.get_tensor(self.output_details[0]["index"]).astype(np.float32), -1.0, 1.0)
        action = np.array([tflite_output[0][0],tflite_output[0][1]], dtype=np.float32)
    
        return action, self.policy_step_info
    
    
    
    
