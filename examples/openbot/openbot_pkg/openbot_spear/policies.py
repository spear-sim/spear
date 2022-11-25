import numpy as np
import tensorflow as tf
import tflite_runtime.interpreter as tflite
from openbot_spear.utils import get_compass_observation

# Autopilot class containing the vehicle's low level controller 
class OpenBotDrivingPolicy:

    def __init__(self, config):
        self.acceptance_radius = config.DRIVING_POLICY.ACCEPTANCE_RADIUS
        self.control_saturation = config.DRIVING_POLICY.CONTROL_SATURATION
        
        # perform sanity checks
        assert self.acceptance_radius >= 0.0
        assert self.control_saturation >= 0.0
            
    def update(desired_position_xy, obs):
        return np.zeros(2, dtype=np.float32)

class OpenBotPID(OpenBotDrivingPolicy):

    def __init__(self, config): 
        super().__init__(config)
        self.kp_lin = config.DRIVING_POLICY.PID.PROPORTIONAL_GAIN_DIST
        self.kd_lin = config.DRIVING_POLICY.PID.DERIVATIVE_GAIN_DIST
        self.kp_ang = config.DRIVING_POLICY.PID.PROPORTIONAL_GAIN_HEADING
        self.kd_ang = config.DRIVING_POLICY.PID.DERIVATIVE_GAIN_HEADING
        self.forward_min_angle = config.DRIVING_POLICY.PID.FORWARD_MIN_ANGLE
        self.dt = config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS
        self.xy_position_old = np.zeros(2, dtype=np.float32)
        self.yaw_old = 0.0
        self.first_update = True

        # perform sanity checks
        assert self.kp_lin >= 0.0
        assert self.kd_lin >= 0.0
        assert self.kp_ang >= 0.0
        assert self.kd_ang >= 0.0
        assert self.forward_min_angle >= 0.0
        assert self.dt > 0.0 

    def update(desired_position_xy, obs):
    
        # current position and heading of the vehicle in world frame
        current_pose_yaw_xy = np.array([obs["state_data"][4], obs["state_data"][0], obs["state_data"][1]], dtype=np.float32) # [yaw, x, y]
        
        # compute the relative agent-target pose from the raw observation dictionary
        xy_position_error, yaw_error = get_relative_target_pose(desired_position_xy, current_pose_yaw_xy)

        # avoid discontinuities on the derivative
        if self.first_update: 
            self.xy_position_old = np.array([current_pose_yaw_xy[1],current_pose_yaw_xy[2]], dtype=np.float32)
            self.yaw_old = obs["state_data"][4]
            self.first_update = False

        # compute Euclidean distance to target in [m]:
        xy_position_error_norm = np.linalg.norm(position_error) * 0.01
        
        # velocity computation
        lin_vel_norm = np.linalg.norm((np.array([current_pose_yaw_xy[1],current_pose_yaw_xy[2]], dtype=np.float32)-self.xy_position_old)/self.dt) * 0.01 
        self.xy_position_old = np.array([current_pose_yaw_xy[1],current_pose_yaw_xy[2]], dtype=np.float32)
        ang_vel_norm = (obs["state_data"][4] - self.yaw_old)/self.dt 
        self.yaw_old = obs["state_data"][4]

        # compute angular component of motion 
        right_ctrl = -self.kp_ang * yaw_error - self.kd_ang * ang_vel_norm;
        right_ctrl = np.clip(right_ctrl, -self.control_saturation, self.control_saturation)

        # only compute the linear component of motion if the vehicle is facing its target (modulo forward_min_angle)
        forward_ctrl = 0.0
        if abs(yaw_error) <= self.forward_min_angle:
            forward_ctrl += self.kp_lin * xy_position_error_norm - self.kd_lin * lin_vel_norm
            forward_ctrl = np.clip(forward_ctrl, -self.control_saturation, self.control_saturation)
            forward_ctrl *= abs(np.cos(yaw_error)); # full throttle if the vehicle facing the objective. Otherwise give more priority to the yaw command.

        # compute action for each wheel as the sum of the linear and angular control inputs
        left_wheel_command = forward_ctrl + right_ctrl
        left_wheel_command = np.clip(left_wheel_command, -self.control_saturation, self.control_saturation)
        right_wheel_command = forward_ctrl - right_ctrl
        right_wheel_command = np.clip(right_wheel_command, -self.control_saturation, self.control_saturation)
        action = np.array([left_wheel_command,right_wheel_command], dtype=np.float32)
            
        # is goal reached ?
        target_location_reached = (xy_position_error_norm <= self.acceptance_radius)

        return action, target_location_reached
        
class OpenBotPilotNet(OpenBotDrivingPolicy):

    def __init__(self, config):
        super().__init__(config)
        self.kp_lin = config.DRIVING_POLICY.PILOT_NET.PATH
        
        # load the control policy
        self.interpreter = tflite.Interpreter(policy_name)
        self.interpreter.allocate_tensors()
        
        # get input and output tensor details
        self.input_details = interpreter.get_input_details()
        self.output_details = interpreter.get_output_details()
        # the policy takes two inputs: normalized rgb image and a 3D compass observation.
        print(f"Input details of the control policy: {self.input_details}")
        # the policy gives a 2D output consisting of the duty cycles to be sent to the left (resp. right) motors on a [0, 1] scale
        print(f"Output details of the control policy: {self.output_details}")
    
    def update(desired_position_xy, obs):
    
        # current position and heading of the vehicle in world frame
        current_pose_yaw_xy = np.array([obs["state_data"][4], obs["state_data"][0], obs["state_data"][1]], dtype=np.float32) # [yaw, x, y]
        
        # get the updated compass observation
        compass_observation = get_compass_observation(desired_position_xy, current_pose_yaw_xy)
        
        # fill command tensor
        cmd_input[0][0] = compass_observation[0]
        cmd_input[0][1] = compass_observation[1]
        cmd_input[0][2] = compass_observation[2]
        self.interpreter.set_tensor(self.input_details[1]["index"], cmd_input)
    
        # preprocess (normalize + crop) visual observations:
        target_height = self.input_details[0]["shape"][0] # 90
        target_width = self.input_details[0]["shape"][1] # 160
        preprocessed_rgb_input = np.float32(obs["camera_final_color"])/255.0 # normalization in the [0.0, 1.0] range
        preprocessed_rgb_input = tf.image.crop_to_bounding_box(preprocessed_rgb_input, tf.shape(preprocessed_rgb_input)[0] - target_height, tf.shape(preprocessed_rgb_input)[1] - target_width, target_height, target_width)
        
        # fill image tensor
        self.interpreter.set_tensor(self.input_details[0]["index"], np.expand_dims(preprocessed_rgb_input, axis=0))
            
        # run inference
        start_time = time.time()
        self.interpreter.invoke()
        stop_time = time.time()
        execution_time = (stop_time - start_time) * 1000
        print('Infererence time: {:.3f}ms'.format(execution_time))

        # generate an action from inference result
        result = np.empty((0, 2), dtype=float)
        tflite_output = interpreter.get_tensor(output_details[0]["index"])
        act = np.clip(np.concatenate((result, tflite_output.astype(float))), -1.0, 1.0)
        action = np.array([act[0][0],act[0][1]], dtype=np.float32)
        
        # is goal reached ?
        target_location_reached = (xy_position_error_norm <= self.acceptance_radius)
    
        return action, target_location_reached
    
    
    
    
