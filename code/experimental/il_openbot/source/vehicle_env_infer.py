import math
import csv
import datetime
import numpy as np
import tensorflow.lite as tflite
import tensorflow as tf

from vehicle_env import VehicleEnv


class VehicleEnv_Infer(VehicleEnv):
    def __init__(self,
                 policy_path=None,
                 **kwargs):
        assert policy_path is not None, "No policy path specified for infer."
        self.policy_path = policy_path

        super().__init__(log_obs_dim=15,
                         **kwargs)

        # Internal variables saved during _get_action()
        self.action = None
        self.cmd = None

        # Inference stuff
        # Following 2 details set in initialise_policy()
        self.interpreter = None  # The tflite model that's queried for predictions
        self.input_details = None
        self.output_details = None

        self.cmd_input = None  # set in self.init_policy()
        self._initialise_policy()

    def _initialise_policy(self):
        self.interpreter = tflite.Interpreter(self.policy_path)
        self.interpreter.allocate_tensors()

        # Get input and output tensors.
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def _get_action(self):
        obs = self.get_observation()

        # Process (crop) visual observations:
        if not obs["visual_observation"].any():
            raise ValueError("Visual observation is empty")
            # Something went wrong and the robot was spawed out of the map...

        img_input = self._get_image_input()
        cmd_input = self._get_cmd_input()
        if self.debug_mode:
            print(
                f"Cmd - dist: {cmd_input[0,0]:.3f}, angle: [{cmd_input[0,1]:.3f}, {cmd_input[0,2]:.3f}]")
        output = self._predict_model(img_input, cmd_input)
        self.action = self._clip_action(output)
        return self.action

    def _clip_action(self, action):

        result = np.empty((0, 2), dtype=float)
        act = np.clip(np.concatenate(
            (result, action.astype(float))), -1.0, 1.0)
        action = np.array([act[0][0], act[0][1]])
        return action

    def _predict_model(self, img_input, cmd_input):
        # Inference:
        self.interpreter.set_tensor(self.input_details[0]["index"],
                                    np.expand_dims(img_input, axis=0))
        self.interpreter.set_tensor(self.input_details[1]["index"],
                                    cmd_input)

        self.interpreter.invoke()

        # Output of the Artificial Neural Network
        output = self.interpreter.get_tensor(self.output_details[0]["index"])
        return output

    def _get_image_input(self):
        obs = self.get_observation()

        # img_input = np.array(np.random.random_sample(
        #     input_details[0]["shape"]), dtype=np.float32)

        img_input = np.float32(obs["visual_observation"])/255
        img_input = tf.image.crop_to_bounding_box(img_input, tf.shape(
            img_input)[0] - 90, tf.shape(img_input)[1] - 160, 90, 160)
        return img_input

    def _get_cmd_input(self):
        # here desired_position_xy directly refers to the goal position
        # rather than any waypoint...
        self.cmd = self._compute_neural_net_input()

        cmd_input = np.array(np.random.random_sample(
            self.input_details[1]["shape"]), dtype=np.float32)
        cmd_input[0][0] = np.float32(self.cmd[0])
        cmd_input[0][1] = np.float32(self.cmd[1])
        cmd_input[0][2] = np.float32(self.cmd[2])
        return cmd_input

    def _compute_neural_net_input(self):
        if self._target_waypoint_reached():
            # If reached target location, do nothing
            obs_net = np.array([0.0, 0.0, 1.0])
            return obs_net

        delta_yaw = self._get_delta_yaw()

        sin_yaw = math.sin(delta_yaw)
        cos_yaw = math.cos(delta_yaw)

        dist = self._get_dist_to_goal_xy(get_final_goal=True)

        obs_net = np.array([dist, sin_yaw, cos_yaw], dtype=np.float32)

        return obs_net

    def _fill_database(self, executed_steps):
        print("Filling database...")
        f_infer = open(self.datafolder_name + "sensor_data/Inference.txt", 'w',
                       encoding="utf-8")
        writer_infer = csv.writer(f_infer, delimiter=",")
        writer_infer.writerow(('timestamp',
                               'pos_x', 'pos_y', 'pos_z',
                               'roll', 'pitch', 'yaw',
                               'goal_x', 'goal_y',
                               'dist', 'sin_yaw', 'cos_yaw',
                               'ctrl_left', 'ctrl_right',
                               'reward'))
        for i in range(executed_steps):
            # Write pose data
            writer_infer.writerow((int(self.array_obs[i][14]),
                                  self.array_obs[i][0],
                                  self.array_obs[i][1],
                                  self.array_obs[i][2],
                                  self.array_obs[i][3],
                                  self.array_obs[i][4],
                                  self.array_obs[i][5],
                                  self.array_obs[i][6],
                                  self.array_obs[i][7],
                                  self.array_obs[i][8],
                                  self.array_obs[i][9],
                                  self.array_obs[i][10],
                                  self.array_obs[i][11],
                                  self.array_obs[i][12],
                                  self.array_obs[i][13],
                                   ))
        f_infer.close()

    def _fill_array_obs(self, step_count):
        obs = self.get_observation()

        # agent pos X wrt. world
        self.array_obs[step_count][0] = obs["state_data"][0]
        # agent pos Y wrt. world
        self.array_obs[step_count][1] = obs["state_data"][1]
        # agent pos Z wrt. world
        self.array_obs[step_count][2] = obs["state_data"][2]
        # agent Roll wrt. world
        self.array_obs[step_count][3] = obs["state_data"][3]
        # agent Pitch wrt. world
        self.array_obs[step_count][4] = obs["state_data"][4]
        # agent Yaw wrt. world
        self.array_obs[step_count][5] = obs["state_data"][5]

        desired_position_xy = self._get_desired_position_xy()
        # desired (waypoint) agent pos X wrt. world
        self.array_obs[step_count][6] = desired_position_xy[0]
        # desired (waypoint) agent pos Y wrt. world
        self.array_obs[step_count][7] = desired_position_xy[1]

        # cmd_input for neural net
        self.array_obs[step_count][8] = self.cmd[0]
        self.array_obs[step_count][9] = self.cmd[1]
        self.array_obs[step_count][10] = self.cmd[2]

        # Action left and right for step
        self.array_obs[step_count][11] = self.action[0]
        self.array_obs[step_count][12] = self.action[1]

        # reward
        self.array_obs[step_count][13] = self.get_reward()

        timestamp = 10000*datetime.datetime.now().timestamp()
        self.array_obs[step_count][14] = timestamp

    def _goal_reached(self):
        return True # the waypoint is the goal for infer (contrary to data collection)