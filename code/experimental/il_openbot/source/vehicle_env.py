import math
import cv2
import os
import re
import csv
import shutil
import numpy as np
import abc

from interiorsim import Env
from PIL import Image

DIR_PATH = os.path.dirname(os.path.realpath(__file__))
SPEED_MULTIPLIER = 1


class VehicleEnv(Env, abc.ABC):
    # VehicleEnv is an abstract class.
    def __init__(self,
                 config,
                 log_obs_dim,
                 dataset_path=None,
                 num_steps=1000,
                 speed_multiplier=None,
                 debug_mode=False,
                 create_video=True):
        self.log_obs_dim = log_obs_dim
        self.create_video = create_video
        self.debug_mode = debug_mode
        self.dataset_path = dataset_path if dataset_path is not None else DIR_PATH
        self.num_steps = num_steps  # amount of steps to take during perform_run()
        # Filled everytime perform_run in behaviour_policy is called
        self.array_obs = None
        self.datafolder_name = None
        self.folder_name = None
        self.executed_steps = None

        # Actual pose and desired position are set whenver self.step() or
        # self.reset() is called
        self.actual_pose_yaw_xy = None
        self.desired_position_xy = None

        self.index_waypoint = 1
        self.num_waypoints = None  # set in self.reset()

        self._env_data = {}  # set inside step()

        # Config stuff
        if speed_multiplier is None:
            self.speed_multiplier = SPEED_MULTIPLIER
        else:
            self.speed_multiplier = speed_multiplier

        super().__init__(config)

    def _show_obs_and_wait_for_key(self):
        obs = self.get_observation()
        # OpenCV expects BGR instead of RGB
        print(f"State data: xyz [{obs['state_data'][0]:.2f},{obs['state_data'][1]:.2f},{obs['state_data'][2]:.2f}], roll pitch yaw [{obs['state_data'][3]:.2f}, {obs['state_data'][4]:.2f}, {obs['state_data'][5]:.2f}]")
        cv2.imshow("visual_observation",
                   obs["visual_observation"][:, :, [2, 1, 0]])
        cv2.waitKey(100)

    def _get_actual_pose_yaw_xy(self):
        obs = self.get_observation()
        actual_pose_yaw_xy = np.array([obs["state_data"][5],
                                       obs["state_data"][0],
                                       obs["state_data"][1]])
        return actual_pose_yaw_xy

    def _get_desired_position_xy(self, index_waypoint=None):
        info = self.get_step_info()
        traj_data = info["agent_controller_step_info"]["trajectory_data"]

        if index_waypoint is None:
            index_waypoint = traj_data.shape[0]//3 - 1
        # [Xdes, Ydes]
        desired_position_xy = np.array([traj_data[3*index_waypoint],
                                        traj_data[3*index_waypoint + 1]])
        return desired_position_xy

    def _get_velocities(self):
        obs = self.get_observation()

        # XY position of the next waypoint in world frame:
        d_xy = np.array(
            [obs["state_data"][0],
                obs["state_data"][1]]) - self.desired_position_xy

        # Current position and heading of the vehicle in world frame:
        d_yaw = obs["state_data"][5] - self.actual_pose_yaw_xy[0]

        # Numerical diff:
        dt = self._config["SIMULATION_CONTROLLER"]["SIMULATION_STEP_TIME_SECONDS"]
        time_constant = dt*10
        lin_vel_norm = np.linalg.norm(d_xy/time_constant)
        lin_vel = lin_vel_norm * 0.036  # In [m/s]

        yaw_vel = d_yaw/time_constant

        return lin_vel, yaw_vel

    def _goal_reached(self):
        goal_reached = self.index_waypoint == self.num_waypoints
        if goal_reached:
            print("(num_waypoints == index_waypoint).")
        else:
            # if the waypoint is not the goal
            print(f"Waypoint {self.index_waypoint}" +
                  f"/{self.num_waypoints} reached.")
            self.index_waypoint = self.index_waypoint + 1

        return goal_reached

    def get_observation(self):
        # not calling self._get_observation as it performs a client call (slower)
        return self._env_data["obs"]

    def get_reward(self):
        # not calling self._get_reward as it performs a client call (slower)
        return self._env_data["reward"]

    def get_step_info(self):
        # not calling self._get_step_info as it performs a client call (slower)
        return self._env_data["info"]

    def step(self, action):
        obs, reward, is_done, step_info = super().step(action)

        self._env_data.update(
            {"obs": obs,
             "reward": reward,
             "done": is_done,
             "info": step_info}
        )
        return obs, reward, is_done, step_info

    def reset(self):
        obs = super().reset()
        self.index_waypoint = 1
        # the following part sends a step with no action to initialise desired
        # position and actual pose.

        # Send Zero action to the agent and collect initial trajectory observations:
        # This is also necessary to initialise self._env_data
        _, _, _, info = self.step({"apply_voltage": [0.0, 0.0]})

        # Setting num_waypoints to detect whether goal reached,
        # i.e., index_waypoint == num_waypoints
        traj_data = info["agent_controller_step_info"]["trajectory_data"]
        self.num_waypoints = len(traj_data)/3 - 1

        self.actual_pose_yaw_xy = self._get_actual_pose_yaw_xy()
        self.desired_position_xy = self._get_desired_position_xy(
            index_waypoint=self.index_waypoint)

        return obs

    def _set_and_make_folders(self, map_name, run):
        self.folder_name = f"{self.dataset_path}/dataset/uploaded/run_{map_name}_{run}"

        self.datafolder_name = self.folder_name+"/data/"
        os.makedirs(self.datafolder_name, exist_ok=True)
        os.makedirs(self.datafolder_name+"sensor_data", exist_ok=True)
        os.makedirs(self.datafolder_name+"images", exist_ok=True)

    def _remove_folder(self):
        shutil.rmtree(self.folder_name)

    def _save_image(self, obs, step_count):
        img = Image.fromarray(obs["visual_observation"])
        img.save(self.datafolder_name+"images/%d.jpeg" % step_count)

    def _generate_video(self, map_name, run):

        print("Generating video from the sequence of observations")
        image_folder = f"{self.dataset_path}/dataset/uploaded/run_{map_name}_{run}/data/images"
        video_name = f"{self.dataset_path}/videos/run_{map_name}_{run}.avi"

        if not (os.path.exists(f"{self.dataset_path}/videos")):
            os.makedirs(f"{self.dataset_path}/videos")

        images = [img for img in os.listdir(image_folder)]
        frame = cv2.imread(os.path.join(image_folder, images[0]))
        height, width, _ = frame.shape

        # rate = int(
        #     1/self.config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS)

        video = cv2.VideoWriter(video_name, 0, 100, (width, height))

        # good initial sort but doesnt sort numerically very well
        images.sort(key=lambda f: int(re.sub('\D', '', f)))

        for image in images:
            video.write(cv2.imread(os.path.join(image_folder, image)))

        cv2.destroyAllWindows()
        video.release()

    def _clamp(self, value):
        control_saturation = self._config.ROBOT_SIM.CONTROL_SATURATION
        smallest = -control_saturation
        largest = control_saturation
        return max(smallest, min(value, largest))

    def _get_vector_to_goal_xy(self, get_final_goal=False):
        # Target error vector (global coordinate system):
        actual_position_xy = np.array([self.actual_pose_yaw_xy[1],
                                       self.actual_pose_yaw_xy[2]])

        if get_final_goal:
            # with index_waypoint = None, the function returns the final goal
            desired_position_xy = self._get_desired_position_xy(
                index_waypoint=None)
        else:
            desired_position_xy = self.desired_position_xy

        relative_position_to_target = desired_position_xy - actual_position_xy
        return relative_position_to_target

    def _get_dist_to_goal_xy(self, get_final_goal=False):
        vector_to_goal_xy = self._get_vector_to_goal_xy(get_final_goal)
        # Compute Euclidean distance to target in [m]:
        dist = np.linalg.norm(vector_to_goal_xy)*0.01
        return dist

    def _target_waypoint_reached(self):
        config = self._config.SIMULATION_CONTROLLER
        acceptance_radius = config.IMITATION_LEARNING_TASK.ACCEPTANCE_RADIUS
        dist = self._get_dist_to_goal_xy()
        return dist < acceptance_radius

    @staticmethod
    def _clamp_to_pi(angle):
        # Fit to range [-pi, pi]:
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle <= -math.pi:
            angle += 2 * math.pi
        return angle

    def _get_actual_forward_angle(self, yaw_vehicle=None):
        # Compute robot forward axis (global coordinate system):
        forward = np.array([1, 0])  # Front axis is the X axis.

        if yaw_vehicle is None:
            yaw_vehicle = self.actual_pose_yaw_xy[0]

        rot = np.array([[math.cos(yaw_vehicle), -math.sin(yaw_vehicle)],
                        [math.sin(yaw_vehicle), math.cos(yaw_vehicle)]])

        forward_rotated = np.dot(rot, forward)

        # Compute yaw:
        forward_angle = math.atan2(forward_rotated[1],
                                   forward_rotated[0])
        return forward_angle

    def _get_desired_forward_angle(self, vector_to_goal_xy=None):
        if vector_to_goal_xy is None:
            vector_to_goal_xy = self._get_vector_to_goal_xy()

        relative_angle = math.atan2(vector_to_goal_xy[1],
                                    vector_to_goal_xy[0])
        return relative_angle

    def _get_delta_yaw(self, actual_forward_angle=None,
                       desired_forward_angle=None):
        if actual_forward_angle is None:
            actual_forward_angle = self._get_actual_forward_angle()

        if desired_forward_angle is None:
            desired_forward_angle = self._get_desired_forward_angle()

        delta_yaw = actual_forward_angle - desired_forward_angle
        delta_yaw = self._clamp_to_pi(delta_yaw)
        return delta_yaw

    def perform_run(self, map_name, run_no):

        failure_run = False

        # Reset env and make folder according to this run
        self._reset_array_obs()
        self._set_and_make_folders(map_name, run_no)

        # Reset the simulation to get the first observation
        obs = self.reset()

        # Take a few steps:
        for step_count in range(self.num_steps):
            print(f"Step {step_count}/{self.num_steps}")

            action = self._get_action()

            # Send action to the agent and collect observations:
            obs, _, done, info = self.step(
                {"apply_voltage": [action[0], action[1]]})

            if self.debug_mode:
                # OpenCV expects BGR instead of RGB
                self._show_obs_and_wait_for_key()

            self._fill_array_obs(step_count)

            # For now we don't consider underground operation
            if self.array_obs[step_count][4] < -0.1:
                failure_run = True
                print(f"Underground z: {self.array_obs[step_count][4]}")
                break

            # Save the images:
            self._save_image(obs, step_count)

            if self._target_waypoint_reached():
                if step_count < 50:
                    # There is some issue related to initial collision, where
                    # the car flies around (start in collision status with
                    # large force)
                    failure_run = True
                    print("Reached too early")
                    break  # re-execute the run

                if self._goal_reached():
                    # Goal reached !
                    print("Goal reached")
                    self._write_status_file(status="Goal",
                                            iterations=step_count)
                    break

            # Interrupt the step loop if the done flag is raised:
            if done:
                if step_count < 50:
                    # There is some issue related to initial collision, where
                    # the car flies around (start in collision status with
                    # large force)
                    failure_run = True
                    print("Done and early reached")
                    break  # re-execute the run

                if info["task_step_info"]["hit_obstacle"]:
                    print("Collision detected ! Killing simulation and" +
                          " restarting run.")
                    failure_run = True
                    self._write_status_file(status="Collision",
                                            iterations=step_count)

                if info["task_step_info"]["hit_goal"]:
                    self._write_status_file(status="Goal",
                                            iterations=step_count)
                    print("Goal reached (from info).")
                break

            # Updated the actual position and desired pose after
            # index_waypoint was incremented
            self.actual_pose_yaw_xy = self._get_actual_pose_yaw_xy()
            self.desired_position_xy = self._get_desired_position_xy(
                index_waypoint=self.index_waypoint)

        # Once run finished either remove the folder when collided or save data
        if failure_run:
            print("Restarting run.")
            self._remove_folder()
            return 0
        else:
            if self.create_video:
                self._generate_video(map_name, run_no)
            executed_steps = step_count + 1
            self._fill_database(executed_steps)
            return 1

    def _reset_array_obs(self):
        self.array_obs = np.empty([self.num_steps, self.log_obs_dim])

    def _write_status_file(self, status, iterations):
        with open(self.datafolder_name + "sensor_data/ctrlLog.txt", 'w',
                  encoding="utf-8") as f_status:
            writer_status = csv.writer(f_status, delimiter=",")
            writer_status.writerow(('Status', 'Iterations'))
            writer_status.writerow((status, iterations))

    @abc.abstractclassmethod
    def _get_action(self):
        raise NotImplementedError("get_action")

    @abc.abstractclassmethod
    def _fill_database(self, executed_steps):
        raise NotImplementedError("fill_database")

    @abc.abstractclassmethod
    def _fill_array_obs(self, step_count):
        raise NotImplementedError("fill_array_obs")
