import gym
import numpy as np
import os

import spear
from scipy.spatial.transform import Rotation

from examples.common.camera import CameraSensor

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)
from examples.common.agent import SimpleAgent, HabitatNavAgent, SimpleForceAgent, OpenBotAgent, UrdfRobotAgent


def random_position(range=550):
    pos = np.random.rand(3)
    pos += -0.5
    pos *= range * 2
    pos[2] = 50
    return pos.astype(np.float64)


class SpPointNavEnv(gym.Env):
    def __init__(self, env_config):
        spear.log("__init__ Start.")
        self._config = env_config['config']
        self._dummy = env_config["dummy"]
        self._use_camera = env_config['use_camera']
        self._use_random_goal = False
        if env_config['test']:
            self._dummy = True
            self._agent = None

        if self._dummy:
            pass
        else:
            self._config = spear.get_config(
                user_config_files=[
                    os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")),
                    os.path.realpath(os.path.join(common_dir, "default_config.common.yaml"))])

            self._instance = spear.Instance(self._config)

            self._instance.engine_service.begin_tick()
            self._gameplay_statics_class = self._instance.unreal_service.get_static_class(class_name="UGameplayStatics")
            self._gameplay_statics_default_object = self._instance.unreal_service.get_default_object(uclass=self._gameplay_statics_class, create_if_needed=False)
            self._set_game_paused_func = self._instance.unreal_service.find_function_by_name(uclass=self._gameplay_statics_class, name="SetGamePaused")

            self._unreal_actors = self._instance.unreal_service.find_actors_by_type_as_dict(class_name="AActor")
            self._unreal_actors_map = {v: k for k, v in self._unreal_actors.items()}

            self._instance.engine_service.tick()
            self._instance.engine_service.end_tick()

            self._instance.engine_service.begin_tick()

            # spawn agent
            if env_config["agent"] == "simple":
                self._agent = SimpleAgent(self._instance)
            elif env_config["agent"] == "simple_force":
                self._agent = SimpleForceAgent(self._instance)
            elif env_config["agent"] == "habitat":
                self._agent = HabitatNavAgent(self._instance)
            elif env_config["agent"] == "openbot":
                self._agent = OpenBotAgent(self._instance)
            elif env_config["agent"] == "urdf":
                self._agent = UrdfRobotAgent(self._instance)
            else:
                spear.log("Unknown agent: ", env_config["agent"])
                self._instance.engine_service.tick()
                self._instance.engine_service.end_tick()
                self._instance.close()
                exit(-1)

            self._unreal_goal_actor = self._instance.unreal_service.spawn_actor(
                class_name="/Game/Agents/BP_Goal.BP_Goal_C",
                location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
                spawn_parameters={"Name": "Goal", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
            )

            if self._use_camera:
                self._camera_sensor = CameraSensor(self._instance, self._agent._agent, render_pass_names=["final_color", "depth"], width=240, height=320)

            self._instance.engine_service.tick()

            self._instance.engine_service.end_tick()
        # define obs and action
        self.observation_space = gym.spaces.Dict({
            # "location": gym.spaces.Box(-1000, 1000, (3,), np.float64),
            # "rotation": gym.spaces.Box(-1000, 1000, (3,), np.float64),
            "goal_in_agent_frame": gym.spaces.Box(-2000, 2000, (3,), np.float64),
        })
        if self._use_camera:
            self.observation_space["rgb"] = gym.spaces.Box(0, 255, (self._camera_sensor._width, self._camera_sensor._height, 3,), np.uint8)
        if self._agent:
            self.action_space = self._agent.get_action_space()
        else:
            self.action_space = gym.spaces.Dict({
                "set_drive_torque": gym.spaces.Box(0, 1, (2,), np.float64),
                "set_brake_torque": gym.spaces.Box(0, 1, (2,), np.float64)
            })

        # init obs and action
        self._obs = {
            "location": np.array([0, 0, 10]),
            "rotation": np.array([0, 0, 0]),
            "goal_in_agent_frame": np.array([0, 0, 0]),
        }
        if self._use_camera:
            self._obs["rgb"] = np.zeros([self._camera_sensor._width, self._camera_sensor._height, 3])

        # define episode information
        self._goal = random_position(range=500)  # + np.array([-200, -200,0])
        self._step = 0

        spear.log("self.action_space", self.action_space)
        spear.log("self.observation_space", self.observation_space)
        spear.log("__init__ Done.")

    def reset(self):
        self._goal = random_position(range=500)  # + np.array([-200, -200,0])
        if self._dummy:
            new_location = random_position()
            new_rotation = np.array([0, 0, 0])
        else:
            self._instance.engine_service.begin_tick()
            self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": False})

            self._agent.reset()

            self._instance.engine_service.tick()

            agent_obs = self._agent.get_observation()
            new_location = agent_obs['location']
            new_rotation = agent_obs['rotation']

            if self._use_random_goal:
                position = self._agent.get_random_points(1)[0]
                self._goal = np.array([position['x'], position['y'], position['z'] + self._agent._z_offset])
            else:
                self._goal = np.array([0, 0, 0 + self._agent._z_offset])
            if self._unreal_goal_actor:
                self._instance.unreal_service.call_function(self._unreal_goal_actor, self._agent._unreal_set_actor_location_and_rotation_func, {
                    "NewLocation": dict(zip(["X", "Y", "Z"], self._goal.tolist())),
                    "NewRotation": {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
                    "bSweep": False,
                    "bTeleport": True
                })

            if self._use_camera:
                self._obs["rgb"] = self._camera_sensor.get_images()

            self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": True})
            self._instance.engine_service.end_tick()

        quat = Rotation.from_euler("xyz", new_rotation, degrees=True)
        new_rotation = np.array(quat.as_euler("xyz", degrees=True))

        self._obs['location'] = new_location
        self._obs['rotation'] = new_rotation
        self._obs["goal_in_agent_frame"] = quat.inv().as_matrix().dot(self._goal - self._obs['location'])

        spear.log("reset Done.", self._step)

        self._step = 0

        obs = {}
        for key, value in self._obs.items():
            if key in self.observation_space.spaces:
                obs[key] = value
        return obs

    def step(self, action):
        if self._dummy:
            new_rotation = self._obs['rotation'] + np.array([0, 0, action['move_right'][0]])
            quat = Rotation.from_euler("xyz", new_rotation, degrees=True)
            new_rotation = np.array(quat.as_euler("xyz", degrees=True))

            new_location = self._obs['location'] + action['move_forward'][0] * quat.as_matrix().dot(np.array([1, 0, 0]))

            collision = abs(new_location[0]) > 550 or abs(new_location[1]) > 550
        else:
            self._instance.engine_service.begin_tick()
            self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": False})

            old_obs = self._agent.get_observation()

            self._agent.apply_action(action)

            self._instance.engine_service.tick()

            hit_actors = self._agent.get_hit_actors()
            obs = self._agent.get_observation()
            new_location = obs['location']
            new_rotation = obs['rotation']
            quat = Rotation.from_euler("xyz", new_rotation, degrees=True)
            new_rotation = np.array(quat.as_euler("xyz", degrees=True))

            if self._use_camera:
                self._obs["rgb"] = self._camera_sensor.get_images()

            # diff = np.array([action['add_to_location'][0], action['add_to_location'][1], 0]) + old_obs['location'] - new_location
            # if np.linalg.norm(diff) > 10:
            #     print("DIFF?", diff, action, obs['location'], new_location)
            self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": True})
            self._instance.engine_service.end_tick()

            collision = abs(new_location[0]) > 1000 or abs(new_location[1]) > 1000 or abs(new_location[1]) > 1000
            collision |= abs(new_rotation[0]) > 30 or abs(new_rotation[1]) > 30
            if collision:
                spear.log("hit boundary", new_location, new_rotation)
            if len(hit_actors) > 0:
                hit_actor_names = []
                for hit_actor in hit_actors:
                    if hit_actor in self._unreal_actors_map:
                        hit_actor_name = self._unreal_actors_map[hit_actor]
                        hit_actor_name_list = hit_actor_name.split("/")
                        if hit_actor_name_list[1] == "02_floor":
                            pass
                        else:
                            hit_actor_names.append(hit_actor_name)
                if len(hit_actor_names) > 0:
                    spear.log("hit!", len(hit_actor_names), hit_actor_names)
                    collision = True
            new_location = np.clip(new_location, -1000, 1000)

        self._obs['location'] = new_location
        self._obs['rotation'] = new_rotation
        self._obs["goal_in_agent_frame"] = quat.inv().as_matrix().dot(self._goal - self._obs['location'])

        distance = np.linalg.norm(self._goal - self._obs['location'])
        succ = distance < 50
        timeout = self._step > 500
        reward = -distance * 0.001 + (100 if succ else 0) + (-10 if collision else 0)
        done = succ or collision or timeout

        info = {}

        if done:
            spear.log("SUCC " if succ else "COLL ", self._step, self._goal, self._obs['location'], self._obs['goal_in_agent_frame'], action, reward, done)
        elif self._step % 100 == 0 or done:
            # spear.log("step ", self._step, self._goal, self._obs['location'], self._obs['goal_in_agent_frame'], action, reward, done)
            pass
        self._step += 1

        obs = {}
        for key, value in self._obs.items():
            if key in self.observation_space.spaces:
                obs[key] = value
        return obs, reward, done, info

    def close(self):
        if self._dummy:
            pass
        else:
            # close the unreal instance and rpc connection
            self._instance.close()

        spear.log("close Done.")
