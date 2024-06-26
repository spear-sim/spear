import json

import gym
import numpy as np

import spear
from scipy.spatial.transform import Rotation


class AgentBase():
    def __init__(self, instance):
        self._instance = instance
        self._agent = None

        unreal_actor_static_class = self._instance.unreal_service.get_static_class("AActor")
        self._unreal_set_actor_location_and_rotation_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_SetActorLocationAndRotation")
        self._unreal_get_actor_location_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_GetActorLocation")
        self._unreal_get_actor_rotation_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_GetActorRotation")
        unreal_static_mesh_static_class = self._instance.unreal_service.get_static_class("UStaticMeshComponent")
        self._unreal_add_force_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_static_mesh_static_class, name="AddForce")

        self._hit_event_class = self._instance.unreal_service.get_static_class_v2("/Script/CoreUObject.Class'/Script/SpComponents.SpHitEventActor'")
        self._hit_event_actor = self._instance.unreal_service.spawn_actor(
            class_name="/Script/CoreUObject.Class'/Script/SpComponents.SpHitEventActor'",
            location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "SpHitEventActor"}
        )
        self._subscribe_actor_func = self._instance.unreal_service.find_function_by_name(
            uclass=self._hit_event_class, name="SubscribeToActor")
        self._get_hit_event_desc_func = self._instance.unreal_service.find_function_by_name(
            uclass=self._hit_event_class, name="GetHitEventDescs")

        self._nav_mesh_actor_class = self._instance.unreal_service.get_static_class_v2("/Script/CoreUObject.Class'/Script/SpComponents.SpNavMeshActor'")
        self._nav_mesh_actor = self._instance.unreal_service.spawn_actor(
            class_name="/Script/CoreUObject.Class'/Script/SpComponents.SpNavMeshActor'",
            location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "SpNavMeshActor"}
        )
        self._nav_mesh_setup_func = self._instance.unreal_service.find_function_by_name(uclass=self._nav_mesh_actor_class, name="setup")
        self._get_point_func = self._instance.unreal_service.find_function_by_name(uclass=self._nav_mesh_actor_class, name="getRandomPoints")
        self._get_path_func = self._instance.unreal_service.find_function_by_name(uclass=self._nav_mesh_actor_class, name="getPaths")

    def get_observation_space(self):
        return gym.spaces.Dict({
            "location": gym.spaces.Box(-1000, 1000, (3,), np.float64),
            "rotation": gym.spaces.Box(-1000, 1000, (3,), np.float64),
        })

    def get_observation(self):
        # get observation
        current_location = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_location_func)['ReturnValue']
        current_rotation = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_rotation_func)['ReturnValue']
        current_location = np.array([current_location['x'], current_location['y'], current_location['z']])
        current_rotation = np.array([current_rotation['roll'], current_rotation['pitch'], current_rotation['yaw']])
        self._obs = {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.float64),
            "location": current_location,
            "rotation": current_rotation,
        }
        return self._obs

    def get_action_space(self):
        assert False

    def apply_action(self, action):
        assert False

    def reset(self):
        assert False

    def get_hit_actors(self):
        hit_events = self._instance.unreal_service.call_function(uobject=self._hit_event_actor, ufunction=self._get_hit_event_desc_func)['ReturnValue']
        # print("hit_events", hit_events)
        hit_actors = set()
        if len(hit_events) > 0:
            for event in hit_events:
                hit_actors.add(event['otherActor'])
        # for hit_actor in hit_actors:
        #     hit_actor_name = None
        #     if hit_actor not in self._actor_maps:
        #         hit_actor_name = self._instance.unreal_service.get_stable_name_for_component(hit_actor, include_actor_name=True)
        #         self._actor_maps[hit_actor] = {"actor_name", hit_actor_name}
        #     else:
        #         hit_actor_name = self._actor_maps[hit_actor]
        #     print("    hit_actor_name", hit_actor_name)

        return hit_actors

    def get_random_points(self, num_points):
        points = self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._get_point_func, args={"num_points": num_points})['ReturnValue']
        return points


class SimpleAgent(AgentBase):
    def __init__(self, instance):
        super().__init__(instance)

        self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._nav_mesh_setup_func, args={
            "agent_height": 100.0, "agent_radius": 100.0
        })
        new_location = self.get_random_points(1)[0]

        new_location['z'] += 50  # add distance between agent center and z_min
        self._agent = self._instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_SimpleAgentPawn.BP_SimpleAgentPawn_C",
            location=new_location, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        spear.log("agent = ", self._agent)
        if self._agent == 0:
            spear.log("spawn agent failed!")
        self._root_component = self._instance.unreal_service.get_component_by_name("UStaticMeshComponent", self._agent, "StaticMeshComponent")

        spear.log("root_component = ", self._root_component)

        self._instance.unreal_service.call_function(uobject=self._hit_event_actor, ufunction=self._subscribe_actor_func, args={
            "Actor": self._instance.unreal_service.to_ptr(self._agent),
        })

    def get_action_space(self):
        return gym.spaces.Dict({
            "add_to_location": gym.spaces.Box(-10, 10, (3,), np.float64),
            # "add_to_rotation": gym.spaces.Box(-1, 1, (3,), np.float64),
        })

    def apply_action(self, action):
        new_location = self._obs['location'] + action['add_to_location']

        new_rotation = self._obs['rotation'].astype(np.float64)
        if "add_to_rotation" in action:
            new_rotation += action['add_to_rotation'].astype(np.float64)

        transform_args = {
            "NewLocation": dict(zip(["X", "Y", "Z"], new_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)

    def reset(self):
        new_location = self.get_random_points(1)[0]
        new_location['z'] += 10
        new_rotation = np.array([0.0, 0.0, 0.0])
        transform_args = {
            "NewLocation": new_location,
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)
        return {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.float64),
            "location": new_location,
            "rotation": new_rotation,
        }


class SimpleForceAgent(SimpleAgent):
    def get_action_space(self):
        return gym.spaces.Dict({
            "add_force": gym.spaces.Box(-1, 1, (3,), np.float64),
            # "add_to_rotation": gym.spaces.Box(-1, 1, (3,), np.float64),
        })

    def apply_action(self, action):
        scale = 1
        force = action['add_force'] * scale
        add_force_args = {
            "Force": dict(zip(["X", "Y", "Z"], force.tolist()))
        }
        self._instance.unreal_service.call_function(self._root_component, self._unreal_add_force_func, add_force_args)


class HabitatNavAgent(AgentBase):

    def __init__(self, instance):
        super().__init__(instance)

        self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._nav_mesh_setup_func, args={
            "agent_height": 100.0, "agent_radius": 100.0
        })
        new_location = self.get_random_points(1)[0]

        new_location['z'] = 50  # add distance between agent center and z_min
        self._agent = self._instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_SimpleAgentPawn.BP_SimpleAgentPawn_C",
            location=new_location, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        spear.log("agent = ", self._agent)
        if self._agent == 0:
            spear.log("spawn agent failed!")
        self._root_component = self._instance.unreal_service.get_component_by_name("UStaticMeshComponent", self._agent, "StaticMeshComponent")

        spear.log("root_component = ", self._root_component)

        self._instance.unreal_service.call_function(uobject=self._hit_event_actor, ufunction=self._subscribe_actor_func, args={
            "Actor": self._instance.unreal_service.to_ptr(self._agent),
        })

    def get_action_space(self):
        return gym.spaces.Dict({
            "move_foward": gym.spaces.Box(-10, 10, (1,), np.float64),
            "move_left": gym.spaces.Box(-180, 180, (1,), np.float64),
            "move_right": gym.spaces.Box(-180, 180, (1,), np.float64),
            # "stop": gym.spaces.Box(0, 1, (1,), np.int),
        })

    def apply_action(self, action):
        add_rotation = action['move_right'][0] - action['move_left'][0]
        new_rotation = self._obs['rotation'] + np.array([0, 0, add_rotation])

        quat = Rotation.from_euler("xyz", new_rotation, degrees=True)
        direction = quat.as_matrix() * np.array([1, 0, 0])
        new_location = self._obs['location'] + action['move_foward'] * direction[:, 0]

        transform_args = {
            "NewLocation": dict(zip(["X", "Y", "Z"], new_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)

    def reset(self):
        new_location = self.get_random_points(1)[0]
        new_location['z'] += 10
        new_rotation = np.array([0, 0, 0])
        transform_args = {
            "NewLocation": new_location,
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)
        return {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.float64),
            "location": new_location,
            "rotation": new_rotation,
        }


class OpenBotAgent(AgentBase):
    def __init__(self, instance):
        super().__init__(instance)

        self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._nav_mesh_setup_func, args={
            "agent_height": 20.0, "agent_radius": 20.0
        })
        init_position = self.get_random_points(1)[0]
        init_position['z'] += 3  # add distance between agent center and z_min
        self._agent = self._instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_OpenBotPawn.BP_OpenBotPawn_C",
            location=init_position, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        spear.log("agent = ", self._agent)
        if self._agent == 0:
            spear.log("spawn agent failed! ")
            self._agent = self._instance.unreal_service.spawn_actor(
                class_name="/Game/Agents/BP_OpenBotPawn.BP_OpenBotPawn_C",
                location={"X": 0.0, "Y": 0.0, "Z": 10.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
                spawn_parameters={"Name": "Agent1", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
            )

        # get vehicle component
        self._chaos_vehicle_movement_component = instance.unreal_service.get_component_by_type_v2(
            "/Script/CoreUObject.Class'/Script/ChaosVehicles.ChaosWheeledVehicleMovementComponent'", self._agent)
        self._chaos_vehicle_movement_component_class = instance.unreal_service.get_class(self._chaos_vehicle_movement_component)

        self._set_drive_torque_func = self._instance.unreal_service.find_function_by_name(uclass=self._chaos_vehicle_movement_component_class, name="SetDriveTorque")
        self._set_brake_torque_func = self._instance.unreal_service.find_function_by_name(uclass=self._chaos_vehicle_movement_component_class, name="SetBrakeTorque")

        self._instance.unreal_service.call_function(uobject=self._hit_event_actor, ufunction=self._subscribe_actor_func, args={
            "Actor": self._instance.unreal_service.to_ptr(self._agent),
        })

    def get_action_space(self):
        return gym.spaces.Dict({
            "set_drive_torque": gym.spaces.Box(-1000, 1000, (2,), np.float64),
            "set_brake_torque": gym.spaces.Box(-1000, 1000, (2,), np.float64)
        })

    def apply_action(self, action):
        set_drive_torque = action['set_drive_torque']
        set_brake_torque = action['set_brake_torque']
        for wheel_index in range(0, 4):
            set_drive_torque_args = {
                "DriveTorque": float(set_drive_torque[wheel_index]),
                "WheelIndex": wheel_index
            }
            self._instance.unreal_service.call_function(self._chaos_vehicle_movement_component, self._set_drive_torque_func, set_drive_torque_args)
            set_brake_torque_args = {
                "BrakeTorque": float(set_brake_torque[wheel_index]),
                "WheelIndex": wheel_index
            }
            self._instance.unreal_service.call_function(self._chaos_vehicle_movement_component, self._set_brake_torque_func, set_brake_torque_args)

    def reset(self):
        new_location = self.get_random_points(1)[0]
        new_location['z'] += 3
        new_rotation = np.array([0, 0, 0])

        transform_args = {
            "NewLocation": new_location,
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)
        return {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.float64),
            "location": new_location,
            "rotation": new_rotation,
        }
