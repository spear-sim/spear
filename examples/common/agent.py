import random

import gym
import numpy as np

import spear
from scipy.spatial.transform import Rotation


class AgentBase():
    def __init__(self, instance):
        self._instance = instance
        self._agent = None

        unreal_actor_static_class = self._instance.unreal_service.get_static_class("AActor")
        self._unreal_set_actor_location_and_rotation_func = self._instance.unreal_service.find_function_by_name(uclass=unreal_actor_static_class,
                                                                                                                name="K2_SetActorLocationAndRotation")
        self._unreal_get_actor_location_func = self._instance.unreal_service.find_function_by_name(uclass=unreal_actor_static_class, name="K2_GetActorLocation")
        self._unreal_get_actor_rotation_func = self._instance.unreal_service.find_function_by_name(uclass=unreal_actor_static_class, name="K2_GetActorRotation")
        self._unreal_static_mesh_static_class = self._instance.unreal_service.get_static_class("UStaticMeshComponent")
        self._unreal_add_force_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_static_mesh_static_class, name="AddForce")
        self._unreal_add_torque_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_static_mesh_static_class, name="AddTorqueInDegrees")

        self._hit_event_class = self._instance.unreal_service.get_static_class_v2("/Script/CoreUObject.Class'/Script/SpComponents.SpHitEventActor'")
        self._hit_event_actor = self._instance.unreal_service.spawn_actor(
            class_name="/Script/CoreUObject.Class'/Script/SpComponents.SpHitEventActor'",
            location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "SpHitEventActor"}
        )
        self._subscribe_actor_func = self._instance.unreal_service.find_function_by_name(uclass=self._hit_event_class, name="SubscribeToActor")
        self._get_hit_event_desc_func = self._instance.unreal_service.find_function_by_name(uclass=self._hit_event_class, name="GetHitEventDescs")

        self._nav_mesh_actor_class = self._instance.unreal_service.get_static_class_v2("/Script/CoreUObject.Class'/Script/SpComponents.SpNavMeshActor'")
        self._nav_mesh_actor = self._instance.unreal_service.spawn_actor(
            class_name="/Script/CoreUObject.Class'/Script/SpComponents.SpNavMeshActor'",
            location={"X": 0.0, "Y": 0.0, "Z": 0.0},
            rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "SpNavMeshActor"})
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
        hit_actors = set()
        if len(hit_events) > 0:
            for event in hit_events:
                hit_actors.add(event['otherActor'])
        return hit_actors

    def get_random_points(self, num_points):
        points = self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._get_point_func, args={"num_points": num_points})['ReturnValue']
        return points


class SimpleAgent(AgentBase):
    def __init__(self, instance):
        super().__init__(instance)
        self._z_offset = 10

        self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._nav_mesh_setup_func, args={
            "agent_height": 100.0, "agent_radius": 40.0
        })
        new_location = self.get_random_points(1)[0]

        new_location['z'] += self._z_offset  # add z offset between agent center and z_min
        self._agent = self._instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_SimpleAgentPawn.BP_SimpleAgentPawn_C",
            location=new_location, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        spear.log("agent = ", self._agent)
        if self._agent == 0:
            spear.log("spawn agent failed!")
        self._root_component = self._instance.unreal_service.get_component_by_name("UStaticMeshComponent", self._agent, "StaticMeshComponent")

        self._unreal_set_velocity_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_static_mesh_static_class, name="SetPhysicsLinearVelocity")
        self._unreal_set_angular_velocity_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_static_mesh_static_class,
                                                                                                     name="SetPhysicsAngularVelocityInRadians")

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
        new_location['z'] += self._z_offset
        new_rotation = np.array([0.0, 0.0, (random.random() - 0.5) * 180])
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, {
            "NewLocation": new_location,
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True})

        # reset velocity and angular velocity
        self._instance.unreal_service.call_function(self._root_component, self._unreal_set_velocity_func, {
            "NewVel": dict(zip(["X", "Y", "Z"], [0, 0, 0])),
            "bAddToCurrent": False
        })
        self._instance.unreal_service.call_function(self._root_component, self._unreal_set_angular_velocity_func, {
            "NewAngVel": dict(zip(["X", "Y", "Z"], [0, 0, 0])),
            "bAddToCurrent": False
        })

        return {
            "location": new_location,
            "rotation": new_rotation,
        }


class SimpleForceAgent(SimpleAgent):
    def __init__(self, instance):
        super().__init__(instance)
        self._use_agent_frame = True
        self._force_scale = 200
        self._torque_scale = self._force_scale * 180  # 60000 to large

    def get_action_space(self):
        if self._use_agent_frame:
            return gym.spaces.Dict({
                "add_force": gym.spaces.Box(0, 1, (1,), np.float64),
                "add_torque": gym.spaces.Box(-1, 1, (1,), np.float64),
            })
        else:
            return gym.spaces.Dict({
                "add_force": gym.spaces.Box(-1, 1, (3,), np.float64),
                "add_torque": gym.spaces.Box(-1, 1, (3,), np.float64),
            })

    def apply_action(self, action):
        if self._use_agent_frame:
            quat = Rotation.from_euler("xyz", self._obs['rotation'], degrees=True)
            add_force_world_frame = quat.as_matrix().dot(np.array([1, 0, 0])) * action['add_force'][0] * self._force_scale
            add_torque_world_frame = np.array([0, 0, action['add_torque'][0]]) * self._torque_scale

            add_force_args = {
                "Force": dict(zip(["X", "Y", "Z"], add_force_world_frame.tolist()))
            }
            self._instance.unreal_service.call_function(self._root_component, self._unreal_add_force_func, add_force_args)
            add_force_args = {
                "Torque": dict(zip(["X", "Y", "Z"], add_torque_world_frame.tolist()))
            }
            self._instance.unreal_service.call_function(self._root_component, self._unreal_add_torque_func, add_force_args)
        else:
            add_force_args = {
                "Force": dict(zip(["X", "Y", "Z"], action['add_force'].tolist()))
            }
            self._instance.unreal_service.call_function(self._root_component, self._unreal_add_force_func, add_force_args)
            add_force_args = {
                "Torque": dict(zip(["X", "Y", "Z"], action['add_torque'].tolist()))
            }
            self._instance.unreal_service.call_function(self._root_component, self._unreal_add_torque_func, add_force_args)


class HabitatNavAgent(SimpleAgent):
    def get_action_space(self):
        return gym.spaces.Dict({
            "move_forward": gym.spaces.Box(0, 50, (1,), np.float64),
            "move_right": gym.spaces.Box(-30, 30, (1,), np.float64),
            # "stop": gym.spaces.Box(0, 1, (1,), np.int),
        })

    def apply_action(self, action):
        new_rotation = self._obs['rotation'] + np.array([0, 0, action['move_right'][0]])
        quat = Rotation.from_euler("xyz", new_rotation, degrees=True)
        new_location = self._obs['location'] + action['move_forward'][0] * quat.as_matrix().dot(np.array([1, 0, 0]))

        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, {
            "NewLocation": dict(zip(["X", "Y", "Z"], new_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True})


class OpenBotAgent(AgentBase):
    def __init__(self, instance):
        super().__init__(instance)
        self._z_offset = 3
        self._force_scale = 1

        self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._nav_mesh_setup_func, args={
            "agent_height": 20.0, "agent_radius": 20.0
        })
        init_position = self.get_random_points(1)[0]
        init_position['z'] += self._z_offset  # add distance between agent center and z_min
        self._agent = self._instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_OpenBotPawn.BP_OpenBotPawn_C",
            location=init_position,
            rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": (random.random() - 0.5) * 2 * 180},
            spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        spear.log("agent = ", self._agent)

        # get vehicle component
        self._chaos_vehicle_movement_component = instance.unreal_service.get_component_by_type_v2(
            "/Script/CoreUObject.Class'/Script/ChaosVehicles.ChaosWheeledVehicleMovementComponent'", self._agent)
        self._chaos_vehicle_movement_component_class = instance.unreal_service.get_class(self._chaos_vehicle_movement_component)

        self._set_drive_torque_func = self._instance.unreal_service.find_function_by_name(uclass=self._chaos_vehicle_movement_component_class, name="SetDriveTorque")
        self._set_brake_torque_func = self._instance.unreal_service.find_function_by_name(uclass=self._chaos_vehicle_movement_component_class, name="SetBrakeTorque")
        self._stop_movement_func = self._instance.unreal_service.find_function_by_name(uclass=self._chaos_vehicle_movement_component_class, name="StopMovementImmediately")

        self._instance.unreal_service.call_function(uobject=self._hit_event_actor, ufunction=self._subscribe_actor_func, args={
            "Actor": self._instance.unreal_service.to_ptr(self._agent),
        })
        wheels_property = self._instance.unreal_service.find_property_by_name_on_uobject(self._chaos_vehicle_movement_component, "Wheels")
        ReturnValue = self._instance.unreal_service.get_property_value(wheels_property)
        self._wheels = ReturnValue[1:-1].strip().split(",")
        # print("self._wheels", self._wheels)
        self._wheel_class = self._instance.unreal_service.get_static_class_v2("/Script/CoreUObject.Class'/Script/ChaosVehicles.ChaosVehicleWheel'")
        self._get_wheel_angular_velocity_func = self._instance.unreal_service.find_function_by_name(uclass=self._wheel_class, name="GetWheelAngularVelocity")

    def get_observation_space(self):
        return gym.spaces.Dict({
            "location": gym.spaces.Box(-1000, 1000, (3,), np.float64),
            "rotation": gym.spaces.Box(-1000, 1000, (3,), np.float64),
            # "linear_velocity": gym.spaces.Box(-1000, 1000, (3,), np.float64), # TODO
            # "angular_velocity": gym.spaces.Box(-1000, 1000, (3,), np.float64), # TODO
            # "wheel_velocity": gym.spaces.Box(-1000, 1000, (4,), np.float64),
        })

    def get_observation(self):
        # get observation
        current_location = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_location_func)['ReturnValue']
        current_rotation = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_rotation_func)['ReturnValue']
        current_location = np.array([current_location['x'], current_location['y'], current_location['z']])
        current_rotation = np.array([current_rotation['roll'], current_rotation['pitch'], current_rotation['yaw']])
        wheel_velocity = np.zeros([4, ])
        for i in range(0, len(self._wheels)):
            wheel = self._wheels[i]
            wheel_velocity[i] = self._instance.unreal_service.call_function(self._instance.unreal_service.from_ptr(wheel), self._get_wheel_angular_velocity_func)['ReturnValue']

        self._obs = {
            "location": current_location,
            "rotation": current_rotation,
            "wheel_velocity": wheel_velocity,
        }
        return self._obs

    def get_action_space(self):
        return gym.spaces.Dict({
            # "set_drive_torque": gym.spaces.Box(-0.5, 0.5, (2,), np.float64),
            # "set_brake_torque": gym.spaces.Box(0, 1, (2,), np.float64)
            "apply_voltage": gym.spaces.Box(-1, 1, (2,), np.float64),
        })

    def apply_action(self, action):
        if "apply_voltage" in action:
            # see e87b904f - AOpenBotPawn::setDriveTorquesFromDutyCycle
            apply_voltage = np.array([action["apply_voltage"][0], action["apply_voltage"][1], action["apply_voltage"][0], action["apply_voltage"][1]])
            wheel_rotation_speed = self._obs["wheel_velocity"]

            battery_voltage = 12.0
            motor_velocity_constant = 65.89
            gear_ratio = 50.0
            motor_torque_max = 0.19
            control_dead_zone = 2.5
            action_scale = 255
            electrical_resistance = 0.5

            motor_torque_constant = 1 / motor_velocity_constant
            motor_speed = gear_ratio * wheel_rotation_speed * np.pi / 30.0
            counter_electromotive_force = motor_torque_constant * motor_speed

            motor_winding_current = battery_voltage * apply_voltage - counter_electromotive_force / electrical_resistance
            motor_torque = motor_torque_constant * motor_winding_current
            motor_torque = np.clip(motor_torque, -motor_torque_max, motor_torque_max)

            motor_torque[np.logical_and(abs(motor_speed) < 1e-5, abs(apply_voltage) <= control_dead_zone / action_scale)] = 0.0
            for wheel_index in range(0, 4):
                set_drive_torque_args = {
                    "DriveTorque": motor_torque[wheel_index],
                    "WheelIndex": wheel_index
                }
                self._instance.unreal_service.call_function(self._chaos_vehicle_movement_component, self._set_drive_torque_func, set_drive_torque_args)
        else:
            if "set_brake_torque" in action:
                set_brake_torque = action['set_brake_torque']

            for wheel_index in range(0, 4):
                set_drive_torque_args = {
                    "DriveTorque": action['set_drive_torque'][wheel_index % 2] * self._force_scale,
                    "WheelIndex": wheel_index
                }
                self._instance.unreal_service.call_function(self._chaos_vehicle_movement_component, self._set_drive_torque_func, set_drive_torque_args)
                if "set_brake_torque" in action:
                    set_brake_torque_args = {
                        "BrakeTorque": set_brake_torque[wheel_index % 2] * self._force_scale,
                        "WheelIndex": wheel_index
                    }
                    self._instance.unreal_service.call_function(self._chaos_vehicle_movement_component, self._set_brake_torque_func, set_brake_torque_args)

    def reset(self):
        new_location = self.get_random_points(1)[0]
        new_location['z'] += self._z_offset
        new_rotation = np.array([0, 0, (random.random() - 0.5) * 2 * 180])

        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, {
            "NewLocation": new_location,
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True})

        self._instance.unreal_service.call_function(self._chaos_vehicle_movement_component, self._stop_movement_func, {})
        return {
            "location": new_location,
            "rotation": new_rotation,
        }


class UrdfRobotAgent(AgentBase):
    def __init__(self, instance):
        super().__init__(instance)
        self._z_offset = 5
        self._wheel_velocity_scale = 10

        self._instance.unreal_service.call_function(uobject=self._nav_mesh_actor, ufunction=self._nav_mesh_setup_func, args={
            "agent_height": 120.0, "agent_radius": 50.0
        })

        self._agent = self._instance.unreal_service.spawn_actor(
            # class_name="/Game/Agents/BP_Fetch.BP_Fetch_C",
            class_name="/Game/Agents/BP_FetchBase.BP_FetchBase_C",
            location={"x": 0.0, "y": 0.0, "Z": 0.0},
            rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0},
            spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )

        self._unreal_constraint_static_class = self._instance.unreal_service.get_static_class_v2("/Script/CoreUObject.Class'/Script/Engine.PhysicsConstraintComponent'")
        self._unreal_set_angular_velocity_target_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_constraint_static_class,
                                                                                                            name="SetAngularVelocityTarget")
        self._unreal_get_constraint_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_constraint_static_class,
                                                                                               name="GetConstraint")

        self._unreal_set_velocity_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_static_mesh_static_class, name="SetPhysicsLinearVelocity")
        self._unreal_set_angular_velocity_func = self._instance.unreal_service.find_function_by_name(uclass=self._unreal_static_mesh_static_class,
                                                                                                     name="SetPhysicsAngularVelocityInRadians")

        self._joint_components = self._chaos_vehicle_movement_component = instance.unreal_service.get_components_by_type_v2(
            "/Script/CoreUObject.Class'/Script/Engine.PhysicsConstraintComponent'", self._agent)
        self._link_components = self._chaos_vehicle_movement_component = instance.unreal_service.get_components_by_type_v2(
            "/Script/CoreUObject.Class'/Script/Engine.StaticMeshComponent'", self._agent)

        self._constraint_instance_bpl_class = self._instance.unreal_service.get_static_class_v2("/Script/CoreUObject.Class'/Script/Engine.ConstraintInstanceBlueprintLibrary'")
        self._constraint_instance_bpl_default_object = instance.unreal_service.get_default_object(uclass=self._constraint_instance_bpl_class, create_if_needed=False)
        self._set_parent_dominate_func = self._instance.unreal_service.find_function_by_name(
            uclass=self._constraint_instance_bpl_class,
            name="SetParentDominates")

        self._instance.unreal_service.call_function(uobject=self._hit_event_actor, ufunction=self._subscribe_actor_func, args={
            "Actor": self._instance.unreal_service.to_ptr(self._agent),
        })

    def get_action_space(self):
        return gym.spaces.Dict({
            "wheel_joint_l": gym.spaces.Box(0, 1, (1,), np.float64),
            "wheel_joint_r": gym.spaces.Box(0, 1, (1,), np.float64),
        })

    def apply_action(self, actions):
        for joint_name, action in actions.items():
            joint_component = self._joint_components[joint_name]
            if joint_component > 0:
                self._instance.unreal_service.call_function(uobject=joint_component,
                                                            ufunction=self._unreal_set_angular_velocity_target_func,
                                                            args={"InVelTarget": {"x": action[0] * self._wheel_velocity_scale, "y": 0.0, "Z": 0.0}})

    def reset(self):
        new_location = self.get_random_points(1)[0]
        new_location['z'] += self._z_offset
        new_rotation = np.array([0, 0, (random.random() - 0.5) * 2 * 180])

        for joint_name, joint_component in self._joint_components.items():
            if "wheel_joint" in joint_name or "caster_joint" in joint_name:
                constraint = self._instance.unreal_service.call_function(joint_component, self._unreal_get_constraint_func)["ReturnValue"]
                self._instance.unreal_service.call_function(self._constraint_instance_bpl_default_object,
                                                            self._set_parent_dominate_func,
                                                            args={"Accessor": constraint, "bParentDominates": True})

        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, {
            "NewLocation": new_location,
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True})

        # TODO special tick to make sure agent teleport occur with bParentDominates == True and reset after ticking
        self._instance.engine_service.tick()
        self._instance.engine_service.end_tick()
        self._instance.engine_service.begin_tick()

        for joint_name, joint_component in self._joint_components.items():
            if "wheel_joint" in joint_name or "caster_joint" in joint_name:
                constraint = self._instance.unreal_service.call_function(joint_component, self._unreal_get_constraint_func)["ReturnValue"]
                # print("constraint", constraint)
                self._instance.unreal_service.call_function(self._constraint_instance_bpl_default_object,
                                                            self._set_parent_dominate_func,
                                                            args={"Accessor": constraint, "bParentDominates": False})

        # reset velocity and angular velocity
        for link_name, link_component in self._link_components.items():
            self._instance.unreal_service.call_function(link_component, self._unreal_set_velocity_func, {
                "NewVel": dict(zip(["X", "Y", "Z"], [0, 0, 0])),
                "bAddToCurrent": False
            })
            self._instance.unreal_service.call_function(link_component, self._unreal_set_angular_velocity_func, {
                "NewAngVel": dict(zip(["X", "Y", "Z"], [0, 0, 0])),
                "bAddToCurrent": False
            })

        return {
            "location": new_location,
            "rotation": new_rotation,
        }
