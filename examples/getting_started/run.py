#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import spear
import time


NUM_STEPS = 100


# Custom Env implementation for OpenBot
class OpenBotEnv(spear.Env):

    def __init__(self, config):
    
        super().__init__(config)
        self._wheel_rotation_speeds = None
    
    def step(self, action):

        obs, reward, is_done, step_info = super().step(action=action)
        assert "wheel_encoder" in obs.keys()
        self._wheel_rotation_speeds = obs["wheel_encoder"]

        return obs, reward, is_done, step_info

    def reset(self, reset_info=None):
    
        obs = super().reset(reset_info=reset_info)
        assert "wheel_encoder" in obs.keys()
        self._wheel_rotation_speeds = obs["wheel_encoder"]

        return obs

    def _apply_action(self, action):

        # calculate wheel torques based on input PWM signals
        if "apply_voltage" in action.keys():

            assert self._wheel_rotation_speeds is not None
            assert len(action["apply_voltage"]) == 2

            duty_cycle = np.array([action["apply_voltage"][0], action["apply_voltage"][1], action["apply_voltage"][0], action["apply_voltage"][1]], dtype=np.float64)

            # Motor torque: 1200 gf.cm (gram force centimeter) == 0.1177 N.m
            # Gear ratio: 50
            # Max wheel torque: 5.88399 N.m
            # https://www.conrad.de/de/p/joy-it-com-motor01-getriebemotor-gelb-schwarz-passend-fuer-einplatinen-computer-arduino-banana-pi-cubieboard-raspbe-1573543.html

            motor_velocity_constant = self._config.OPENBOT_ENV.MOTOR_VELOCITY_CONSTANT # Motor torque constant in [N.m/A]
            gear_ratio              = self._config.OPENBOT_ENV.GEAR_RATIO              # Gear ratio of the OpenBot motors
            motor_torque_constant   = 1.0 / motor_velocity_constant;                   # Motor torque constant in [rad/s/V]

            battery_voltage         = self._config.OPENBOT_ENV.BATTERY_VOLTAGE         # Voltage of the battery powering the OpenBot [V]
            control_dead_zone       = self._config.OPENBOT_ENV.CONTROL_DEAD_ZONE       # Absolute duty cycle (in the ACTION_SCALE range) below which a command does not produce any torque on the vehicle
            motor_torque_max        = self._config.OPENBOT_ENV.MOTOR_TORQUE_MAX # Motor maximal torque [N.m]
            electrical_resistance   = self._config.OPENBOT_ENV.ELECTRICAL_RESISTANCE   # Motor winding electrical resistance [Ohms]

            # Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to 
            # be processed by the low-level microcontroller. For more details, feel free to check the
            # "speedMultiplier" command in the OpenBot code.
            # https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375
            action_scale = self._config.OPENBOT_ENV.ACTION_SCALE

            # Acquire the ground truth motor and wheel velocity for motor counter-electromotive force
            # computation purposes (or alternatively friction computation purposes).

            # The ground truth velocity of the robot wheels in [RPM]
            wheel_rotation_speeds = self._wheel_rotation_speeds

            # The ground truth rotation speed of the motors in [rad/s]
            motor_speed = gear_ratio * wheel_rotation_speeds * np.pi / 30.0 # Expressed in [rad/s]

            # Compute the counter electromotive force using the motor torque constant
            counter_electromotive_force = motor_torque_constant * motor_speed; # Expressed in [V]

            # The electrical current allowed to circulate in the motor is the result of the
            # difference between the applied voltage and the counter electromotive force
            motor_winding_current = ((battery_voltage * duty_cycle) - counter_electromotive_force) / electrical_resistance; # Expressed in [A]

            # The torque is then obtained using the torque coefficient of the motor in [N.m]
            motor_torque = motor_torque_constant * motor_winding_current

            # Motor torque is saturated to match the motor limits
            motor_torque = np.clip(motor_torque, -motor_torque_max, motor_torque_max)

            # The torque applied to the robot wheels is finally computed accounting for the gear ratio
            wheel_torques = gear_ratio * motor_torque

            # Control dead zone at near-zero speed. This is a simplified but reliable way to deal with
            # the friction behavior observed on the real vehicle in the low-speed/low-duty-cycle regime.
            for i in range(len(duty_cycle)):
                # TODO: get value from the config system
                if abs(motor_speed[i]) < 1e-5 and abs(duty_cycle[i]) <= control_dead_zone / action_scale :
                    wheel_torques[i] = 0.0

            # modify action before sending it to the simulator
            action["apply_wheel_torques"] = wheel_torques
            action.pop("apply_voltage")

        super()._apply_action(action)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # create Env object
    if config.SIMULATION_CONTROLLER.AGENT == "SphereAgent":
        env = spear.Env(config)
    elif config.SIMULATION_CONTROLLER.AGENT == "VehicleAgent":
        env = OpenBotEnv(config)

    # reset the simulation to get the first observation    
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        cv2.imshow("camera.final_color", obs["camera.final_color"]) # note that spear.Env returns BGRA by default
        cv2.waitKey(0)

    # take a few steps
    for i in range(NUM_STEPS):
        if config.SIMULATION_CONTROLLER.AGENT == "SphereAgent":
            obs, reward, done, info = env.step(action={"add_force": np.array([10000.0, 0.0, 0.0], dtype=np.float64)})
            if not args.benchmark:
                spear.log("SphereAgent: ")
                spear.log("position:", obs["position"])
                spear.log("rotation:", obs["rotation"])
                spear.log("camera:", obs["camera.final_color"].shape, obs["camera.final_color"].dtype)
                spear.log(reward, done, info)
        elif config.SIMULATION_CONTROLLER.AGENT == "VehicleAgent":
            obs, reward, done, info = env.step(action={"apply_voltage": np.array([1.0, 1.0], dtype=np.float64)})
            if not args.benchmark:
                spear.log("VehicleAgent: ")
                spear.log("state_data:", obs["state_data"])
                spear.log("wheel_encoder:", obs["wheel_encoder"])
                spear.log("camera:", obs["camera.final_color"].shape, obs["camera.final_color"].dtype)
                spear.log("sonar:", obs["sonar"])
                spear.log(reward, done, info)
        else:
            assert False

        if not args.benchmark:
            cv2.imshow("camera.final_color", obs["camera.final_color"]) # note that spear.Env returns BGRA by default
            cv2.waitKey(0)

        if done:
            env.reset()

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / NUM_STEPS)*1000.0, NUM_STEPS / elapsed_time_seconds))
    else:
        cv2.destroyAllWindows()

    # close the environment
    env.close()

    spear.log("Done.")
