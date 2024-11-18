#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np

def get_drive_torques(duty_cycles, wheel_rotation_speeds, config):

    motor_velocity_constant = config.EXAMPLES.COMMON.OPENBOT_UTILS.MOTOR_VELOCITY_CONSTANT # Motor torque constant in [N.m/A]
    gear_ratio              = config.EXAMPLES.COMMON.OPENBOT_UTILS.GEAR_RATIO              # Gear ratio of the OpenBot motors
    motor_torque_constant   = 1.0 / motor_velocity_constant                                # Motor torque constant in [rad/s/V]

    battery_voltage         = config.EXAMPLES.COMMON.OPENBOT_UTILS.BATTERY_VOLTAGE         # Voltage of the battery powering the OpenBot [V]
    control_dead_zone       = config.EXAMPLES.COMMON.OPENBOT_UTILS.CONTROL_DEAD_ZONE       # Absolute duty cycle (in the ACTION_SCALE range) below which a command does not
                                                                                           # produce any torque on the vehicle
    motor_torque_max        = config.EXAMPLES.COMMON.OPENBOT_UTILS.MOTOR_TORQUE_MAX        # Motor maximal torque [N.m]
    electrical_resistance   = config.EXAMPLES.COMMON.OPENBOT_UTILS.ELECTRICAL_RESISTANCE   # Motor winding electrical resistance [Ohms]

    # Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to 
    # be processed by the low-level microcontroller. For more details, feel free to check the
    # "speedMultiplier" command in the OpenBot code.
    #     https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375
    action_scale = config.EXAMPLES.COMMON.OPENBOT_UTILS.ACTION_SCALE

    # Acquire the ground truth motor and wheel velocity for motor counter-electromotive force
    # computation purposes (or alternatively friction computation purposes).

    # The ground truth rotation speed of the motors in [rad/s]
    motor_speeds = gear_ratio * wheel_rotation_speeds

    # Compute the counter electromotive force using the motor torque constant
    counter_electromotive_forces = motor_torque_constant * motor_speeds # Expressed in [V]

    # The electrical current allowed to circulate in the motor is the result of the
    # difference between the applied voltage and the counter electromotive force
    motor_winding_currents = ((battery_voltage * duty_cycles) - counter_electromotive_forces) / electrical_resistance # Expressed in [A]

    # The torque is then obtained using the torque coefficient of the motor in [N.m]
    motor_torques = motor_torque_constant * motor_winding_currents

    # Motor torque is saturated to match the motor limits
    motor_torques = np.clip(motor_torques, -motor_torque_max, motor_torque_max)

    # The torque applied to the robot wheels is finally computed accounting for the gear ratio
    drive_torques = gear_ratio * motor_torques

    # Control dead zone at near-zero speed. This is a simplified but reliable way to deal with
    # the friction behavior observed on the real vehicle in the low-speed/low-duty-cycle regime.
    # For simplicity, we hardcode the threshold value 1e-5, but in future we should get this
    # value from the config system.
    drive_torques[np.logical_and(abs(motor_speeds) < 1e-5, abs(duty_cycles) <= control_dead_zone / action_scale)] = 0.0

    return drive_torques
