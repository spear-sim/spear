import argparse
import os



python_bin = "python"
models_dir = "/Users/mroberts/code/github/interiorsim/code/experiments/srcc/models"



def eval_model(sim_eval_mode, model_name, model_type, model_path):

    cmd = python_bin + " _eval_models_habitat_gibson.py" + \
        " --sim_eval_mode " + sim_eval_mode + \
        " --model_name "    + model_name + \
        " --model_type "    + model_type + \
        " --model_path "    + model_path
    print("")
    print(cmd)
    print("")
    retval = os.system(cmd)
    assert retval == 0



sim_eval_modes = ["challenge_sim", "test_sim"]
# sim_eval_modes = ["test_sim"]
# sim_eval_modes = ["challenge_sim"]

for sim_eval_mode in sim_eval_modes:

    model_name = "rgb_train_sliding_on_train_noise_multiplier_0.0"
    model_type = "rgb"
    model_path = \
        os.path.join(models_dir,
            "job_19888270"                        + "." \
            "sensor_RGB_SENSOR"                   + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_0.0"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000055"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "rgb_train_sliding_off_train_noise_multiplier_0.5"
    model_type = "rgb"
    model_path = \
        os.path.join(models_dir,
            "job_19633792"                        + "." \
            "sensor_RGB_SENSOR"                   + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_0.5"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000049"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "rgb_train_sliding_off_train_noise_multiplier_1.0"
    model_type = "rgb"
    model_path = \
        os.path.join(models_dir,
            "job_19633834"                        + "." \
            "sensor_RGB_SENSOR"                   + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_1.0"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000051"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "depth_train_sliding_on_train_noise_multiplier_0.0"
    model_type = "depth"
    model_path = \
        os.path.join(models_dir,
            "job_19888281"                        + "." \
            "sensor_DEPTH_SENSOR"                 + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_0.0"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000047"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "depth_train_sliding_off_train_noise_multiplier_0.5"
    model_type = "depth"
    model_path = \
        os.path.join(models_dir,
            "job_19633798"                        + "." \
            "sensor_DEPTH_SENSOR"                 + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_0.5"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000059"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "depth_train_sliding_off_train_noise_multiplier_1.0"
    model_type = "depth"
    model_path = \
        os.path.join(models_dir,
            "job_19633842"                        + "." \
            "sensor_DEPTH_SENSOR"                 + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_1.0"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000057"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "predicted_depth_train_sliding_on_train_noise_multiplier_0.0"
    model_type = "predicted_depth"
    model_path = \
        os.path.join(models_dir,
            "job_19888281"                        + "." \
            "sensor_DEPTH_SENSOR"                 + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_0.0"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000047"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "predicted_depth_train_sliding_off_train_noise_multiplier_0.5"
    model_type = "predicted_depth"
    model_path = \
        os.path.join(models_dir,
            "job_19633798"                        + "." \
            "sensor_DEPTH_SENSOR"                 + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_0.5"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000059"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)

    model_name = "predicted_depth_train_sliding_off_train_noise_multiplier_1.0"
    model_type = "predicted_depth"
    model_path = \
        os.path.join(models_dir,
            "job_19633842"                        + "." \
            "sensor_DEPTH_SENSOR"                 + "." \
            "train_data_gibson"                   + "." \
            "noise_multiplier_1.0"                + "." \
            "noise_model_controller_Proportional" + "." \
            "agent_radius_0.20"                   + "." \
            "success_reward_10.0"                 + "." \
            "slack_reward_-0.01"                  + "." \
            "collision_reward_0.0"                + "." \
            "spl_max_collisions_500"              + "_" \
            "ckpt.000000057"                      + \
            ".pth")
    eval_model(sim_eval_mode, model_name, model_type, model_path)
