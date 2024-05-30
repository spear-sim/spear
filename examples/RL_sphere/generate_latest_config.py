import os

import spear

if __name__ == '__main__':

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    spear.configure_system(config)

    # write temp file
    temp_dir = os.path.realpath(os.path.join(config.SPEAR.INSTANCE.TEMP_DIR))
    temp_config_file = os.path.realpath(os.path.join(temp_dir, config.SPEAR.INSTANCE.TEMP_CONFIG_FILE))

    spear.log("Writing temp config file: " + temp_config_file)

    os.makedirs(temp_dir, exist_ok=True)
    with open(temp_config_file, "w") as output:
        config.dump(stream=output, default_flow_style=False)