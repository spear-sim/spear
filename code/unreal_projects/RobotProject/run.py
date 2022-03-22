"""
This is an example code to show how to launch an unreal executable using our configuration system.
"""

import argparse

from interiorsim import InteriorSimEnv

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--config_files", nargs="*", help="List that contains locations to config files. If this argument is skipped, only default_config.yaml from unrealai package will be used to generate an output config file.")

    args = parser.parse_args()

    # create a list of config files to be used
    config_files = []

    # add config file from input if valid
    if args.config_files:
        for file in args.config_files:
            config_files.append(file)

    # load configs
    config = InteriorSimEnv.get_config(config_files)

    env = InteriorSimEnv(config)

    print()
    print("pinging...")
    print()
    print(env.conn.ping())
    print(env.conn.echo("Echo successful?"))
    print()

    print(f"Game paused successfully? {env.conn.pause()}")

    print("printing action space...")
    print(env.conn.getActionSpace())

    print("printing observation space...")
    print(env.conn.getObservation())

    # close your unreal executable environment gracefully
    env.close()
