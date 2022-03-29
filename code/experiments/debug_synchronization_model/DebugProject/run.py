"""
This is an example code to debug sychronization model used in interiorsim and SimulationController plugin.
"""

import argparse
import cv2
import numpy as np

from interiorsim import config
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
    config = config.get_config(config_files)

    env = InteriorSimEnv(config)

    print("python: pinging...")
    print(f"python: Is Game paused ? {env._isPaused()}")

    print("python: printing action space...")
    print(env._getActionSpace())

    print("python: printing observation space...")
    print(env._getObservationSpace())

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # IMPORTANT! perform one tick
    # here, the image captured before tick() should be blank because scenecapturecomponent2D which is responsible for image capture will be paused when game starts. 
    # unless you set the camera attached to scenecapturecomponent2D to tick when paused, the image will be blank

    env._beginTick()

    location_before_tick = env.get_observation("location")
    image_before_tick = env.get_observation("image")

    env._tick()

    location_after_tick = env.get_observation("location")
    image_after_tick = env.get_observation("image")

    env._endTick()

    print(f"before tick location {location_before_tick}")
    print(f"after tick location {location_after_tick}")

    cv2.imshow("before tick image observation", cv2.cvtColor(image_before_tick.astype(np.uint8), cv2.COLOR_BGR2RGB))
    cv2.imshow("after tick image observation", cv2.cvtColor(image_after_tick.astype(np.uint8), cv2.COLOR_BGR2RGB))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#


    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    print(f"python: before begintick---Is Game Paused? {env._isPaused()}")
    env._beginTick()

    print("python: sending action...")
    action = {"set_location": [0, 0, 20]}
    env._applyAction(action)

    print(f"python: before tick---Is Game Paused? {env._isPaused()}")
    env._tick()
    print(f"python: after tick---Is Game Paused? {env._isPaused()}")

    arr = env.get_observation("location")
    print("python: printing observation (numpy)....")
    print(arr)

    arr = env.get_observation("image")
    print("python: getobservation (image)....")
    cv2.imshow("image observation", cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_BGR2RGB))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    env._endTick()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    print("======================================python: applying set_location actions======================================")
    for i in range(5):
        print("-------------------------------------------------------------------------------------------------")
        env._beginTick()

        env._applyAction({"set_location": [i*10, 0, 15]})
        print(f"python: applying action...{i*10, 0, 15}")
        
        arr = env.get_observation("location")
        print("python: before tick getobservation (location)....")
        print(arr)

        arr_b1 = env.get_observation("image")
        arr_b2 = env.get_observation("image")
        assert np.all(arr_b1 == arr_b2)

        if i>=1:
            assert np.all(arr_b2 == arr_a2)

        env._tick()

        arr_a1 = env.get_observation("image")
        arr_a2 = env.get_observation("image")
        assert np.all(arr_a1 == arr_a2)

        assert not np.allclose(arr_a2, arr_b2)

        arr = env.get_observation("location")
        print("python: getobservation (location)....")
        print(arr)

        print("python: getobservation (image)....")
        cv2.imshow("before tick image observation", cv2.cvtColor(arr_b2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)

        print("python: getobservation (image)....")
        cv2.imshow("end tick image observation", cv2.cvtColor(arr_a2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)

        env._endTick()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    print("======================================python: applying apply_force actions======================================")
    
    for i in range(2):
        print("-------------------------------------------------------------------------------------------------")
        env._beginTick()

        env._applyAction({"apply_force": [1500000]})
        print(f"python: applying action of force multiplier value {1500000}")

        arr = env.get_observation("location")
        print("python: before tick getobservation (location)....")
        print(arr)

        arr_b1 = env.get_observation("image")
        arr_b2 = env.get_observation("image")
        assert np.all(arr_b1 == arr_b2)

        if i>=1:
            assert np.all(arr_b2 == arr_a2)

        env._tick()

        arr_a1 = env.get_observation("image")
        arr_a2 = env.get_observation("image")
        assert np.all(arr_a1 == arr_a2)

        assert not np.allclose(arr_a2, arr_b2)

        arr = env.get_observation("location")
        print("python: getobservation (location)....")
        print(arr)
        
        print("python: getobservation (image)....")
        cv2.imshow("before tick image observation", cv2.cvtColor(arr_b2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)

        print("python: getobservation (image)....")
        cv2.imshow("end tick image observation", cv2.cvtColor(arr_a2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)

        env._endTick()

    cv2.destroyAllWindows()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#

    # close your unreal executable environment gracefully
    env.close()
