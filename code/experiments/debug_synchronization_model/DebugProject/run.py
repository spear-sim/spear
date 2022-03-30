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

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # For this section, you'll have to modify SimulationController.cpp with the following changes to observe the effects:
    # in the constructor of SimulationController(), change default frame_state from idle to request_pre_tick
    # in the beginning of beginTick() function, comment ASSERT() statement
    # at the end of endFrameEventHandler() function, change frame_state.store() from idle to request_pre_tick
    # print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")
    # # IMPORTANT!
    # # here, the image captured before tick() should be blank because scenecapturecomponent2D which is responsible for image capture will be paused when game starts. 
    # # unless you set the camera attached to scenecapturecomponent2D to tick when paused, the image will be blank

    # env._beginTick()

    # env._tick()

    # img_1 = env.get_observation("image") # will be blank image

    # env._endTick()
    # img_2 = env.get_observation("image") # will be blank image

    # cv2.imshow("1st end frame image (should be blank)", cv2.cvtColor(img_1.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)
    # cv2.imshow("2nd begin frame image (should be blank)", cv2.cvtColor(img_2.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)

    # # ---------------------------- #
    # env._beginTick()

    # env._tick()

    # img_3 = env.get_observation("image")  # will not be blank image

    # env._endTick()
    # img_4 = env.get_observation("image")  # will not be blank image

    # cv2.imshow("2nd end frame image (should not be blank)", cv2.cvtColor(img_3.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)
    # cv2.imshow("3rd begin frame image (should not be blank)", cv2.cvtColor(img_4.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)

    # # ---------------------------- #
    # env._beginTick()

    # env._tick()

    # img_5 = env.get_observation("image")  # will not be blank image

    # env._endTick()
    # img_6 = env.get_observation("image")  # will not be blank image
 
    # cv2.imshow("3rd end frame image (should not be blank)", cv2.cvtColor(img_5.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)
    # cv2.imshow("4th begin frame image (should not be blank)", cv2.cvtColor(img_6.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)

    # cv2.destroyAllWindows()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")
    
    #--------------------------------------------cheking game paused state at different stages of tick cycle------------------------------------------------------------------------#
    #-----------------------------------also, after applying action, checking actor location just before tick() and after tick()----------------------------------------------------#
    print(f"python: before begintick---Is Game Paused? {env._isPaused()}")
    env._beginTick()

    action = {"set_location": [0, 0, 20]}
    env._applyAction(action)
    print(f"python: applying action set_location...{[0, 0, 20]}")
    
    # arr = env.get_observation("location") # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
    # print(f"python: printing location before tick() {arr}")

    print(f"python: before tick---Is Game Paused? {env._isPaused()}")
    env._tick()
    print(f"python: after tick---Is Game Paused? {env._isPaused()}")

    arr = env.get_observation("location")
    print(f"python: printing location after tick() {arr}")

    env._endTick()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")

    # print("======================================python: applying set_location actions======================================")
    # for i in range(5):
    #     print("-------------------------------------------------------------------------------------------------")
    #     env._beginTick()

    #     env._applyAction({"set_location": [i*10, 0, 15]})
    #     print(f"python: applying action...{i*10, 0, 15}")
        
    #     arr = env.get_observation("location")
    #     print(f"python: before tick actor location....{arr}")

    #     arr_b1 = env.get_observation("image")
    #     arr_b2 = env.get_observation("image")
    #     assert np.all(arr_b1 == arr_b2)

    #     if i>=1:
    #         assert np.all(arr_b2 == arr_a2)

    #     env._tick()

    #     arr_a1 = env.get_observation("image")
    #     arr_a2 = env.get_observation("image")
    #     assert np.all(arr_a1 == arr_a2)

    #     assert not np.allclose(arr_a2, arr_b2)

    #     arr = env.get_observation("location")
    #     print(f"python: after tick actor location.... {arr}")

    #     cv2.imshow("before tick image observation", cv2.cvtColor(arr_b2.astype(np.uint8), cv2.COLOR_BGR2RGB))
    #     cv2.waitKey(0)

    #     cv2.imshow("end tick image observation", cv2.cvtColor(arr_a2.astype(np.uint8), cv2.COLOR_BGR2RGB))
    #     cv2.waitKey(0)

    #     env._endTick()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")   
    
    print("======================================python: applying apply_force actions======================================")    
    for i in range(10):
        print("-------------------------------------------------------------------------------------------------")
        env._beginTick()

        env._applyAction({"apply_force": [300000]})
        print(f"python: applying action of force multiplier value {300000}")

        # arr = env.get_observation("location") # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
        # print(f"python: before tick actor location....{arr}")

        # arr_b1 = env.get_observation("image") # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
        # arr_b2 = env.get_observation("image") # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
        # assert np.all(arr_b1 == arr_b2)

        # if i>=1:
            # assert np.all(arr_b2 == arr_a2)

        env._tick()

        arr_a1 = env.get_observation("image")
        arr_a2 = env.get_observation("image")
        assert np.all(arr_a1 == arr_a2)

        # assert not np.allclose(arr_a2, arr_b2)

        arr = env.get_observation("location")
        print(f"python: after tick action location.... {arr}")
        
        # cv2.imshow("before tick image observation", cv2.cvtColor(arr_b2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        # cv2.waitKey(0)

        cv2.imshow("end tick image observation", cv2.cvtColor(arr_a2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)

        env._endTick()

    cv2.destroyAllWindows()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#

    # close your unreal executable environment gracefully
    env.close()
