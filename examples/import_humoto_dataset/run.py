#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()
    
    with instance.begin_frame():

        # spawn checking_floor_lamp_with_right_hand_suit animation
        location = {"X": 425.0, "Y": 275.0, "Z": 25.0}
        bp_humoto_path = "/Game/Humoto/checking_floor_lamp_with_right_hand_suit/checking_floor_lamp_with_right_hand_suit/FbxScene_checking_floor_lamp_with_right_hand_suit.FbxScene_checking_floor_lamp_with_right_hand_suit_C"
        bp_humoto_uclass = game.unreal_service.load_class(uclass="AActor", name=bp_humoto_path)
        bp_humoto = game.unreal_service.spawn_actor(uclass=bp_humoto_uclass, location=location)

        # spawn drinking_from_mug1_and_talking_suit animation
        location = {"X": 225.0, "Y": 475.0, "Z": 30.0}
        bp_humoto_path = "/Game/Humoto/drinking_from_mug1_and_talking_suit/drinking_from_mug1_and_talking_suit/FbxScene_drinking_from_mug1_and_talking_suit.FbxScene_drinking_from_mug1_and_talking_suit_C"
        bp_humoto_uclass = game.unreal_service.load_class(uclass="AActor", name=bp_humoto_path)
        bp_humoto = game.unreal_service.spawn_actor(uclass=bp_humoto_uclass, location=location)

    with instance.end_frame():
        pass

    spear.log("Done.")
