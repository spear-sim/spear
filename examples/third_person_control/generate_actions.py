import argparse
import os

import numpy as np
import pandas as pd
import spear

default_action = {
    "AddMovementInput": np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "Jump": np.array([0.0, 0.0, 0.0], dtype=np.float64),
}


def get_data_frame(action):
    columns = np.array([[name + ".x", name + ".y", name + ".z"] for name, _ in action.items()]).ravel()  # append .x .y .z to each action name
    data = np.array([vec for _, vec in action.items()]).reshape(1, -1)
    return pd.DataFrame(columns=columns, data=data)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    args = parser.parse_args()

    df = pd.DataFrame()
    # turn right (negates accumulated "turn left" torque values)
    for _ in range(100):
        action = default_action.copy()
        df = pd.concat([df, get_data_frame(action)])
    for _ in range(1):
        action = default_action.copy()
        # action["AddMovementInput"] = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        action["Jump"] = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])
    for _ in range(100):
        action = default_action.copy()
        df = pd.concat([df, get_data_frame(action)])
    for _ in range(100):
        action = default_action.copy()
        action["AddMovementInput"] = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])
    # save to csv
    df.to_csv(args.actions_file, float_format="%.5f", mode="w", index=False)

    spear.log("Done.")
