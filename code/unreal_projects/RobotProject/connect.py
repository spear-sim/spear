"""
This is an example code to show how to launch an unreal executable using our configuration system.
"""

import argparse
import numpy as np

from interiorsim.adaptors import RpcEndiannessType, RpcAction
from interiorsim import InteriorSimEnv
from interiorsim.communicator import Communicator

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--config_files", nargs="*", help="List that contains locations to config files. If this argument is skipped, only default_config.yaml from unrealai package will be used to generate an output config file.")

    args = parser.parse_args()

    # # create a list of config files to be used
    # config_files = []

    # # add config file from input if valid
    # if args.config_files:
    #     for file in args.config_files:
    #         config_files.append(file)

    # # load configs
    # config = InteriorSimEnv.get_config(config_files)

    # env = InteriorSimEnv(config)

    client = Communicator("127.0.0.1", 8000, timeout=1, reconnect_limit=1)

    print()
    print("pinging...")
    print()
    print(client.ping())
    print(client.echo("Echo successful?"))
    print()

    print(f"Game paused successfully? {client.pause()}")

    print("printing action space...")
    print(client.getActionSpace())

    print("printing observation space...")
    print(client.getObservationSpace())

    oshape = tuple(x for x in client.getObservationSpace()["vector"]["shape"])

    data_type = InteriorSimEnv.get_numpy_dtype(client.getObservationSpace()["vector"]["dtype"])

    if client.getEndianness() == InteriorSimEnv.getEndianness().value:
        print("1")
        pass
    elif client.getEndianness() == RpcEndiannessType.BigEndian.value:
        data_type = data_type.newbyteorder(">")
        print("2")
    elif client.getEndianness() == RpcEndiannessType.LittleEndian.value:
        data_type = data_type.newbyteorder("<")
        print("3")

    print("oshape = {}".format(oshape))
    print("data_type = {}".format(data_type))
    arr = np.frombuffer(client.getObservation()["vector"], dtype=data_type, count=-1).reshape(oshape)

    print("printing observation....")
    print(client.getObservation())

    print("printing observation (numpy)....")
    print(arr)

    print("sending action...")
    # action = RpcAction({"force": [1, 1, 0, 0]})
    action = {"force": [1, 1, 0, 0]}
    client.applyAction(action)

    # close your unreal executable environment gracefully
    client.close()
