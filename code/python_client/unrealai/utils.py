import sys, os
from subprocess import Popen
from unrealai.constants import PACKAGE_ROOT_DIR
from unrealai.exceptions import UnrealPlaformException, UnrealConfigError
from typing import Optional, Dict, TextIO
import yaml
import numpy as np
from unrealai.unrealrl.adaptors import RpcDataType


def check_project_path(path_to_uproject: str = "") -> None:
    if not path_to_uproject:
        # don't raise exception as this a valid handled case
        return

    if not os.path.exists(path_to_uproject):
        raise UnrealPlaformException(
            f"Entered path {path_to_uproject} does not exist. Please enter a valid path."
        )

    _, ext = os.path.splitext(path_to_uproject)
    if ext != ".uproject":
        raise UnrealPlaformException(
            f"The path entered contains an extension {ext} which is not a Unreal Engine project file extension. Please enter a path to a valid file."
        )


def start_unreal_engine(binary_path: str = "", *args) -> Optional[Popen]:

    if not binary_path:
        raise UnrealPlaformException(f"Empty path entered. Please enter a valid path.")

    if not os.path.exists(binary_path):
        raise UnrealPlaformException(
            f"Entered path {binary_path} does not exist. Please enter a valid path."
        )

    _, ext = os.path.splitext(binary_path)

    if sys.platform == "darwin":
        if ext != ".app":
            raise UnrealPlaformException(
                f"The path entered contains an extension {ext} which is not an executable on {sys.platform}. Please enter a path to a valid Unreal Engine executable file."
            )
        exe_path = binary_path + "/Contents/MacOS/"
        exe_file = os.listdir(exe_path)[0]
        try:
            p = Popen([exe_path + exe_file, *args])
        except OSError:
            raise OSError(
                f"Could not launch the Unreal Engine executable. Please check the executable's path."
            )
        except ValueError:
            raise ValueError(
                f"The parameters entered to run with the executable are invalid. Please check the parameters."
            )
    elif sys.platform == "win32":
        if ext != ".exe":
            raise UnrealPlaformException(
                f"The path entered contains an extension {ext} which is not an executable on {sys.platform}. Please enter a path to a valid Unreal Engine executable file."
            )
        try:
            p = Popen([binary_path, *args])
        except OSError:
            raise OSError(
                f"Could not launch the Unreal Engine executable. Please check the executable's path."
            )
        except ValueError:
            raise ValueError(
                f"The parameters entered to run with the executable are invalid. Please check the parameters."
            )
    elif sys.platform == "linux":
        if ext != "" and ext != ".sh":
            raise UnrealPlaformException(
                f"The path entered contains an extension {ext} which is not an executable on {sys.platform}. Please enter a path to a valid Unreal Engine executable file."
            )
        try:
            p = Popen([binary_path, *args])
        except OSError:
            raise OSError(
                f"Could not launch the Unreal Engine executable. Please check the executable's path."
            )
        except ValueError:
            raise ValueError(
                f"The parameters entered to run with the executable are invalid. Please check the parameters."
            )
    else:
        raise UnrealPlaformException(
            f"Unknown platform detected. Make sure you are on either MacOS, Windows(64bit, 32bit), or Ubuntu platform."
        )

    return p


def load_config(config_path: str = PACKAGE_ROOT_DIR + "/config.yaml") -> Dict:
    try:
        with open(config_path) as df:
            return _read_config_yaml(df)
    except OSError:
        path = os.path.abspath(config_path)
        raise UnrealConfigError(f"Config file could not be found at {path}.")
    except UnicodeDecodeError:
        raise UnrealConfigError(
            f"There was an error decoding Config file from {config_path}."
        )


def _read_config_yaml(stream: TextIO) -> Dict:
    try:
        return yaml.safe_load(stream)
    except yaml.parser.ParserError as e:
        raise UnrealConfigError(
            "Error parsing yaml file. Please check for formatting errors. "
            "A tool such as http://www.yamllint.com/ can be helpful with this."
        )
    except yaml.YAMLError as e:
        raise UnrealConfigError(f"Error reading yaml file data. {e}.")


def truncate(num, n):
    temp = str(num)
    for x in range(len(temp)):
        if temp[x] == ".":
            try:
                return float(temp[: x + n + 1])
            except:
                return float(temp)
    return float(temp)


def get_numpy_dtype(x):
    return {
        RpcDataType.Boolean.value: np.bool,
        RpcDataType.UInteger8.value: np.uint8,
        RpcDataType.Integer8.value: np.int8,
        RpcDataType.UInteger16.value: np.uint16,
        RpcDataType.Integer16.value: np.int16,
        RpcDataType.Float32.value: np.float32,
        RpcDataType.Double.value: np.float64,
    }[x]
