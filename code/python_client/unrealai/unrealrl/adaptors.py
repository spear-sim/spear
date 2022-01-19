"""
Modified from AirSim (https://microsoft.github.io/AirSim) project, thanks to MIT license.
"""

from typing import Dict, Any, List
from enum import Enum


class MsgpackMixin:
    """
    Class for sending and receiving RPC data using msgpackrpc.
    """

    def __repr__(self):
        from pprint import pformat

        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def to_msgpack(self, *args, **kwargs):
        return self.__dict__

    @classmethod
    def from_msgpack(cls, encoded):
        obj = cls()
        obj.__dict__ = {
            k: (
                v
                if not isinstance(v, dict)
                else getattr(getattr(obj, k).__class__, "from_msgpack")(v)
            )
            for k, v in encoded.items()
        }
        return obj


class RpcActionSpec(MsgpackMixin):
    """
    Specifies the action specification.
    - Shape: Defines the action space
    - Dtype: Data type of actions
    - bIsDiscrete: True if action space is discrete
    - Bounds: Bounds respected by these actions
    - Description: Describes the action values represented by this observationspec
    """

    Shape = []
    Dtype: Any
    bIsDiscrete: bool
    Bounds: Any
    Description: str


class RpcActionSpecs(MsgpackMixin):
    """
    Specifies the action specification.
    - NumActions: Defines the number of RpcAction arrays
    - Specs: List of RpcAction
    """

    NumActions = 0
    Specs: Any


class RpcAction(MsgpackMixin):
    """
    Specifies the action information of an agent
    - Data : List of float values
    """

    Data: Any

    def __init__(self, Data: Any):
        self.Data = Data


class RpcActionDict(MsgpackMixin):
    """
    Specifies the action dictionary
    - AgentToActionMap: Map of action info for each agent
    """

    AgentToActionMap: Dict[str, List[RpcAction]] = {}


class RpcDataType(Enum):
    """
    Specifies different types of Data types that can be used.
    """

    Boolean = 0
    UInteger8 = 1
    Integer8 = 2
    UInteger16 = 3
    Integer16 = 4
    Float32 = 5
    Double = 6


class RpcObservationSpec(MsgpackMixin):
    """
    Specifies the observation specification
    - Shape: Defines the observation space
    - Bounds: Defines any bounds on the observations values
    - Dtype: Data type for this observation
    - Description: Describes the observation values
    """

    Shape: Any
    Bounds: Any
    Dtype: Any
    Description: str


class RpcObservationSpecs(MsgpackMixin):
    """
    Specifies the observation specification
    - NumObservations: Number of observations
    - Specs: Contains all the RpcObservationSpec details
    """

    NumObservations = 0
    Specs: Any


class RpcObservation(MsgpackMixin):
    """
    Specifies the observation info
    - bIsSet: True if observation is set by Unreal Environment. If False, disregard the observation
    - Data: Defines the observation information
    """

    bIsSet: bool = False
    Data: Any


class RpcObservationDict(MsgpackMixin):
    """
    Specifies the observation dictionary
    - AgentToObservationMap : Map of observation info for each agent
    """

    AgentToObservationMap: Dict[str, List[RpcObservation]] = {}

    def __init__(self, in_dict={}):
        self.AgentToObservationMap = in_dict


class RpcAgentStepInfo(MsgpackMixin):
    """
    Specifies Reward and done information
    - Reward : float value corresponding to reward of an agent
    - Done : True if end of episode for an agent
    """

    Reward = 0.0
    Done = False
