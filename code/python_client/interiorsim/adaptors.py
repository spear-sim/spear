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


class RpcDataType(Enum):
    """
    Specifies different types of Data types that can be used.
    """

    Boolean = 0
    UInteger8 = 1
    Integer8 = 2
    UInteger16 = 3
    Integer16 = 4
    UInteger32 = 5
    Integer32 = 6
    Float32 = 7
    Double = 8


class RpcEndiannessType(Enum):
    """
    Different types of Endianness
    """

    LittleEndian = 0
    BigEndian = 1


class Box(MsgpackMixin):
    """
    Contains the action and observation space information
    """

    low: float
    hight: float
    shape: List
    dtype: RpcDataType
