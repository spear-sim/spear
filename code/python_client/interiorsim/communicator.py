import msgpackrpc

from interiorsim.adaptors import RpcAction, RpcEndiannessType  # pip install -e code/third_party/msgpack-rpc-python

class Communicator(object):
    def __init__(self, ip="", port=8080, timeout=3000, reconnect_limit=10):
        if ip == "":
            ip = "127.0.0.1"
        self._client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout=timeout, reconnect_limit=reconnect_limit)

    def ping(self) -> bool:
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout
        Returns:
            bool:
        """
        return self._client.call("ping")

    def echo(self, msg: str) -> str:
        """
        Echo a message from server. Can be used in addition to ping to test connection.
        Args:
            msg (string): Send a message that will be echoed back by server.
        """
        return self._client.call("echo", msg)

    def closeConnection(self):
        """Close the connection with Unreal Environment."""
        self._client.close()

    def pause(self) -> None:
        """
        Pause unreal simulation.
        """
        return self._client.call("pause")

    def unPause(self) -> None:
        """
        Unpause unreal simulation.
        """
        return self._client.call("unPause")

    def isPaused(self) -> bool:
        """
        Returns true if the unreal environment is paused
        Returns:
            bool: If the unreal environment is paused
        """
        return self._client.call("isPaused")

    def getEndianness(self) -> RpcEndiannessType:
        return self._client.call("getEndianness")

    def close(self) -> None:
        """
        Close unreal environment.
        """
        self._client.call("close")

    def getObservationSpace(self):
        """
        Retreive observation space of an agent
        """
        return self._client.call("getObservationSpace")

    def getActionSpace(self):
        """
        Retreive action space of an agent
        """
        return self._client.call("getActionSpace")

    def getObservation(self):
        """
        Retreive observation of an agent
        Returns:
            RpcObservation : see @RpcObservation more details about the type of ret value
        """
        return self._client.call("getObservation")

    def applyAction(self, action) -> None:
        """
        Sends action information to a particular agent in the enviroment.
        Args:
            action : see adators.RpcAction
        """
        self._client.call("applyAction", action)

    def set_game_viewport_rendering_flag(self, flag: bool) -> None:
        """
        Args:
            flag (bool) :   False if you don't want to render game viewport. By default UE has this flag as True
        """
        self._client.call("SetGameViewPortRenderingFlag", flag)
