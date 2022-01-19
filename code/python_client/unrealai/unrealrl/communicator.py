import msgpackrpc  # pip install -e thirdparty/msgpack-rpc-python

from typing import List
from .adaptors import *


class Communicator(object):
    def __init__(self, ip="", port=8080, timeout_value=3000, reconn_limit=10):
        if ip == "":
            ip = "127.0.0.1"
        self._client = msgpackrpc.Client(
            msgpackrpc.Address(ip, port),
            timeout=timeout_value,
            reconnect_limit=reconn_limit,
        )

    def ping(self) -> bool:
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout
        Returns:
            bool:
        """
        return self._client.call("Ping")

    def echo(self, msg: str) -> str:
        """
        Echo a message from server. Can be used in addition to ping to test connection.
        Args:
            msg (string): Send a message that will be echoed back by server.
        """
        return self._client.call("Echo", msg)

    def close_connection(self):
        """Close the connection with Unreal Environment."""
        self._client.close()

    def pause(self, should_pause: bool) -> None:
        """
        Pause/Unpause unreal environment.
        Args:
            should_pause (bool): True to pause, False to release
        Returns:
            bool: If pause was successfull or not
        """
        self._client.call("Pause", should_pause)

    def is_paused(self) -> bool:
        """
        Returns true if the unreal environment is paused
        Returns:
            bool: If the unreal environment is paused
        """
        return self._client.call("IsPaused")

    def reset(self) -> None:
        """
        Reset unreal environment to original state.
        """
        self._client.call("Reset")

    def close(self) -> None:
        """
        Close unreal environment.
        """
        self._client.call("Close")

    def set_synchronous_mode(self, is_syncmode: bool) -> None:
        """
        Set if you want to run the simulation in synchornous mode
        Args:
            is_syncmode (bool) : true is you want sync mode, else false
        """
        self._client.call("SetSynchronousMode", is_syncmode)

    def is_synchronous_mode(self) -> bool:
        """
        Whether simulation is running in sync mode or not
        Returns:
            bool : true if simulation is running in sync mode
        """
        return self._client.call("IsSynchronousMode")

    def set_fixed_delta_seconds(self, fixed_delta_seconds) -> None:
        """
        Set required fixed delta seconds, this will run at defined framerate = 1 / fixed_delta_seconds
        Args:
            fixed_delta_seconds (double) : time in seconds
        """
        self._client.call("SetFixedDeltaSeconds", fixed_delta_seconds)

    def get_fixed_delta_seconds(self):
        """
        Get the fixed delta seconds the simulation is running on
        Returns:
            (double) : fixed delta seconds value
        """
        return self._client.call("GetFixedDeltaSeconds")

    # Debug
    # TODO: Need to figure out the configuration details
    # def set_physics_substepping(
    #     self, should_set: bool, max_sub_step_dt: float, max_sub_steps: int
    # ):
    #     self._client.call(
    #         "SetPhysicsSubStepping", should_set, max_sub_step_dt, max_sub_steps
    #     )
    # def set_physics_substepping(self) -> None:
    #     """
    #     Use this to specify substepping options for Physics computations
    #     """
    #     self._client.call("SetPhysicsSubStepping")

    def get_frame_count(self):
        """
        Return the number of frames elapsed since beginning of simulation
        Returns:
            number of frames elapsed since the beginning
        """
        return self._client.call("GetFrameCount")

    def get_agent_list(self) -> List[str]:
        """
        Get list of RL agents in the Unreal Environment
        """
        return self._client.call("GetAgentList")

    def get_actor_list_of_tag(self, tag_name: str = "") -> List[str]:
        """
        Get list of all actors in the level using the tag associated with them
        Args:
            tag_name (string): get all actors associated with this tag from unreal environment
        Returns:
            list : list of all actors in the world with tags as tag_name
        """
        return self._client.call("GetActorListOfTag", tag_name)

    def tick(self) -> None:
        """
        Moves enviroment ahead by one simulation time frame.
        """
        self._client.call("Tick")

    def get_observation_specs(self, agent_name: str) -> RpcObservationSpecs:
        """
        Retreive observation specs of an agent from unreal environment
        Args:
            - agent_name (str) : name of the agent as in unreal environment
        Returns:
            - RpcObservationSpecs object that contains details about the agent's observation specs
        """
        return self._client.call("GetObservationSpecs", agent_name)

    def get_action_specs(self, agent_name: str) -> RpcActionSpecs:
        """
        Retreive Action specs of an agent from unreal environment
        Args:
            - agent_name (str) : name of the agent as in unreal environment
        Returns:
            - RpcActionSpecs object that contains details about the agent's Action specs
        """
        return self._client.call("GetActionSpecs", agent_name)

    def set_actions(self, actions: RpcActionDict) -> None:
        """
        Output of learning model must be formatted to this datasturcture(RpcActionDict)
        before sending to server.
        Args:
            RpcActionDict : Dictionary of actions of agents to be sent to unreal environment
        """
        self._client.call("SetActions", actions)

    def get_observations(self, agent_names: List[str]) -> RpcObservationDict:
        """
        Args:
            agent_names : list of agent names whose observation is required
        Returns:
            RpcObservationDict : Dictionary of sensor readings from environment of all requested agents
        """
        return self._client.call("GetObservations", agent_names)

    def get_observation(self, agent_name: str) -> RpcObservation:
        """
        Retreive observation of an agent
        Returns:
            RpcObservation : see @RpcObservation more details about the type of ret value
        """
        return self._client.call("GetObservation", agent_name)

    def set_action(self, agent_name: str, action: RpcAction) -> None:
        """
        Sends action information to a particular agent in the enviroment.
        Args:
            agent_name : name of the agent who has to receeive this information
            action : see adators.RpcAction
        """
        self._client.call("SetAction", agent_name, action)

    def get_agent_step_info(self, actor_name: str) -> RpcAgentStepInfo:
        """
        This gets step information such as reward, done from server
        """
        return self._client.call("GetAgentStepInfo", actor_name)

    def begin_episode(self) -> None:
        """
        This should be called at the beginning of episode to reset the env to beginning stage.
        """
        self._client.call("BeginEpisode")

    def set_gameviewport_rendering_flag(self, flag: bool) -> None:
        """
        Args:
            flag (bool) :   False if you don't want to render game viewport.
                            By default UE has this flag as True
        """
        self._client.call("SetGameViewPortRenderingFlag", flag)

    def is_env_ready(self) -> bool:
        """
        Return:
            bool : True if env is ready, else False
        """
        return self._client.call("IsEnvReady")

    def start_experiment(self) -> None:
        """
        Sends command to server to start the experiment.
        This is a blocking call, thus, the server won't return control until it's ready.
        """
        self._client.call("StartExperiment")

    # Debug
    def run_console_command(self, command: str) -> bool:
        """
        Allows the client to execute a command in Unreal's native console, via an API.
        Args:
            command (str) : Desired Unreal Engine Console command to run
        Returns:
            bool: True if success in running command
        """
        return self._client.call("RunConsoleCommand", command)

    # experimental
    def allow_one_frame_renderthread_lag(self, allow: bool) -> None:
        """If true, Render thread lags behind gamethread by one frame.
        Args:
            allow (bool): If True, sets one frame lag
        """
        self._client.call("AllowOneFrameRTLag", allow)
