from abc import ABC, abstractmethod
from typing import Dict, Optional, List, Any, Union

from .communicator import Communicator

import unrealai.utils as utils

from .adaptors import (
    RpcActionDict as ActionDict,
    RpcObservationDict as ObservationDict,
    RpcActionSpec as ActionSpec,
    RpcAction as Action,
    RpcObservationSpec as ObservationSpec,
    RpcObservation as Observation,
    RpcAgentStepInfo as AgentStepInfo,
    RpcDataType as DataType,
)

from unrealai.exceptions import (
    UnrealEnvException,
    UnrealActionException,
    UnrealObservationException,
    UnrealActionSpecException,
    UnrealObservationException,
)

import unrealai.logger

import numpy as np
import time

logger = unrealai.logger.get_logger(__name__, "file")


class BaseEnv(ABC):
    @abstractmethod
    def step(self):
        """
        Signals the environment to move the simulation forward
        by one step.
        """

    @abstractmethod
    def reset(self):
        """
        Signals the environment to reset the simulation.
        """

    @abstractmethod
    def close(self):
        """
        Signals the environment to close.
        """


class UnrealEnv(BaseEnv):
    """
    Environment class for performing RL experiments with Unreal Engine via UnrealRL Plugin.
    """

    # TODO: use class or named tuple for env_config
    def __init__(self, env_config: Optional[Dict]):

        super(UnrealEnv, self).__init__()

        # read config variables
        if env_config is None or len(env_config) == 0:
            env_config = utils.load_config()["env_config"]

        path_to_exe: Optional[str] = env_config.get("binary_path", None)
        ip: str = env_config.get("server_ip", "")
        port: str = env_config.get("server_port", "8080")
        timeout_value: int = env_config.get("timeout_value", 3600)
        reconn_tries: int = env_config.get("reconn_tries", 1000)
        launch_params: List[str] = env_config.get("launch_params", [])
        thread_sync: bool = env_config.get("enable_thread_sync", False)
        delta_seconds: float = env_config.get("delta_seconds", 0.1)

        # start engine, returns a Popen object
        # path_to_exe = None, for connecting to already running instance of Unreal Engine Environment
        if path_to_exe is not None:
            self.__ue_process = utils.start_unreal_engine(path_to_exe, *launch_params)
            # TODO: Why is this sleep needed?
            # (Without this sleep, even "play.py" does not work in Linux. It also shows sometimes error in MacOS.)
            time.sleep(5)

        # rpc client communicator to exchange information with Unreal Engine
        self._communicator = Communicator(
            ip=ip,
            port=int(port),
            timeout_value=timeout_value,
            reconn_limit=reconn_tries,
        )

        logger.info(f"Waiting for Unreal Environment to get ready...")

        # send start experiment signal to server
        self._communicator.start_experiment()

        # sync gamethread and render thread
        self._communicator.allow_one_frame_renderthread_lag(not thread_sync)

        # to keep track of number of frames elapsed
        self._frame_count = 0

        # agent names
        self._agents: List[str] = []
        self._retrieve_agents()
        if len(self._agents) == 0:
            raise UnrealEnvException(
                "Could not retrieve any RL agents from Unreal Environment. Make sure the environment has agents configured."
            )

        # obs specs of all valid agents
        self._obs_specs: Dict[str, Optional[ObservationSpec]] = {}
        self._retrieve_obs_specs()
        if len(self._obs_specs) == 0:
            raise UnrealEnvException(
                "Could not retrieve any observation specs from Unreal Environment. Make sure the environment has agents and observation specs for those agents defined."
            )

        # action specs of all valid agents
        self._action_specs: Dict[str, Optional[ActionSpec]] = {}
        self._retrieve_action_specs()
        if len(self._action_specs) == 0:
            raise UnrealEnvException(
                "Could not retrieve any action specs from Unreal Environment. Make sure the environment has agents and action specs for those agents defined."
            )

        # Ensure same timestep size between two ticks.
        self._communicator.set_fixed_delta_seconds(delta_seconds)

        # env current obs
        self._current_obs: Optional[ObservationDict] = None

        # env current reward
        self._current_reward: Dict[str, Optional[float]] = {}
        self._cumm_reward: Dict[str, Optional[float]] = {}

        # env current done state
        self._current_done: Dict[str, Optional[bool]] = {}

    def _retrieve_agents(self) -> None:
        """Retrieves RL agents from Unreal Environment."""
        self._agents = self._communicator.get_agent_list()

    def _retrieve_obs_specs(self) -> None:
        """Retrieves observation specifications of RL agents from Unreal Environment."""
        for agent in self.agents:
            self._obs_specs[agent] = self._communicator.get_observation_specs(agent)

    def _retrieve_action_specs(self) -> None:
        """Retrieves action specifications of RL agents from Unreal Environment."""
        for agent in self.agents:
            self._action_specs[agent] = self._communicator.get_action_specs(agent)

    def print_obs_spec(self) -> None:
        """Prints observation specs of RL agents in a readable format."""
        logger.info(f"Observation Specs :")
        logger.info(
            f"---------------------------------------------------------------------"
        )
        for agent_name, obs_spec in self.obs_specs.items():
            if obs_spec is not None:
                logger.info(f"Agent Name : {agent_name}")
                logger.info(
                    f"Number of observations recorded by this agent are {obs_spec['NumObservations']}"
                )
                for ind, spec in enumerate(obs_spec["Specs"]):
                    logger.info(f"___________Observation_{ind}_____________________")
                    logger.info(f"{spec['Description']}")
                    logger.info(f"Data Type is {DataType(spec['Dtype']).name}")
                    logger.info(
                        f"Number of readings in each dimension : {spec['Shape']}"
                    )
                    logger.info(f"Bounds on these observations are : {spec['Bounds']}")
            logger.info(
                f"---------------------------------------------------------------------"
            )

    def print_action_spec(self) -> None:
        """Prints action specs of RL agents in a readable format."""
        logger.info(f"Action Specs :")
        logger.info(
            f"---------------------------------------------------------------------"
        )
        for agent_name, action_spec in self.action_specs.items():
            if action_spec is not None:
                logger.info(f"Agent Name : {agent_name}")
                logger.info(
                    f"Number of Actions for this agent are {action_spec['NumActions']}"
                )
                for ind, spec in enumerate(action_spec["Specs"]):
                    logger.info(f"______________Action_{ind}_________________________")
                    logger.info(f"{spec['Description']}")
                    logger.info(
                        f"Number of actions in each dimension : {spec['Shape']}"
                    )
                    logger.info(
                        f"Action values are {'Discrete' if spec['bIsDiscrete'] else 'Continuous'} in nature"
                    )
                    logger.info(f"Bounds on the action are {spec['Bounds']}")
        logger.info(
            f"---------------------------------------------------------------------"
        )

    def _validate_observation(self, agent_name, observations):
        """Make sure the format of observations are as specified in the observation specs of the agent"""
        observation_specs = self.obs_specs[agent_name]["Specs"]

        if len(observations) != len(observation_specs):
            raise UnrealObservationException(
                f"Number of observations ({len(observations)}) and observations specs ({len(observation_specs)}) do not match for agent {agent_name}"
            )

        for observation, observation_spec in zip(observations, observation_specs):
            if observation.shape != tuple(observation_spec["Shape"]):
                raise UnrealObservationException(
                    f"Observation shape for agent {agent_name} is inconsistent. Expected {tuple(observation_spec['Shape'])}, but received {observation.shape}."
                )

            if observation.dtype != utils.get_numpy_dtype(observation_spec["Dtype"]):
                raise UnrealObservationException(
                    f"Observation dtype for agent {agent_name} is inconsistent. Expected {utils.get_numpy_dtype(observation_spec['Dtype'])}, but received {observation.dtype}."
                )

            if (
                observation.min() < observation_spec["Bounds"][0]
                or observation.max() > observation_spec["Bounds"][1]
            ):
                raise UnrealObservationException(
                    f"Observation is not within expected range. Expected {observation_spec['Bounds']}, but received values in range [{observation.min(), observation.max()}]"
                )

    def get_obs_numpy(
        self, obs: Optional[ObservationDict]
    ) -> Dict[str, Optional[List[np.ndarray]]]:
        """Convert observations retrieved from Unreal Environment to a numpy array."""

        ret_dict: Dict[str, Optional[List[np.ndarray]]] = {}

        for agent_name, obs_vec in obs["AgentToObservationMap"].items():
            if len(obs_vec) == 0:
                continue

            # go over each obs in the vector of observations returned by agent
            for ind, obs in enumerate(obs_vec):
                if obs["bIsSet"]:
                    tensor = obs["Data"]
                    shape = self._obs_specs[agent_name]["Specs"][ind]["Shape"]
                    # oshape = tuple(x for x in shape) if dim_count > 1 else (1, shape[0])
                    oshape = tuple(x for x in shape)
                    arr = np.array(
                        tensor,
                        dtype=utils.get_numpy_dtype(
                            self._obs_specs[agent_name]["Specs"][ind]["Dtype"]
                        ),
                    ).reshape(oshape)
                    if agent_name in ret_dict:
                        ret_dict[agent_name].append(arr)
                    else:
                        ret_dict[agent_name] = [arr]
        return ret_dict

    def get_obs_for_agent(self, agent_name: str) -> Optional[List[np.ndarray]]:
        """
        Retrieve observations of a particular agent from Unreal Environment.
        Returns : All observations that the agents observes in a numpy array format
        """
        obs_vec = self._communicator.get_observation(agent_name)

        if len(obs_vec) == 0:
            return None

        obs_dict = {"AgentToObservationMap": {agent_name: obs_vec}}

        ret_dict = self.get_obs_numpy(obs_dict)

        if len(ret_dict) != 0:
            return ret_dict[agent_name]
        else:
            raise UnrealObservationException(
                f"Could not retrieve observation for agent {agent_name}"
            )

        return None

    def _format_action_to_send(
        self, input_actions: Dict[str, Optional[Any]]
    ) -> Optional[ActionDict]:
        """
        Formats the input action data into the required format for Communicator to send it to
        Unreal Environment.
        """
        ret_obj = ActionDict()
        for agent_name, actions in input_actions.items():
            if actions is not None:
                list_of_actions = []
                for ind, action in enumerate(actions):
                    self._validate_action(agent_name, ind, action)
                    if type(action) == np.ndarray:
                        list_of_actions.append(Action(action.tolist()))
                    else:
                        list_of_actions.append(Action([action.item()]))
                ret_obj.AgentToActionMap = {agent_name: list_of_actions}

        return ret_obj if len(ret_obj.AgentToActionMap) != 0 else None

    def _validate_action(self, agent_name: str, ind, action: np.ndarray):
        """Validate input action"""
        action_spec = self.action_specs[agent_name]["Specs"][ind]

        # shape check
        if not action_spec["bIsDiscrete"]:
            # TODO: Validate np.ndarray shaped action
            if tuple(x for x in action.shape) != tuple(x for x in action_spec["Shape"]):
                raise UnrealActionException(
                    f"Actions array shape for agent {agent_name} is inconsistent. Expected {action_spec['Shape']}, but got {action.shape}."
                )

        # # TODO: Discuss if we should check this. Maybe convert before sending?
        # # dtype check
        # if action.dtype != utils.get_numpy_dtype(action_spec["Dtype"]):
        #     raise UnrealActionException(
        #         f"Action dtype for agent {agent_name} is inconsistent. Expected {utils.get_numpy_dtype(action_spec['Dtype'])}, but received {action.dtype}."
        #     )

        # bounds check
        if (
            np.amin(action) < action_spec["Bounds"][0]
            or np.amax(action) > action_spec["Bounds"][1]
        ):
            raise UnrealActionException(
                f"Input action bounds are not within required range. Required range is {action_spec['Bounds']}, your values range in {np.amin(action), np.amax(action)}"
            )

    def step(self, action: Dict[str, Optional[Any]]) -> None:
        """
        Moves the simulation ahead by one frame. Accepts a dictionary of actions for agents
        who are expected to receive actions and updates the internal state of the environment.
        """
        # process action
        _actions = self._format_action_to_send(action)

        if _actions is None:
            raise UnrealActionException(
                f"Could not format actions to required format for rpc communicator. Step called with invalid action input."
            )

        # send action
        self._communicator.set_actions(_actions)

        # tick by 1 frame
        self._communicator.tick()

        # collect observation
        self._current_obs = self._communicator.get_observations(self.agents)

        if len(self.current_obs) == 0:
            raise UnrealObservationException(
                f"Could not get valid observations from Unreal Environment."
            )

        # reward, done info for only valid observations received.
        for _name in self.agents:
            if _name in self.current_obs.keys():
                info = self._communicator.get_agent_step_info(_name)
                self._current_reward[_name] = utils.truncate(info["Reward"], 6)
                self._current_done[_name] = info["Done"]
            else:
                self._current_reward[_name] = None
                self._current_done[_name] = None

        self._frame_count += 1

    def reset(self) -> None:
        """Resets the unreal environment to initial state, i.e beginning of the episode."""
        # send begin episode signal
        self._communicator.begin_episode()
        # tick by 1 frame to apply begin episode changes
        self._communicator.tick()

    def close(self) -> None:
        self._communicator.close()
        self._communicator.close_connection()

    @property
    def conn(self) -> Communicator:
        return self._communicator

    @property
    def agents(self) -> List[str]:
        return self._agents

    @property
    def obs_specs(self) -> Dict[str, Optional[ObservationSpec]]:
        return self._obs_specs

    @property
    def action_specs(self) -> Dict[str, Optional[ActionSpec]]:
        return self._action_specs

    @property
    def current_obs(self) -> Dict[str, Union[List[np.ndarray], np.ndarray]]:
        return self.get_obs_numpy(self._current_obs)

    @property
    def current_reward(self) -> Dict[str, Optional[float]]:
        return self._current_reward

    @property
    def current_done(self) -> Dict[str, Optional[bool]]:
        return self._current_done
