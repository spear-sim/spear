import gym
from gym import error, spaces
import cv2
import numpy as np
from typing import Tuple, Dict, Union, List, Any, Optional
from collections import namedtuple

from unrealai.unrealrl.envs import UnrealEnv
from unrealai.unrealrl.adaptors import RpcDataType as DataType
import unrealai.utils as utils
import unrealai.logger

logger = unrealai.logger.get_logger(__name__, "file")


class UnrealGymException(error.Error):
    """Any error related to the gym wrapper of unreal env."""

    pass


class UnrealGymObservationException(UnrealGymException):
    """Any errors related to observations in Gym setup."""

    pass


GymStepResult = Tuple[np.ndarray, float, bool, Dict]

# NOTE: Gym supports only one agent training
class UnrealToGymWrapper(gym.Env):
    """
    Gym Wrapper for Unreal Engine Environment
    Params:
        unreal_env  : An object of class UnrealEnv that will be wrapped inside Gym environment.
        agent_tag   : A string that specifies which specific agent to search for in the Unreal Environment provided you do not want to search for all RL agents.
                      For example, for environments with agents that have visual observations will have an additional tag called "VisualAgent".
                      So, providing agent_tag = VisualAgent will only consider Visual Agents for that experiment run.
    """

    def __init__(self, unreal_env: UnrealEnv, agent_tag: Optional[str] = None):
        if unreal_env is None:
            raise UnrealGymException(
                "Provided unreal environment for gym wrapper is empty. Please input a valid unreal environment object."
            )

        self._env = unreal_env

        # make sure agents and specs are loaded
        if len(self._env.agents) == 0:
            self._env._retrieve_agents()

        if len(self._env.action_specs) == 0:
            self._env._retrieve_action_specs()

        if len(self._env.obs_specs) == 0:
            self._env._retrieve_obs_specs()

        if len(self._env.agents) == 0:
            raise UnrealGymException(
                "Looks like there are no RL agents in Unreal Environment. Please check your unreal environment and try again."
            )
        elif len(self._env.agents) > 1:
            raise UnrealGymException(
                "The number of agents in the specified Unreal Environment is greater than 1. Gym works only with single agent."
            )

        if len(self._env.action_specs) == 0:
            raise UnrealGymException(
                "The Unreal Environment does not have action specs defined. Cannot proceed."
            )

        if len(self._env.obs_specs) == 0:
            raise UnrealGymException(
                "The Unreal Environment does not have observation specs defined. Cannot proceed."
            )

        # get appropriate agent with required tag name as the agent for this experiment
        # default, get the first agent in the list of agents
        if agent_tag:
            self._agent_name = self._env.conn.get_actor_list_of_tag(agent_tag)[0]
        else:
            self._agent_name = self._env.conn.get_agent_list()[0]

        self._done = False

        # agent's observation and action specs
        spec = namedtuple("spec", "obs action")
        agent_spec = spec(
            self._env.obs_specs[self.agent],
            self._env.action_specs[self.agent],
        )

        if agent_spec.action is None:
            raise UnrealGymException(
                f"Agent : {self.agent}'s action spec is not defined in the Unreal environment."
            )

        if agent_spec.obs is None:
            raise UnrealGymException(
                f"Agent : {self.agent}'s observation spec is not defined in the Unreal environment."
            )

        self._set_action_space(agent_spec.action)
        self._set_observation_space(agent_spec.obs)

    def _set_action_space(self, action_spec):
        list_spaces = []
        self._num_actions = action_spec["NumActions"]

        if self._num_actions == 0:
            raise UnrealGymException(
                f"There are no actions defined for this agent in the Unreal Environment."
            )

        for ind in range(self._num_actions):
            spec = action_spec["Specs"][ind]
            action_size = tuple(x for x in spec["Shape"])

            if spec["bIsDiscrete"]:
                if len(spec["Shape"]) == 1:
                    list_spaces.append(spaces.Discrete(action_size[0]))
                else:
                    list_spaces.append(spaces.MultiDiscrete(action_size))
            else:  # continuous actions
                bounds = spec["Bounds"]
                list_spaces.append(
                    spaces.Box(
                        bounds[0], bounds[1], shape=action_size, dtype=np.float32
                    )
                )

        # WARNING : Tuple of Gym spaces is not supported in many implementations of standard algorithms yet.
        # You will get NotImplementedError in this case.
        self._action_space = spaces.Tuple(list_spaces)
        if self._num_actions == 1:
            self._action_space = self._action_space[0]

    def _set_observation_space(self, observation_spec):
        self._observation_space = []
        num_obs = observation_spec["NumObservations"]
        if num_obs == 0:
            raise UnrealGymException(
                f"There are no observations defined for this agent in the Unreal Environment."
            )

        observation_size: Any

        specs = observation_spec["Specs"]
        if num_obs == 1:
            observation_size = tuple(x for x in specs[0]["Shape"])
            lower_bound = specs[0]["Bounds"][0]
            upper_bound = specs[0]["Bounds"][1]

            self._observation_space = spaces.Box(
                lower_bound,
                upper_bound,
                shape=observation_size,
                dtype=utils.get_numpy_dtype(specs[0]["Dtype"]),
            )
        else:
            list_spaces: List[gym.Space] = []
            observation_size = []
            for num in range(num_obs):
                # observation size
                observation_size.append(tuple(x for x in specs[num]["Shape"]))

                lower_bound = specs[num]["Bounds"][0]
                upper_bound = specs[num]["Bounds"][1]

                list_spaces.append(
                    spaces.Box(
                        lower_bound,
                        upper_bound,
                        shape=observation_size[num],
                        dtype=utils.get_numpy_dtype(specs[num]["Dtype"]),
                    )
                )
            # WARNING : Tuple of Gym spaces is not supported in many implementations of standard algorithms yet.
            # You will get NotImplementedError in this case.
            self._observation_space = spaces.Tuple(list_spaces)

    def get_observation(self) -> Union[List[np.ndarray], np.ndarray]:
        """This function is used to format obs from UnrealEnv into required Gym format."""

        obs = self._env.get_obs_for_agent(self.agent)

        if obs is None:
            raise UnrealGymException(
                f"Observation for agent {self.agent} could not be retrieved."
            )

        # make sure observation is in the required format
        self._env._validate_observation(self.agent, obs)

        if self._env.obs_specs[self.agent]["NumObservations"] == 1:
            obs = obs[0]
        else:
            obs = tuple(obs)

        return obs

    def pre_process_obs(self, obs) -> Union[List[np.ndarray], np.ndarray]:
        """Use this function to pre-process any observations received from Unreal Env."""
        pass

    def step(self, action: Any) -> GymStepResult:
        """
        Run one timestep of the environment. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state.
        Accepts an action and returns a tuple (observation, reward, done, info).
        Args:
            action (object): an action provided by the environment
        Returns:
            observation (np.ndarray): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (boolean): whether the episode has ended
            info (dict): contains auxiliary diagnostic information
        """

        if self._done:
            logger.warning(
                f"'step()' is called even though the current episode is already 'done'."
                "This causes an undefined behavior."
                "You should always call 'reset()' once you receive that the eposide is 'done'."
            )
            return

        # process action
        if self._num_actions == 1:
            action = [action]
        in_action: Dict[str, Optional[Any]] = {self.agent: action}
        action_to_send = self._env._format_action_to_send(in_action)

        if action_to_send is None:
            raise UnrealGymException(
                f"Could not format actions to required format for rpc communicator. Step called with invalid action input."
            )

        # send action
        self._env.conn.set_actions(action_to_send)

        # tick by 1 frame
        self._env.conn.tick()

        # collect observation
        self._current_obs = self.get_observation()

        # reward, done info
        info = self._env.conn.get_agent_step_info(self.agent)
        current_reward = utils.truncate(info["Reward"], 6)
        self._done = info["Done"]

        return (
            self._current_obs,
            current_reward,
            self._done,
            {},
        )

    def reset(self) -> Union[List[np.ndarray], np.ndarray]:
        """
        Resets the state of the environment and returns an initial observation.
        Returns: observation (object/list): the initial observation of the
        space.
        """
        self._env.reset()
        self._done = False

        obs = self.get_observation()

        return obs

    def render(self):
        cv2.imshow("Agent's observation", self._current_obs)
        cv2.waitKey(10)
        cv2.destroyAllWindows()

    def close(self):
        self._env.close()

    @property
    def reward_range(self) -> Tuple[float, float]:
        return -float("inf"), float("inf")

    @property
    def action_space(self):
        return self._action_space

    @property
    def observation_space(self):
        return self._observation_space

    @property
    def agent(self):
        return self._agent_name

    @property
    def unreal_env(self):
        return self._env

    @property
    def conn(self):
        return self._env.conn


# from gym.envs.registration import register

# don't register the env class with gym as this setup might be overriden by user
# register(
#     id="UnrealGymEnv-v0",
#     entry_point="unrealai.unrealrl.wrappers.gym_wrapper:UnrealToGymWrapper",
# )
