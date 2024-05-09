import gymnasium as gym
import pytest
from gymnasium.utils.env_checker import check_env

import gym_stompy  # noqa: F401


@pytest.mark.parametrize(
    "env_task, obs_type",
    [
        ("StompyInsertion-v0", "state"),
        ("StompyInsertion-v0", "pixels"),
        ("StompyInsertion-v0", "pixels_agent_pos"),
        ("StompyTransferCube-v0", "pixels"),
        ("StompyTransferCube-v0", "pixels_agent_pos"),
    ],
)
def test_stompy(env_task, obs_type):
    env = gym.make(f"gym_stompy/{env_task}", obs_type=obs_type)
    check_env(env.unwrapped)
