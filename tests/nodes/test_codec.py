from time import time_ns

from dora_lerobot.contracts import Action, Observation
from dora_lerobot.nodes.codec import (
    action_to_message,
    message_to_action,
    message_to_observation,
    observation_to_message,
)


def test_observation_codec_preserves_contract():
    observation = Observation(time_ns(), {"joint_1": 0.2})
    decoded = message_to_observation(observation_to_message(observation))
    assert decoded == observation


def test_action_codec_preserves_contract():
    action = Action(("joint_1",), (0.2,), time_ns())
    decoded = message_to_action(action_to_message(action))
    assert decoded == action
