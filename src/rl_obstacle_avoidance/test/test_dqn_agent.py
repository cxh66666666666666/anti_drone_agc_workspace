#!/usr/bin/env python3

import os
import sys
import tempfile
import unittest

import torch


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from dqn_agent import DQNAgent  # noqa: E402
from replay_buffer import Experience  # noqa: E402


class DQNAgentPersistenceTest(unittest.TestCase):
    def test_action_selection_respects_allowed_actions(self):
        agent = DQNAgent(21, 5)
        state = [0.0] * 21
        for _ in range(20):
            self.assertIn(
                agent.select_action(state, epsilon=1.0, allowed_actions=[1, 3]),
                (1, 3))
        self.assertIn(
            agent.select_action(state, epsilon=0.0, allowed_actions=[2, 4]),
            (2, 4))

    def test_td_target_masks_disallowed_next_actions(self):
        agent = DQNAgent(21, 5, gamma=0.99)
        with torch.no_grad():
            for parameter in agent.q_network.parameters():
                parameter.zero_()
            for parameter in agent.target_network.parameters():
                parameter.zero_()
            agent.target_network.net[-1].bias.copy_(
                torch.tensor([1.0, 2.0, 3.0, 4.0, 100.0]))
        experience = Experience(
            [0.0] * 21, 0, 0.0, [0.0] * 21, False, [0])
        loss = agent.update([experience])
        self.assertAlmostEqual(loss, 0.5 * 0.99 ** 2, places=5)

    def test_saved_state_dict_loads_without_format_migration(self):
        source = DQNAgent(21, 5)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "model.pt")
            source.save(path)

            restored = DQNAgent(21, 5)
            restored.load(path)

        for source_param, restored_param in zip(
                source.q_network.parameters(), restored.q_network.parameters()):
            self.assertTrue(torch.equal(source_param, restored_param))


if __name__ == "__main__":
    unittest.main()
