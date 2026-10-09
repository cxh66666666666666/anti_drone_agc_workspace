#!/usr/bin/env python3

import os
import sys
import tempfile
import unittest

import torch


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from dqn_agent import DQNAgent, DuelingQNetwork  # noqa: E402
from replay_buffer import Experience, PrioritizedReplayBuffer  # noqa: E402


class DuelingQNetworkTest(unittest.TestCase):
    def test_dense_obstacle_observation_and_action_shape(self):
        net = DuelingQNetwork(22, 6)
        output = net(torch.zeros(3, 22))
        self.assertEqual(tuple(output.shape), (3, 6))

    def test_default_hidden_dims_forward_shape(self):
        net = DuelingQNetwork(21, 5)
        self.assertEqual(net(torch.randn(8, 21)).shape, (8, 5))

    def test_single_layer_forward_shape(self):
        net = DuelingQNetwork(21, 5, hidden_dims=(64,))
        self.assertEqual(net(torch.randn(4, 21)).shape, (4, 5))

    def test_three_layer_forward_shape(self):
        net = DuelingQNetwork(21, 5, hidden_dims=(256, 128, 64))
        self.assertEqual(net(torch.randn(2, 21)).shape, (2, 5))

    def test_invalid_hidden_dims_rejected(self):
        for bad in ((), (0,), (128, 0), (-8,), ("x",), (1.5,), 8):
            with self.assertRaises(ValueError):
                DuelingQNetwork(21, 5, hidden_dims=bad)


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
        agent = DQNAgent(2, 5, gamma=0.99, hidden_dims=(4,))
        with torch.no_grad():
            for parameter in agent.q_network.parameters():
                parameter.zero_()
            for parameter in agent.target_network.parameters():
                parameter.zero_()
            # 在线网络 Q(s,a) = A - mean(A) = [-2, -1, 0, 1, 2]
            agent.q_network.advantage.bias.copy_(
                torch.tensor([0.0, 1.0, 2.0, 3.0, 4.0]))
            # 目标网络 Q(s,a) = A - mean(A) = [-20, -10, 0, 10, 20]
            agent.target_network.advantage.bias.copy_(
                torch.tensor([10.0, 20.0, 30.0, 40.0, 50.0]))
        # 下一步只允许动作 {0, 1}：mask 生效时在线网络选动作 1 (Q=-1)，
        # TD target = 0.99 * Q_target(1) = 0.99 * (-10) = -9.9；
        # 当前 Q(s, a=0) = -2 → SmoothL1(7.9) = 7.4。
        experience = Experience(
            [0.0] * 2, 0, 0.0, [0.0] * 2, False, [0, 1])
        loss, td_errors = agent.update([experience])
        self.assertAlmostEqual(loss, 7.4, places=5)
        self.assertEqual(len(td_errors), 1)

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

    def test_save_load_roundtrip_custom_hidden_dims(self):
        source = DQNAgent(21, 5, hidden_dims=(64, 32))
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "model.pt")
            source.save(path)

            restored = DQNAgent(21, 5, hidden_dims=(64, 32))
            restored.load(path)

        for source_param, restored_param in zip(
                source.q_network.parameters(), restored.q_network.parameters()):
            self.assertTrue(torch.equal(source_param, restored_param))

    def test_load_rejects_mismatched_checkpoint_with_clear_error(self):
        source = DQNAgent(21, 5, hidden_dims=(128, 128))
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "model.pt")
            source.save(path)

            restored = DQNAgent(21, 5, hidden_dims=(256, 256, 256))
            with self.assertRaises(ValueError) as context:
                restored.load(path)
        message = str(context.exception)
        self.assertIn("hidden_dims", message)
        self.assertIn("256", message)

    def test_training_state_restores_optimizer_and_target_network(self):
        source = DQNAgent(2, 2, hidden_dims=(4,))
        experience = Experience(
            [0.0, 0.0], 0, 1.0, [0.0, 0.0], True, None)
        source.update([experience])
        state = source.training_state_dict()

        restored = DQNAgent(2, 2, hidden_dims=(4,))
        restored.load_training_state_dict(state)

        for name in ("q_network", "target_network"):
            source_params = getattr(source, name).parameters()
            restored_params = getattr(restored, name).parameters()
            for source_param, restored_param in zip(source_params,
                                                     restored_params):
                self.assertTrue(torch.equal(source_param, restored_param))
        self.assertEqual(len(restored.optimizer.state),
                         len(source.optimizer.state))


class PrioritizedReplayPersistenceTest(unittest.TestCase):
    def test_state_roundtrip_preserves_contents_and_sampling_state(self):
        source = PrioritizedReplayBuffer(capacity=8, alpha=0.6, beta=0.4)
        for action in range(4):
            source.push([action], action, float(action), [action + 1], False,
                        [0, 1], priority_hint=action + 1)
        source.sample(2)

        restored = PrioritizedReplayBuffer(capacity=8)
        restored.load_state_dict(source.state_dict())

        self.assertEqual(restored.count, source.count)
        self.assertEqual(restored.ptr, source.ptr)
        self.assertEqual(restored.beta, source.beta)
        self.assertEqual(restored.buffer, source.buffer)
        self.assertEqual(restored.tree.tree, source.tree.tree)


if __name__ == "__main__":
    unittest.main()
