#!/usr/bin/env python3

import os
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from training_utils import (  # noqa: E402
    count_termination_reason,
    evaluation_score,
    parse_hidden_dims,
    rolling_evaluation_metrics,
    update_best_score,
)

try:
    import train_dqn  # noqa: E402
    HAS_TRAIN_SCRIPT = True
except ImportError:
    HAS_TRAIN_SCRIPT = False

try:
    import evaluate_policy  # noqa: E402
    HAS_EVAL_SCRIPT = True
except ImportError:
    HAS_EVAL_SCRIPT = False


def make_eval(success=0.0, safe_success=None, collision=0.0, risk_entry=0.0,
              timeout=0.0, reward=0.0):
    if safe_success is None:
        safe_success = success
    return {
        "success_rate": success,
        "safe_success_rate": safe_success,
        "collision_rate": collision,
        "risk_entry_rate": risk_entry,
        "timeout_rate": timeout,
        "mean_reward": reward,
    }


def episode_info(done_reason, collision_events=0, success=False, safe=None):
    """构造评估所需的最小 info 字段集。"""
    if safe is None:
        safe = success
    return {
        "done_reason": done_reason,
        "episode_success": success,
        "safe_success": safe,
        "collision_events": collision_events,
        "geometry_collision_events": 0,
        "min_geometry_clearance": 1.0,
        "min_obstacle_min": 1.0,
        "position_error": 0.0,
    }


class _ScriptedEnv:
    """按脚本回放每回合 reset/step 结果的假环境。"""

    n_actions = 2
    episode_step_limit = 1

    def __init__(self, episodes):
        self._episodes = list(episodes)
        self._index = -1
        self.reset_ok = False

    def reset(self):
        self._index += 1
        self.reset_ok = self._episodes[self._index]["reset_ok"]
        return [0.0]

    def allowed_actions(self):
        return [0, 1]

    def step(self, action):
        info = dict(self._episodes[self._index]["info"])
        info.setdefault("applied_action", action)
        return [0.0], 1.0, True, info


def scripted_episodes():
    """shield_infeasible / collision / timeout / reset 失败 各一回合。"""
    return [
        {"reset_ok": True, "info": episode_info("shield_infeasible")},
        {"reset_ok": True,
         "info": episode_info("collision", collision_events=1)},
        {"reset_ok": True, "info": episode_info("timeout", success=True)},
        {"reset_ok": False, "info": None},
    ]


class RollingEvaluationTest(unittest.TestCase):
    def test_window_one_equals_single_evaluation(self):
        metrics = make_eval(success=0.8, collision=0.1, timeout=0.05, reward=12.5)
        rolling = rolling_evaluation_metrics([metrics], 1)
        self.assertEqual(rolling["window_used"], 1)
        self.assertAlmostEqual(rolling["rolling_success_rate"], 0.8)
        self.assertAlmostEqual(rolling["rolling_collision_rate"], 0.1)
        self.assertAlmostEqual(rolling["rolling_timeout_rate"], 0.05)
        self.assertAlmostEqual(rolling["rolling_mean_reward"], 12.5)

    def test_partial_window_uses_available_entries(self):
        history = [make_eval(success=1.0, reward=10.0),
                   make_eval(success=0.5, reward=20.0)]
        rolling = rolling_evaluation_metrics(history, 3)
        self.assertEqual(rolling["window_used"], 2)
        self.assertAlmostEqual(rolling["rolling_success_rate"], 0.75)
        self.assertAlmostEqual(rolling["rolling_mean_reward"], 15.0)

    def test_full_window_keeps_only_last_n(self):
        history = [make_eval(success=0.1, reward=0.0),
                   make_eval(success=0.5, reward=10.0),
                   make_eval(success=0.9, reward=20.0),
                   make_eval(success=1.0, reward=30.0)]
        rolling = rolling_evaluation_metrics(history, 3)
        self.assertEqual(rolling["window_used"], 3)
        self.assertAlmostEqual(rolling["rolling_success_rate"], 0.8)
        self.assertAlmostEqual(rolling["rolling_mean_reward"], 20.0)

    def test_empty_history_gives_zero_metrics(self):
        rolling = rolling_evaluation_metrics([], 3)
        self.assertEqual(rolling["window_used"], 0)
        self.assertAlmostEqual(rolling["rolling_success_rate"], 0.0)
        self.assertAlmostEqual(rolling["rolling_mean_reward"], 0.0)

    def test_invalid_window_rejected(self):
        with self.assertRaises(ValueError):
            rolling_evaluation_metrics([], 0)

    def test_missing_fields_rejected(self):
        with self.assertRaises(ValueError):
            rolling_evaluation_metrics([{"success_rate": 1.0}], 1)

    def test_new_bucket_rates_are_averaged_when_present(self):
        history = [
            dict(make_eval(success=1.0), shield_infeasible_rate=0.2,
                 reset_failure_rate=0.0),
            dict(make_eval(success=0.0), shield_infeasible_rate=0.0,
                 reset_failure_rate=0.1),
        ]
        rolling = rolling_evaluation_metrics(history, 2)
        self.assertAlmostEqual(
            rolling["rolling_shield_infeasible_rate"], 0.1)
        self.assertAlmostEqual(rolling["rolling_reset_failure_rate"], 0.05)

    def test_new_bucket_rates_tolerate_legacy_records_without_the_fields(self):
        # 旧评估记录缺这两个列时按 0.0 处理，且不影响必需的校验
        rolling = rolling_evaluation_metrics([make_eval(success=1.0)], 1)
        self.assertAlmostEqual(
            rolling["rolling_shield_infeasible_rate"], 0.0)
        self.assertAlmostEqual(rolling["rolling_reset_failure_rate"], 0.0)

    def test_trajectory_complete_rate_is_averaged_when_present(self):
        history = [
            dict(make_eval(success=1.0), trajectory_complete_rate=0.5),
            dict(make_eval(success=1.0), trajectory_complete_rate=0.9),
        ]
        rolling = rolling_evaluation_metrics(history, 2)
        self.assertAlmostEqual(
            rolling["rolling_trajectory_complete_rate"], 0.7)

    def test_trajectory_complete_rate_tolerates_legacy_records(self):
        # 旧评估记录缺该列时按 0.0 处理
        rolling = rolling_evaluation_metrics([make_eval(success=1.0)], 1)
        self.assertAlmostEqual(
            rolling["rolling_trajectory_complete_rate"], 0.0)

    def test_empty_history_reports_new_bucket_rates(self):
        rolling = rolling_evaluation_metrics([], 3)
        self.assertEqual(rolling["window_used"], 0)
        self.assertAlmostEqual(
            rolling["rolling_shield_infeasible_rate"], 0.0)
        self.assertAlmostEqual(rolling["rolling_reset_failure_rate"], 0.0)
        self.assertAlmostEqual(
            rolling["rolling_trajectory_complete_rate"], 0.0)


class BestModelSelectionTest(unittest.TestCase):
    @staticmethod
    def _rolling_of(history, window=3):
        return rolling_evaluation_metrics(history, window)

    def test_stable_high_success_beats_single_spike(self):
        spike = [make_eval(success=0.2), make_eval(success=1.0),
                 make_eval(success=0.2)]
        stable = [make_eval(success=0.8), make_eval(success=0.8),
                  make_eval(success=0.8)]
        self.assertGreater(evaluation_score(self._rolling_of(stable)),
                           evaluation_score(self._rolling_of(spike)))

    def test_lower_collision_rate_preferred(self):
        higher_collision = [make_eval(success=0.8, collision=0.1)] * 3
        lower_collision = [make_eval(success=0.8, collision=0.05)] * 3
        self.assertGreater(evaluation_score(self._rolling_of(lower_collision)),
                           evaluation_score(self._rolling_of(higher_collision)))

    def test_collision_rate_precedes_raw_success(self):
        risky = [make_eval(success=1.0, safe_success=0.5, collision=0.2)] * 3
        safe = [make_eval(success=0.7, safe_success=0.7, collision=0.0)] * 3
        self.assertGreater(evaluation_score(self._rolling_of(safe)),
                           evaluation_score(self._rolling_of(risky)))

    def test_higher_mean_reward_breaks_tie(self):
        low_reward = [make_eval(success=0.8, collision=0.0, timeout=0.0,
                                reward=5.0)] * 3
        high_reward = [make_eval(success=0.8, collision=0.0, timeout=0.0,
                                 reward=15.0)] * 3
        self.assertGreater(evaluation_score(self._rolling_of(high_reward)),
                           evaluation_score(self._rolling_of(low_reward)))

    def test_zero_valid_episodes_never_becomes_best(self):
        self.assertIsNone(update_best_score(None, (1.0, 0.0, 0.0, 100.0), 0))
        best = (0.8, -0.1, -0.1, 10.0)
        self.assertEqual(
            update_best_score(best, (1.0, 0.0, 0.0, 100.0), 0), best)

    def test_better_score_updates_best(self):
        best = (0.8, -0.1, -0.1, 10.0)
        self.assertEqual(
            update_best_score(best, (1.0, 0.0, 0.0, 100.0), 10),
            (1.0, 0.0, 0.0, 100.0))

    def test_worse_score_keeps_best(self):
        best = (0.9, -0.05, -0.05, 20.0)
        self.assertEqual(
            update_best_score(best, (0.5, -0.4, 0.0, 5.0), 30), best)


class TerminationReasonCountTest(unittest.TestCase):
    def test_counts_accumulate_per_reason(self):
        counts = {}
        count_termination_reason(counts, "collision")
        count_termination_reason(counts, "collision")
        count_termination_reason(counts, "avoidance_complete")
        count_termination_reason(counts, None)
        self.assertEqual(counts, {
            "collision": 2,
            "avoidance_complete": 1,
            "unknown": 1,
        })

    def test_returns_same_dict_for_chaining(self):
        counts = {"a": 1}
        self.assertIs(count_termination_reason(counts, "b"), counts)
        self.assertEqual(counts["b"], 1)


class ArgumentParsingTest(unittest.TestCase):
    def test_parse_hidden_dims_valid(self):
        self.assertEqual(parse_hidden_dims("128,128"), (128, 128))
        self.assertEqual(parse_hidden_dims("256,128"), (256, 128))
        self.assertEqual(parse_hidden_dims(" 64 , 32 "), (64, 32))

    def test_parse_hidden_dims_invalid(self):
        for bad in ("", ",", "128,", ",128", "128,,64", "abc", "128,-4", "0",
                    "128,1.5"):
            with self.assertRaises(ValueError):
                parse_hidden_dims(bad)


@unittest.skipUnless(HAS_TRAIN_SCRIPT, "train_dqn 依赖的 ROS 模块不可用")
class TrainCliValidationTest(unittest.TestCase):
    @staticmethod
    def _parse(argv):
        with mock.patch("sys.argv", ["train_dqn.py"] + argv):
            return train_dqn.parse_args()

    def test_hidden_dims_flag_parsed_to_tuple(self):
        args = self._parse(["--hidden-dims", "256,128"])
        self.assertEqual(args.hidden_dims, (256, 128))

    def test_curriculum_is_enabled_by_default_and_can_be_disabled(self):
        self.assertTrue(self._parse([]).curriculum)
        self.assertFalse(self._parse(["--no-curriculum"]).curriculum)

    def test_default_evaluation_interval_limits_simulation_overhead(self):
        self.assertEqual(self._parse([]).eval_interval, 100)

    def test_full_training_checkpoint_roundtrip(self):
        agent = train_dqn.DQNAgent(2, 2, hidden_dims=(4,))
        buffer = train_dqn.PrioritizedReplayBuffer(8)
        buffer.push([0.0, 0.0], 1, 2.0, [1.0, 0.0], False, [0, 1])
        progress = {
            "completed_episodes": 12,
            "total_steps": 345,
            "update_steps": 90,
            "evaluation_index": 2,
            "last_evaluated_episode": 10,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "training_state.pt")
            train_dqn.save_training_checkpoint(
                path, agent, buffer, progress, [], None, None, 1000)
            restored_agent = train_dqn.DQNAgent(2, 2, hidden_dims=(4,))
            restored_buffer = train_dqn.PrioritizedReplayBuffer(8)
            payload = train_dqn.load_training_checkpoint(
                path, restored_agent, restored_buffer)

        self.assertEqual(payload["progress"], progress)
        self.assertEqual(payload["epsilon_decay_steps"], 1000)
        self.assertEqual(len(restored_buffer), 1)
        for source_param, restored_param in zip(
                agent.q_network.parameters(),
                restored_agent.q_network.parameters()):
            self.assertTrue(train_dqn.torch.equal(source_param,
                                                  restored_param))

    def test_curriculum_progressively_adds_harder_layouts(self):
        scenarios = [
            {"layout": {"type": "dense_corridor"}},
            {"layout": {"type": "frontal_block"}},
            {"layout": {"type": "single_side"}},
            {"layout": {"type": "mirror_pair"}},
        ]
        early = train_dqn.curriculum_pool(scenarios, 0.0)
        late = train_dqn.curriculum_pool(scenarios, 1.0)
        self.assertEqual([item["layout"]["type"] for item in early],
                         ["frontal_block", "single_side"])
        self.assertEqual(len(late), len(scenarios))

    def test_manifest_loader_excludes_empty_obstacle_scenes(self):
        with mock.patch.object(train_dqn.os.path, "isfile", return_value=True), \
                mock.patch("builtins.open", mock.mock_open(read_data='{"scenarios": ['
                    '{"split": "test", "scenario_id": "empty", "obstacles": []},'
                    '{"split": "test", "scenario_id": "used", "obstacles": [{"type": "wall"}]}'
                    ']}')):
            scenarios = train_dqn.load_split_scenarios("test.json", "test")
        self.assertEqual([item["scenario_id"] for item in scenarios], ["used"])

    def test_evaluation_rotates_test_scenarios_without_learning(self):
        class FakeEnv:
            n_actions = 2
            episode_step_limit = 1
            reset_ok = True

            def __init__(self):
                self.armed = []

            def arm_deploy(self, scenario):
                self.armed.append(scenario["scenario_id"])

            def reset(self):
                return [0.0]

            def allowed_actions(self):
                return [0, 1]

            def step(self, action):
                return [0.0], 1.0, True, {
                    "applied_action": action,
                    "done_reason": "timeout",
                    "episode_success": True,
                    "safe_success": True,
                    "min_obstacle_min": 1.0,
                    "position_error": 0.0,
                    "geometry_collision_events": 0,
                    "min_geometry_clearance": 1.0,
                    "collision_events": 0,
                }

        agent = mock.Mock()
        agent.select_action.return_value = 0
        env = FakeEnv()
        scenarios = [{"scenario_id": "test-a"}, {"scenario_id": "test-b"}]
        metrics = train_dqn.evaluate(agent, env, 3, 1, scenarios)
        self.assertEqual(env.armed, ["test-a", "test-b", "test-a"])
        self.assertEqual(metrics["scenario_ids"], env.armed)
        self.assertFalse(agent.learn.called)

    def test_negative_learning_rate_rejected(self):
        with self.assertRaises(SystemExit):
            self._parse(["--learning-rate", "0"])
        with self.assertRaises(SystemExit):
            self._parse(["--learning-rate", "-0.001"])

    def test_invalid_eval_window_rejected(self):
        with self.assertRaises(SystemExit):
            self._parse(["--eval-window", "0"])
        with self.assertRaises(SystemExit):
            self._parse(["--eval-window", "-3"])

    def test_invalid_hidden_dims_rejected(self):
        with self.assertRaises(SystemExit):
            self._parse(["--hidden-dims", ""])
        with self.assertRaises(SystemExit):
            self._parse(["--hidden-dims", "128,abc"])


@unittest.skipUnless(HAS_TRAIN_SCRIPT and HAS_EVAL_SCRIPT,
                     "评估脚本依赖的 ROS 模块不可用")
class EvaluationBucketTest(unittest.TestCase):
    @staticmethod
    def _agent():
        agent = mock.Mock()
        agent.select_action.return_value = 0
        return agent

    def _assert_buckets(self, metrics):
        self.assertEqual(metrics["episodes"], 4)
        self.assertEqual(metrics["valid_episodes"], 3)
        self.assertAlmostEqual(metrics["success_rate"], 0.25)
        self.assertAlmostEqual(metrics["collision_rate"], 0.25)
        self.assertAlmostEqual(metrics["timeout_rate"], 0.25)
        self.assertAlmostEqual(metrics["shield_infeasible_rate"], 0.25)
        self.assertAlmostEqual(metrics["reset_failure_rate"], 0.25)
        self.assertIn("trajectory_complete_rate", metrics)
        # 各桶按 done_reason 互不重叠，且不超过总回合数
        bucket_sum = (metrics["collision_rate"] + metrics["timeout_rate"] +
                      metrics["shield_infeasible_rate"] +
                      metrics["reset_failure_rate"])
        self.assertLessEqual(bucket_sum, 1.0)
        self.assertEqual(
            metrics["termination_reasons"].get("shield_infeasible"), 1)
        self.assertEqual(metrics["termination_reasons"].get("collision"), 1)

    def test_train_evaluation_counts_shield_and_reset_buckets(self):
        metrics = train_dqn.evaluate(
            self._agent(), _ScriptedEnv(scripted_episodes()), 4, 1)
        self._assert_buckets(metrics)

    def test_policy_evaluation_counts_shield_and_reset_buckets(self):
        metrics = evaluate_policy.evaluate(
            self._agent(), _ScriptedEnv(scripted_episodes()), 4, 1)
        self._assert_buckets(metrics)

    def test_evaluation_log_fields_cover_new_columns(self):
        metrics = train_dqn.evaluate(
            self._agent(), _ScriptedEnv(scripted_episodes()), 4, 1)
        rolling = rolling_evaluation_metrics([metrics], 1)
        row = {"evaluation": 1, "after_episode": 1, **metrics, **rolling}
        self.assertLessEqual(set(row), set(train_dqn.EVAL_FIELDS))


if __name__ == "__main__":
    unittest.main()
