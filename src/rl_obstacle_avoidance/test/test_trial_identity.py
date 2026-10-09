#!/usr/bin/env python3

import hashlib
import json
import unittest


def scenario_id(payload):
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return "s-" + hashlib.sha1(encoded.encode("utf-8")).hexdigest()[:10]


class TrialIdentityTest(unittest.TestCase):
    def test_same_scenario_has_stable_id(self):
        payload = {"obstacles": [[3.0, 0.0]], "trajectory": "linear",
                   "altitude": 5.0}
        self.assertEqual(scenario_id(payload), scenario_id(dict(payload)))

    def test_scenario_change_changes_id(self):
        a = {"obstacles": [[3.0, 0.0]], "trajectory": "linear"}
        b = {"obstacles": [[4.0, 0.0]], "trajectory": "linear"}
        self.assertNotEqual(scenario_id(a), scenario_id(b))


if __name__ == "__main__":
    unittest.main()
