#!/usr/bin/env python3

import csv
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path


STAGE2 = Path(__file__).resolve().parents[3] / "experiments" / "stage2"
sys.path.insert(0, str(STAGE2))

from index_historical_runs import build_index


class Stage2HistoryIndexTest(unittest.TestCase):
    def test_legacy_logs_are_explicitly_exploratory(self):
        root = Path(tempfile.mkdtemp(prefix="stage2-history-"))
        try:
            log = root / "l3" / "seed2" / "training_log.csv"
            log.parent.mkdir(parents=True)
            with log.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["episode", "done_reason"])
                writer.writeheader()
                writer.writerow({"episode": 0, "done_reason": "reset_failed"})
            records = build_index([root])
            self.assertEqual(len(records), 1)
            record = records[0]
            self.assertTrue(record["exploratory"])
            self.assertFalse(record["formal_protocol_eligible"])
            self.assertEqual(record["layout"], 3)
            self.assertEqual(record["seed"], 2)
            self.assertEqual(record["summary"]["episodes"], 1)
            self.assertEqual(len(record["sha256"]), 64)
        finally:
            shutil.rmtree(root)


if __name__ == "__main__":
    unittest.main()
