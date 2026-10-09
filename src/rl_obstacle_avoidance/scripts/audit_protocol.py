#!/usr/bin/env python3
"""Audit the frozen observation/action interface without starting ROS.

The audit intentionally reads source text only.  This makes it usable in CI and
prevents a ROS parameter environment from silently changing the protocol.
"""

import argparse
import ast
import json
from pathlib import Path


def _literal_assignment(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name) and target.id == name:
                return ast.literal_eval(node.value)
    raise ValueError("missing literal assignment: %s" % name)


def audit(source_path):
    source_path = Path(source_path)
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(source_path))
    n_actions = int(_literal_assignment(tree, "N_ACTIONS"))
    path_features = int(_literal_assignment(tree, "PATH_FEATURES"))

    observation_dim = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef) or node.name != "observation_space_dim":
            continue
        for child in ast.walk(node):
            if isinstance(child, ast.Return):
                try:
                    observation_dim = int(eval(  # noqa: S307 - fixed AST expression
                        compile(ast.Expression(child.value), str(source_path), "eval"),
                        {"__builtins__": {}},
                        {"self": type("Env", (), {"num_sectors": 12})(),
                         "N_ACTIONS": n_actions,
                         "PATH_FEATURES": path_features}))
                except Exception as exc:  # pragma: no cover - diagnostic path
                    raise ValueError("cannot evaluate observation_space_dim") from exc
                break
        break
    if observation_dim is None:
        raise ValueError("missing observation_space_dim property")

    result = {
        "source": str(source_path),
        "n_actions": n_actions,
        "observation_dim": observation_dim,
        "expected_n_actions": 6,
        "expected_observation_dim": 38,
        "pass": n_actions == 6 and observation_dim == 38,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", nargs="?", default="src/rl_obstacle_avoidance/scripts/gazebo_env.py")
    parser.add_argument("--output", help="write the JSON result to this path")
    args = parser.parse_args()
    result = audit(args.source)
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    print(text, end="")
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
