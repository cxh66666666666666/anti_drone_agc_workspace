#!/usr/bin/env python3
"""d1_batch_runner.py —— 阶段3 D1「可续跑批跑器」（离线编排工具）。

职责：读取 D0 冻结的 manifest（experiments/d/manifest_{split}_v1.json），逐 entry 调用编排脚本
scripts/start_agc_collab.sh，为「每个场景」跑一个全新的「启动→退出」全循环（--cycles 1 --repeat 1），
把每次尝试与每条 run 追加成可审计的 jsonl，并按 eval_protocol_v1 的三类分母核对恒等式。

特性：
  - 断点续跑：状态文件记录已完成 scenario_id，重启后自动跳过，不重跑；
  - 重复种子检测：待跑集合内 seed/scenario_id 重复即报错退出；跨 split 重复打印 PASS/FAIL；
  - 基础设施失败重试：无 batch_dir / 无 attempt 记录 / final_state==8 时按 --retries 重试，
    每次尝试都追加到 attempts 日志，原始失败不被覆盖或静默丢弃；
  - 哈希一致性：entry 的 scenario_hash/world_hash 与 run meta.json 对比，不一致记 hash_mismatch；
  - 汇总：从 index 计算三类分母并核对两条恒等式，写 experiments/d/batches.json（按 batch name 累加）。

本脚本是离线工具（非实时链路节点），按项目约定用 Python 实现；不改动 D0 冻结的节点/launch/编排脚本。

用法：
    python3 src/air_ground_core/scripts/d1_batch_runner.py \
        --manifest experiments/d/manifest_test_v1.json --split test \
        --families nominal --limit 1 [--dry-run] [--headless] [--retries 2]

hot-swap 常驻热切换（一次启动 Gazebo 连跑整组，不每回合重启栈）：
    python3 src/air_ground_core/scripts/d1_batch_runner.py \
        --manifest experiments/d/manifest_test_v1.json --split test \
        --hot-swap [--dry-run] [--serve-ready-timeout 300]
"""

import argparse
import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
import time

try:
    import yaml  # 读取场景文件 obstacles[].name（pyyaml）
except ImportError:  # 缺失时降级：场景障碍名解析返回空，不崩溃
    yaml = None

# ---------------------------------------------------------------- 路径与常量
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 本脚本位于 <workspace>/src/air_ground_core/scripts
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
START_SCRIPT = os.path.join(WORKSPACE, "scripts", "start_agc_collab.sh")
EXP_D = os.path.join(WORKSPACE, "experiments", "d")

# 单 entry 子进程超时 = run_timeout + 启动余量（清理+启动+起飞+交接+等待终态+收尾）
STARTUP_MARGIN_S = 240

STATE_NAME = {
    0: "IDLE", 1: "AIR_SEARCH", 2: "TARGET_ACQUIRED", 3: "GROUND_APPROACH",
    4: "TRACKING_SUCCESS", 5: "MISSION_FAILED", 6: "ABORTED",
    7: "INTERCEPT_CONFIRMED", 8: "INFRA_FAILURE",
}
CATEGORY_BY_STATE = {4: "tracking_success", 5: "mission_failure", 6: "aborted", 8: "infra_failure"}
CATEGORIES = ("tracking_success", "mission_failure", "aborted", "infra_failure")
SPLITS = ("test", "train", "validation")
TERMINAL_STATES = (4, 5, 6, 8)  # 终态集合（与编排脚本一致）
# hot-swap 常驻栈默认世界：无固定障碍、障碍全部动态 spawn 的世界（包内）
HOT_SWAP_WORLD = "src/air_ground_core/worlds/agc_world_dynamic.world"


def p(*args):
    """带 flush 的打印，保证长任务时子进程输出不丢。"""
    print(*args, flush=True)


def now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# ---------------------------------------------------------------- 读写工具
def load_json(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def parse_jsonl(path):
    out = []
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    out.append(json.loads(line))
                except ValueError:
                    continue
    except OSError:
        pass
    return out


def append_jsonl(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, ensure_ascii=False) + "\n")


def write_json_atomic(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def _hash_file(algo, path):
    try:
        h = hashlib.new(algo)
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()
    except (OSError, ValueError):
        return None


def sha256_file(path):
    return _hash_file("sha256", path)


def sha1_file(path):
    return _hash_file("sha1", path)


def abs_in_ws(rel):
    """把 manifest 里「相对 workspace」的路径转成绝对路径。"""
    return rel if os.path.isabs(rel) else os.path.join(WORKSPACE, rel)


# ---------------------------------------------------------------- 名称与筛选
def derive_name(manifest_path):
    """默认文件名用的 name：从 manifest 文件名派生（manifest_test_v1.json -> test_v1）。"""
    base = os.path.basename(manifest_path)
    if base.startswith("manifest_"):
        base = base[len("manifest_"):]
    if base.endswith(".json"):
        base = base[:-len(".json")]
    return base or "manifest"


def guess_split(name):
    for s in SPLITS:
        if name.startswith(s):
            return s
    return "all"


def select_entries(entries, split, families):
    sel = list(entries)
    if split:
        sel = [e for e in sel if e.get("split") == split]
    if families:
        sel = [e for e in sel if e.get("family") in families]
    return sel


def check_within_set_duplicates(sel):
    """待跑集合内 seed/scenario_id 重复检测。

    配对臂例外：同一 seed 的若干条目，若其 pair_id 均非空且相同，视为合法配对（不算重复）。
    返回 (dup_seeds, dup_ids, legal_pair_seeds, pair_id_entries)。
    """
    from collections import Counter, defaultdict
    by_seed = defaultdict(list)
    for e in sel:
        by_seed[e.get("seed")].append(e)
    dup_seeds, legal_pair_seeds = [], []
    for seed, group in by_seed.items():
        if len(group) <= 1:
            continue
        pids = {e.get("pair_id") for e in group}
        # 合法配对：组内 pair_id 唯一且非空（baseline + _fus 共享同 seed/pair_id）
        if len(pids) == 1 and next(iter(pids)):
            legal_pair_seeds.append(seed)
        else:
            dup_seeds.append(seed)
    ids = Counter(e.get("scenario_id") for e in sel)
    dup_ids = sorted(k for k, v in ids.items() if v > 1)
    pair_id_entries = sum(1 for e in sel if e.get("pair_id"))
    return sorted(dup_seeds), dup_ids, sorted(legal_pair_seeds), pair_id_entries


def check_cross_split_duplicates(entries):
    """跨 split 的 seed/scenario_id 泄漏检测；返回 (leak_seeds, leak_ids)。"""
    from collections import defaultdict
    seed_splits = defaultdict(set)
    id_splits = defaultdict(set)
    for e in entries:
        seed_splits[e.get("seed")].add(e.get("split"))
        id_splits[e.get("scenario_id")].add(e.get("split"))
    leak_seeds = sorted(k for k, v in seed_splits.items() if len(v) > 1)
    leak_ids = sorted(k for k, v in id_splits.items() if len(v) > 1)
    return leak_seeds, leak_ids


# ---------------------------------------------------------------- 命令构造
def build_cmd(entry, args, batch_prefix):
    """按 entry 构造 start_agc_collab.sh 命令（每个场景一个全新栈：--cycles 1 --repeat 1）。"""
    cmd = [
        START_SCRIPT,
        "--cycles", "1", "--repeat", "1",
        "--run-timeout", str(args.run_timeout),
        "--batch-prefix", batch_prefix,
        "--scenario", abs_in_ws(entry["scenario_file"]),
        "--world-file", abs_in_ws(entry["world_file"]),
        "--sensors-file", abs_in_ws(entry["sensors_file"]),
        "--scene-seed", str(entry["seed"]),
        "--target-source", str(entry.get("target_source", "gt_adapter")),
        "--obstacle-source", str(entry.get("obstacle_source", "scenario")),
    ]
    if args.gui:
        cmd.append("--gui")
    # inject 非空时透传（mode 为 "randomized_params" 时属声明性元数据，编排脚本按容错处理）
    inj = entry.get("inject")
    if inj:
        mode = inj.get("mode")
        if mode:
            cmd += ["--inject", str(mode)]
            if inj.get("sensor"):
                cmd += ["--inject-sensor", str(inj["sensor"])]
            if inj.get("value") is not None:
                cmd += ["--inject-value", str(inj["value"])]
            if inj.get("delay_s") is not None:
                cmd += ["--inject-delay", str(inj["delay_s"])]
    return cmd


# ---------------------------------------------------------------- 单场景执行
def run_one_entry(entry, args, batch_prefix, log_path):
    """执行一个场景一次尝试；返回 attempt dict（含 batch_dir/run_dir/终态/异常信息）。"""
    cmd = build_cmd(entry, args, batch_prefix)
    p("[运行] %s 命令: %s" % (entry["scenario_id"], shlex.join(cmd)))
    env = dict(os.environ)
    if args.gui:
        env.setdefault("DISPLAY", ":0")
    timeout_s = args.run_timeout + STARTUP_MARGIN_S
    out, rc, timed_out = "", None, False
    try:
        proc = subprocess.run(cmd, cwd=WORKSPACE, env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=timeout_s)
        out = (proc.stdout or b"").decode("utf-8", "ignore")
        rc = proc.returncode
    except subprocess.TimeoutExpired as ex:
        out = (ex.stdout or b"").decode("utf-8", "ignore") if ex.stdout else ""
        timed_out = True
    except OSError as ex:
        out = "启动失败: %s" % ex
    # 落盘原始日志，便于核对命令与报错
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    with open(log_path, "w", encoding="utf-8", errors="ignore") as f:
        f.write(out)

    batch_dir = None
    m = re.search(r"batch_dir=(\S+)", out)
    if m:
        batch_dir = m.group(1)

    attempt_row, run_dir = None, ""
    batch_json = None
    if batch_dir:
        recs = parse_jsonl(os.path.join(batch_dir, "records.jsonl"))
        for r in recs:
            if r.get("type") == "attempt":
                attempt_row = r
                break
        if attempt_row:
            run_dir = attempt_row.get("run_dir", "") or ""
        batch_json = load_json(os.path.join(batch_dir, "batch.json"))

    final_state = attempt_row.get("final_state") if attempt_row else None
    reason = (attempt_row.get("reason", "") if attempt_row else "") or ""
    infra = (batch_dir is None) or (attempt_row is None) or (final_state == 8)
    return {
        "scenario_id": entry["scenario_id"], "split": entry["split"], "family": entry["family"],
        "seed": entry["seed"], "cmd": cmd, "returncode": rc, "timed_out": timed_out,
        "batch_dir": batch_dir, "run_dir": run_dir,
        "final_state": final_state, "final_state_name": STATE_NAME.get(final_state),
        "category": CATEGORY_BY_STATE.get(final_state),
        "reason": reason,
        "reset_ok": attempt_row.get("reset_ok") if attempt_row else None,
        "start_ok": attempt_row.get("start_ok") if attempt_row else None,
        "inject_note": attempt_row.get("inject", "none") if attempt_row else None,
        "inject_declared": entry.get("inject"),
        "batch_any_leftover": (batch_json or {}).get("any_leftover"),
        "infra": infra,
        "error": None if attempt_row else ("timeout" if timed_out else "no_attempt_record"),
        "log_path": log_path,
    }


# ---------------------------------------------------------------- hot-swap 常驻热切换
# 设计：一次启动常驻栈（start_agc_collab.sh --serve），组内逐回合只做
#   set 参数 -> scenario_reset -> 等 odom -> 取 run_dir -> start -> 等终态 -> 记录
# 从而不再每回合重启整套栈。分组依据 (target_source, obstacle_source)：
# 这两者决定 launch 时启动哪些节点，且 /agc/targets 只能有一个发布者，不能混跑。
def ros_env():
    """ROS 命令统一环境：ROS_LOG_DIR=/tmp/ros_logs、DISPLAY=:0。"""
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    env["DISPLAY"] = ":0"
    try:
        os.makedirs("/tmp/ros_logs", exist_ok=True)
    except OSError:
        pass
    return env


def run_ros(inner, timeout=30):
    """在 workspace 下以 bash -lc 执行（先 source ROS/devel）；返回 (rc, 合并输出)。"""
    script = "source /opt/ros/noetic/setup.bash && source devel/setup.bash && " + inner
    try:
        proc = subprocess.run(["bash", "-lc", script], cwd=WORKSPACE, env=ros_env(),
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        return proc.returncode, (proc.stdout or b"").decode("utf-8", "ignore")
    except subprocess.TimeoutExpired as ex:
        out = (ex.stdout or b"").decode("utf-8", "ignore") if ex.stdout else ""
        return 124, out
    except OSError as ex:
        return 1, "启动失败: %s" % ex


def rosparam_set(name, value):
    return run_ros("rosparam set %s %s" % (name, shlex.quote(str(value))), timeout=15)


def rosparam_get(name):
    rc, out = run_ros("rosparam get %s" % name, timeout=15)
    return out.strip() if rc == 0 else ""


def _parse_field(out, field):
    """从 rostopic/rosparam 的 YAML 输出里取字段；字符串值去掉引号。"""
    m = re.search(r"^%s:\s*(.*)$" % field, out, re.MULTILINE)
    return m.group(1).strip().strip("'\"") if m else None


def wait_terminal_ros(timeout_s):
    """轮询 /agc/task/state 到终态；返回 (state:int|None, reason:str)。"""
    t, state, reason = 0, None, ""
    while t < timeout_s:
        _rc, out = run_ros("timeout 6 rostopic echo /agc/task/state -n 1", timeout=12)
        st = _parse_field(out, "state")
        if st is not None:
            try:
                state = int(st)
            except ValueError:
                state = None
            reason = _parse_field(out, "reason_code") or reason
        if state in TERMINAL_STATES:
            return state, reason
        time.sleep(3)
        t += 3
    return state, reason


def group_entries(entries):
    """按 (target_source, obstacle_source) 分组，保持首次出现顺序。"""
    groups = {}
    for e in entries:
        key = (str(e.get("target_source", "gt_adapter")), str(e.get("obstacle_source", "scenario")))
        groups.setdefault(key, []).append(e)
    return groups


def resolve_hot_swap_world(args):
    """常驻栈使用的世界文件：默认包内无固定障碍的动态世界。"""
    if args.hot_swap_world:
        return abs_in_ws(args.hot_swap_world)
    return os.path.join(WORKSPACE, HOT_SWAP_WORLD)


def build_serve_cmd(gkey, args, batch_prefix, world_file):
    """构造常驻栈启动命令（--serve --dynamic-obstacles）。"""
    tgt, obs = gkey
    cmd = [START_SCRIPT, "--serve",
           "--run-timeout", str(args.run_timeout),
           "--batch-prefix", batch_prefix,
           "--world-file", world_file,
           "--target-source", str(tgt), "--obstacle-source", str(obs),
           "--dynamic-obstacles"]
    if args.gui:
        cmd.append("--gui")
    return cmd


def print_hot_swap_dry_run(to_run, args, batch_prefix):
    """dry-run：打印分组、每组场景数、常驻栈命令与每回合的 set/reset/start 命令。"""
    world = resolve_hot_swap_world(args)
    groups = group_entries(to_run)
    p("[hot-swap][dry-run] 分组依据 (target_source, obstacle_source)；每组一个常驻栈，")
    p("[hot-swap][dry-run] 组内逐回合只 set 参数 + 复位 + start，不再重启整套栈。")
    p("[hot-swap][dry-run] 共 %d 组 / %d 个场景；世界文件=%s" % (len(groups), len(to_run), world))
    for gkey, group in groups.items():
        p("-" * 64)
        p("[组 target_source=%s obstacle_source=%s] 场景数=%d"
          % (gkey[0], gkey[1], len(group)))
        p("  常驻栈命令: " + shlex.join(build_serve_cmd(gkey, args, batch_prefix, world)))
        for e in group:
            p("    [回合] %s" % e["scenario_id"])
            p("      rosparam set /agc/scenario_file %s" % abs_in_ws(e["scenario_file"]))
            p("      rosparam set /agc/perception/sensors_file %s" % abs_in_ws(e["sensors_file"]))
            # seed 以 YAML 字符串写入（带引号），避免被解析成 int
            p("      " + shlex.join(["rosparam", "set", "/agc/log/seed", yaml_string_seed(e["seed"])]))
            p("      rosservice call /agc/scenario/reset  # 等 /agc/reset/last_result==SUCCESS 与 odom_valid")
            p("      rosservice call /gazebo/get_world_properties  # 记 sim_models 并核对障碍物理生效")
            p("      rosservice call /agc/task/start      # 先取 /agc/log/run_dir 再 start，等终态")
    p("[hot-swap][dry-run] 注：每组仅在组首尾各启动/退出一次栈（共 %d 次），"
      "而非每回合一次。" % len(groups))


def yaml_string_seed(seed):
    """把 seed 写成 YAML 字符串（带双引号），避免 rosparam 按 int 解析。
    agc_run_logger 用 pgetStr 读取，类型不符会回落默认(unseeded)。"""
    return '"%s"' % seed


def parse_model_names(out):
    """解析 /gazebo/get_world_properties 输出里的 model_names 列表；解析不出返回 None。"""
    m = re.search(r"model_names:\s*\[(.*?)\]", out, re.DOTALL)
    if m:
        return [x.strip().strip("'\"") for x in m.group(1).split(",") if x.strip()]
    m2 = re.search(r"model_names:\s*\n((?:[ \t]*-[ \t]*.+\n?)+)", out)
    if m2:
        return [ln.strip().lstrip("-").strip().strip("'\"")
                for ln in m2.group(1).splitlines() if ln.strip()]
    return None


def query_sim_models():
    """查询 Gazebo 实际加载的模型名列表；服务失败返回 None（不崩溃）。"""
    rc, out = run_ros("rosservice call /gazebo/get_world_properties", timeout=20)
    if rc != 0 or not out:
        return None
    return parse_model_names(out)


def scenario_obstacle_names(entry):
    """读取场景文件 obstacles[].name 列表；解析失败返回 []。"""
    if yaml is None:
        return []
    try:
        with open(abs_in_ws(entry["scenario_file"]), "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
    except (OSError, ValueError, yaml.YAMLError):
        return []
    obs = doc.get("obstacles") or []
    return [str(o["name"]) for o in obs if isinstance(o, dict) and o.get("name")]


def run_hot_swap_entry(entry, args, batch_dir):
    """组内单回合：set 参数 -> reset -> 等 odom -> 查障碍 -> 取 run_dir -> start -> 等终态。"""
    rosparam_set("/agc/scenario_file", abs_in_ws(entry["scenario_file"]))
    rosparam_set("/agc/perception/sensors_file", abs_in_ws(entry["sensors_file"]))
    # seed 以 YAML 字符串写入（"12345"），否则被解析成 int，logger pgetStr 读取失败
    rosparam_set("/agc/log/seed", yaml_string_seed(entry["seed"]))

    # 复位场景（含动态障碍增删/车目标归位），等 reset 成功
    run_ros("rosservice call /agc/scenario/reset", timeout=20)
    reset_ok = False
    for _ in range(20):
        if rosparam_get("/agc/reset/last_result") == "SUCCESS":
            reset_ok = True
            break
        time.sleep(1)

    # 等空地链路恢复
    for _ in range(25):
        av = rosparam_get("/agc/air/odom_valid")
        gv = rosparam_get("/agc/ground/odom_valid")
        if av in ("True", "true") and gv in ("True", "true"):
            break
        time.sleep(1)

    # 障碍物理生效核查：reset 成功后、start 前，查 Gazebo 实际模型名并与场景期望比对
    sim_models = query_sim_models() if reset_ok else None
    exp_names = scenario_obstacle_names(entry)
    if sim_models is not None:
        present = sum(1 for n in exp_names if n in sim_models)
        obs_present = {"present": present, "total": len(exp_names)}
        obs_ok = (present == len(exp_names))
        sim_models_note = None
    else:
        obs_present, obs_ok = None, None
        sim_models_note = "查询 /gazebo/get_world_properties 失败，无法核对障碍物理生效"

    # run_dir 必须在 start 之前取（start 后 logger 可能立刻新开目录）
    run_dir = rosparam_get("/agc/log/run_dir")
    _rc, out = run_ros("rosservice call /agc/task/start", timeout=20)
    start_ok = "AIR_SEARCH" in out

    if start_ok:
        final_state, reason = wait_terminal_ros(args.run_timeout)
    else:
        _rc2, out2 = run_ros("timeout 6 rostopic echo /agc/task/state -n 1", timeout=12)
        st = _parse_field(out2, "state")
        try:
            final_state = int(st) if st is not None else None
        except ValueError:
            final_state = None
        reason = _parse_field(out2, "reason_code") or ""

    infra = (not reset_ok) or (not start_ok) or (final_state == 8) or (not run_dir)
    return {
        "scenario_id": entry["scenario_id"], "split": entry["split"], "family": entry["family"],
        "seed": entry["seed"], "batch_dir": batch_dir, "run_dir": run_dir or "",
        "final_state": final_state, "final_state_name": STATE_NAME.get(final_state),
        "category": CATEGORY_BY_STATE.get(final_state),
        "reason": reason or "", "reset_ok": reset_ok, "start_ok": start_ok,
        "inject_note": "none", "infra": infra,
        "sim_models": sim_models, "scenario_obstacles_present": obs_present,
        "scenario_obstacles_ok": obs_ok, "sim_models_note": sim_models_note,
    }


def start_serve_group(gkey, args, batch_prefix, world, log_path):
    """后台启动常驻栈，stdout/stderr 写日志文件；返回 (proc, logfile)。"""
    cmd = build_serve_cmd(gkey, args, batch_prefix, world)
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    logf = open(log_path, "w", encoding="utf-8", errors="ignore")
    p("[hot-swap] 启动常驻栈: %s" % shlex.join(cmd))
    proc = subprocess.Popen(cmd, cwd=WORKSPACE, env=ros_env(),
                            stdout=logf, stderr=subprocess.STDOUT)
    return proc, logf


def wait_serve_ready(proc, log_path, timeout_s):
    """轮询日志出现 batch_dir= 与 serve_ready=true；进程提前退出或超时返回 None。"""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if proc.poll() is not None:
            p("[hot-swap] 常驻栈提前退出 (rc=%s)，见日志 %s" % (proc.returncode, log_path))
            return None
        try:
            with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
                txt = f.read()
        except OSError:
            txt = ""
        m = re.search(r"batch_dir=(\S+)", txt)
        if m and "serve_ready=true" in txt:
            return m.group(1)
        time.sleep(3)
    p("[hot-swap] 等待 serve_ready 超时 (%ds)，见日志 %s" % (timeout_s, log_path))
    return None


def run_hot_swap(to_run, args, name, split, families, batch_prefix, manifest_path,
                 out_index, state_path, attempts_path, state, completed, serve_log_dir):
    """hot-swap 主流程：分组 -> 每组一次常驻栈 -> 组内逐回合热切换。"""
    world = resolve_hot_swap_world(args)
    groups = group_entries(to_run)
    p("[hot-swap] 共 %d 组 / %d 个场景；常驻世界=%s" % (len(groups), len(to_run), world))
    for gi, (gkey, group) in enumerate(groups.items(), 1):
        p("=" * 64)
        p("[hot-swap] 组 %d/%d: target_source=%s obstacle_source=%s 场景数=%d"
          % (gi, len(groups), gkey[0], gkey[1], len(group)))
        log_path = os.path.join(serve_log_dir, "serve_%s_%s.log" % (gkey[0], gkey[1]))
        proc, logf = start_serve_group(gkey, args, batch_prefix, world, log_path)
        batch_dir = wait_serve_ready(proc, log_path, args.serve_ready_timeout)
        if not batch_dir:
            p("[hot-swap] 组启动失败，终止常驻进程并跳过该组")
            try:
                proc.terminate()
                proc.wait(timeout=30)
            except (subprocess.TimeoutExpired, OSError):
                proc.kill()
            logf.close()
            continue
        p("[hot-swap] 组就绪 batch_dir=%s" % batch_dir)
        try:
            for i, entry in enumerate(group):
                sid = entry["scenario_id"]
                p("-" * 64)
                p("[hot-swap] (%d/%d) 回合 %s" % (i + 1, len(group), sid))
                accepted, k = None, 0
                for k in range(1, args.retries + 2):
                    att = run_hot_swap_entry(entry, args, batch_dir)
                    append_jsonl(attempts_path, {
                        "batch_name": name, "attempt_index": k, "mode": "hot_swap", "ts": now_iso(),
                        **{f: att[f] for f in
                           ("scenario_id", "split", "family", "seed", "batch_dir", "run_dir",
                            "final_state", "final_state_name", "category", "reason",
                            "reset_ok", "start_ok", "inject_note", "infra")},
                    })
                    p("[hot-swap][尝试] %s att%d -> run_dir=%s final_state=%s reason=%s infra=%s"
                      % (sid, k, att["run_dir"], att["final_state"], att["reason"], att["infra"]))
                    if att["run_dir"]:
                        append_jsonl(out_index, build_index_row(entry, att, k, name, hot_swap_world=world))
                    if att["infra"] and k <= args.retries:
                        p("[hot-swap][重试] %s att%d 基础设施失败，进行第 %d 次重试" % (sid, k, k + 1))
                        continue
                    accepted = att
                    break
                meta = (load_json(os.path.join(accepted["run_dir"], "meta.json"))
                        if accepted.get("run_dir") else None)
                completed[sid] = {
                    "run_id": (meta or {}).get("run_id"), "final_state": accepted["final_state"],
                    "category": accepted["category"], "reason": accepted["reason"],
                    "attempts": k, "finished_at": now_iso(),
                }
                state["name"] = name
                state["manifest"] = manifest_path
                state["split"] = split
                state["families"] = sorted(families)
                state["mode"] = "hot_swap"
                state["updated_at"] = now_iso()
                state["completed"] = completed
                write_json_atomic(state_path, state)
        finally:
            # 组结束：写停止信号，等常驻栈干净退出并核对残留
            stop_file = os.path.join(batch_dir, "serve_stop")
            try:
                with open(stop_file, "w", encoding="utf-8") as f:
                    f.write("stop\n")
                p("[hot-swap] 已写停止信号 %s，等待常驻栈退出..." % stop_file)
            except OSError as ex:
                p("[hot-swap] 写停止信号失败: %s" % ex)
            try:
                proc.wait(timeout=180)
            except subprocess.TimeoutExpired:
                p("[hot-swap] 常驻栈未在 180s 内退出，发送 SIGTERM")
                proc.terminate()
                try:
                    proc.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    proc.kill()
            logf.close()
            batch_json = load_json(os.path.join(batch_dir, "batch.json"))
            p("[hot-swap] 组退出 rc=%s any_leftover=%s"
              % (proc.returncode, (batch_json or {}).get("any_leftover")))
    return state, completed


# ---------------------------------------------------------------- 哈希校验
def build_hash_check(entry, meta, hot_swap_world=None):
    """对比 manifest entry 与 run meta.json 的哈希。

    非热切换（原逐回合路径，行为不变）：冻结的 agc_run_logger 用 sha1 对实际 scenario 文件取哈希，
    manifest 用 sha256；meta.world_hash 固定取包内模板 worlds/agc_world.world。故做「算法无关的一致性」
    判定并显式记录 note，绝不静默忽略。

    热切换模式（hot_swap_world 非空）：常驻加载的是基础动态世界（无固定障碍，障碍由 scenario_reset
    动态 spawn），因此 meta.world_hash 应对照该基础世界文件；场景一致性以 scenario_hash 为准（必须匹配）。
    """
    scf = abs_in_ws(entry["scenario_file"])
    sc_sha256, sc_sha1 = sha256_file(scf), sha1_file(scf)
    msc = (meta or {}).get("scenario_hash")
    mwl = (meta or {}).get("world_hash")

    if hot_swap_world is not None:
        # 期望世界哈希取基础动态世界文件（非每个场景的 world_file）
        bw_sha256, bw_sha1 = sha256_file(hot_swap_world), sha1_file(hot_swap_world)
        sc_ok = msc is not None and msc in {entry.get("scenario_hash"), sc_sha256, sc_sha1}
        wl_ok = mwl is not None and mwl in {bw_sha256, bw_sha1}
        note = "热切换模式：世界为基础动态世界，场景障碍由动态 spawn，场景一致性以 scenario_hash 为准"
        if meta is None:
            note = "meta.json 缺失，无法核对哈希（" + note + "）"
        elif not sc_ok:
            note = "scenario_hash 与 manifest 声明文件不一致（可能 run 用了不同场景文件）"
        elif not wl_ok:
            note = "world_hash 与基础动态世界不一致（" + note + "）"
        return {
            "mode": "hot_swap",
            "manifest_scenario_hash": entry.get("scenario_hash"),
            "meta_scenario_hash": msc,
            "scenario_file_sha256": sc_sha256,
            "scenario_file_sha1": sc_sha1,
            "hot_swap_world": hot_swap_world,
            "expected_world_sha1": bw_sha1,
            "expected_world_sha256": bw_sha256,
            "manifest_world_hash": entry.get("world_hash"),
            "meta_world_hash": mwl,
            "scenario_hash_match": sc_ok,
            "world_hash_match": wl_ok,
            "raw_scenario_hash_equal": (msc == entry.get("scenario_hash")),
            "raw_world_hash_equal": (mwl == bw_sha1),
            "hash_mismatch": not (sc_ok and wl_ok),
            "note": note,
        }

    # ---- 非热切换：原行为保持不变 ----
    wlf = abs_in_ws(entry["world_file"])
    wl_sha256, wl_sha1 = sha256_file(wlf), sha1_file(wlf)
    sc_ok = msc is not None and msc in {entry.get("scenario_hash"), sc_sha256, sc_sha1}
    wl_ok = mwl is not None and mwl in {entry.get("world_hash"), wl_sha256, wl_sha1}
    raw_sc = (msc == entry.get("scenario_hash"))
    raw_wl = (mwl == entry.get("world_hash"))

    note = None
    if meta is None:
        note = "meta.json 缺失，无法核对哈希"
    elif not sc_ok:
        note = "scenario_hash 与 manifest 声明文件不一致（可能 run 用了不同场景文件）"
    elif not wl_ok:
        note = "world_hash 指向包内模板 agc_world.world，而非场景 world 文件"
    elif not raw_sc:
        note = "scenario_hash 对应同一文件但算法不同（meta=sha1 vs manifest=sha256）"

    return {
        "manifest_scenario_hash": entry.get("scenario_hash"),
        "meta_scenario_hash": msc,
        "scenario_file_sha256": sc_sha256,
        "scenario_file_sha1": sc_sha1,
        "manifest_world_hash": entry.get("world_hash"),
        "meta_world_hash": mwl,
        "world_file_sha256": wl_sha256,
        "world_file_sha1": wl_sha1,
        "scenario_hash_match": sc_ok,
        "world_hash_match": wl_ok,
        "raw_scenario_hash_equal": raw_sc,
        "raw_world_hash_equal": raw_wl,
        "hash_mismatch": not (sc_ok and wl_ok),
        "note": note,
    }


def build_index_row(entry, att, attempt_index, batch_name, hot_swap_world=None):
    """把一次尝试汇总成 index 行（含 meta 与哈希一致性）。
    hot_swap_world 非空 = 热切换模式：world_hash 对照基础动态世界，并追加 sim_models 等字段。"""
    run_dir = att["run_dir"]
    meta = load_json(os.path.join(run_dir, "meta.json")) if run_dir else None
    hc = build_hash_check(entry, meta, hot_swap_world=hot_swap_world)
    row = {
        "batch_name": batch_name,
        "scenario_id": entry["scenario_id"], "split": entry["split"], "family": entry["family"],
        "seed": entry["seed"], "attempt_index": attempt_index,
        "batch_dir": att["batch_dir"], "run_dir": run_dir,
        "final_state": att["final_state"], "final_state_name": att["final_state_name"],
        "category": att["category"], "reason": att["reason"],
        "reset_ok": att["reset_ok"], "start_ok": att["start_ok"],
        "inject": entry.get("inject"), "inject_note": att["inject_note"],
        "meta": ({
            "run_id": meta.get("run_id"), "scenario_hash": meta.get("scenario_hash"),
            "world_hash": meta.get("world_hash"), "agc_params_hash": meta.get("agc_params_hash"),
            "scripts_hash": meta.get("scripts_hash"), "seed": meta.get("seed"),
            "target_source": meta.get("target_source"), "obstacle_source": meta.get("obstacle_source"),
        } if meta else None),
        "hash_mismatch": hc["hash_mismatch"], "hash_check": hc,
        "ts": now_iso(),
    }
    if hot_swap_world is not None:
        # 障碍物理生效审计（仅热切换）
        row["sim_models"] = att.get("sim_models")
        row["scenario_obstacles_present"] = att.get("scenario_obstacles_present")
        row["scenario_obstacles_ok"] = att.get("scenario_obstacles_ok")
        row["sim_models_note"] = att.get("sim_models_note")
    return row


# ---------------------------------------------------------------- 汇总
def summarize_index(rows, batch_name, meta_info):
    """从 index 行计算三类分母并核对恒等式。"""
    counts = {c: 0 for c in CATEGORIES}
    unknown = 0
    per_family = {}
    per_scenario = {}
    for r in rows:
        c = r.get("category")
        if c in counts:
            counts[c] += 1
        else:
            unknown += 1
        fam = per_family.setdefault(r.get("family"), {k: 0 for k in CATEGORIES})
        if c in fam:
            fam[c] += 1
        ps = per_scenario.setdefault(r.get("scenario_id"), {"runs": 0, "final_state": None, "category": None})
        ps["runs"] += 1
        ps["final_state"] = r.get("final_state")
        ps["category"] = c
    attempted = len(rows)
    valid_task = counts["tracking_success"] + counts["mission_failure"]
    eq_valid = (valid_task == counts["tracking_success"] + counts["mission_failure"])
    eq_attempted = (attempted == valid_task + counts["infra_failure"] + counts["aborted"])
    hash_mismatch_runs = sum(1 for r in rows if r.get("hash_mismatch"))
    return {
        "batch_name": batch_name,
        "manifest": meta_info.get("manifest"), "split": meta_info.get("split"),
        "families": meta_info.get("families"), "index": meta_info.get("index"),
        "attempted_runs": attempted, "valid_task_runs": valid_task,
        "tracking_success_runs": counts["tracking_success"], "mission_failure_runs": counts["mission_failure"],
        "aborted_runs": counts["aborted"], "infra_failure_runs": counts["infra_failure"],
        "unknown_category_runs": unknown,
        "hash_mismatch_runs": hash_mismatch_runs,
        "equations": {
            "valid_task = tracking_success + mission_failure": "PASS" if eq_valid else "FAIL",
            "attempted = valid_task + infra_failure + aborted": "PASS" if eq_attempted else "FAIL",
        },
        "per_family": per_family, "per_scenario": per_scenario,
        "updated_at": now_iso(),
    }


def print_summary(s):
    p("=" * 64)
    p("[分母] attempted=%d  valid_task=%d  tracking_success=%d  mission_failure=%d  aborted=%d  infra_failure=%d  unknown=%d"
      % (s["attempted_runs"], s["valid_task_runs"], s["tracking_success_runs"],
         s["mission_failure_runs"], s["aborted_runs"], s["infra_failure_runs"], s["unknown_category_runs"]))
    for k, v in s["equations"].items():
        p("EQUATION %s : %s" % (k, v))
    p("hash_mismatch 行数: %d" % s["hash_mismatch_runs"])
    p("=" * 64)


# ---------------------------------------------------------------- 主流程
def summarize_and_write(name, manifest_path, split, families, out_index, batches_path):
    """从 index 计算三类分母、核对恒等式并写 batches.json（逐回合与 hot-swap 共用）。"""
    rows = [r for r in parse_jsonl(out_index) if r.get("batch_name") == name]
    s = summarize_index(rows, name, {"manifest": manifest_path, "split": split,
                                     "families": sorted(families), "index": out_index})
    print_summary(s)
    batches = load_json(batches_path) or {}
    batches[name] = s
    write_json_atomic(batches_path, batches)
    p("[汇总] 已写 %s（batch name=%s）" % (batches_path, name))
    return s


def main():
    ap = argparse.ArgumentParser(description="阶段3 D1 可续跑批跑器")
    ap.add_argument("--manifest", required=True, help="manifest json 路径")
    ap.add_argument("--name", default=None, help="批次名（默认取 manifest 名）")
    ap.add_argument("--out-index", default=None, help="index jsonl（默认 experiments/d/batch_index_<name>.jsonl）")
    ap.add_argument("--state", default=None, help="状态文件（默认 experiments/d/batch_state_<name>.json）")
    ap.add_argument("--families", default="", help="场景族过滤（逗号分隔）")
    ap.add_argument("--split", default=None, choices=list(SPLITS), help="split 过滤")
    ap.add_argument("--limit", type=int, default=None, help="最多运行 entry 数")
    ap.add_argument("--offset", type=int, default=0, help="跳过前 K 个 entry")
    ap.add_argument("--run-timeout", type=int, default=150, help="单 attempt 等待终态秒数")
    gui = ap.add_mutually_exclusive_group()
    gui.add_argument("--gui", dest="gui", action="store_true", help="启动 Gazebo GUI（默认）")
    gui.add_argument("--headless", dest="gui", action="store_false", help="无头模式")
    ap.set_defaults(gui=True)
    ap.add_argument("--retries", type=int, default=2, help="基础设施失败重试次数")
    ap.add_argument("--batch-prefix", default=None, help="批跑目录前缀（默认 _d1_<split>_）")
    ap.add_argument("--hot-swap", action="store_true",
                    help="常驻热切换：每组一次启动栈，组内逐回合只 set 参数+复位+start（默认关闭）")
    ap.add_argument("--hot-swap-world", default=None,
                    help="hot-swap 常驻栈世界文件（默认包内 worlds/agc_world_dynamic.world）")
    ap.add_argument("--serve-ready-timeout", type=int, default=300,
                    help="等待常驻栈 serve_ready 的最长秒数（默认 300）")
    ap.add_argument("--dry-run", action="store_true", help="只打印将执行的命令，不运行")
    args = ap.parse_args()

    manifest_path = os.path.abspath(args.manifest)
    manifest = load_json(manifest_path)
    if not manifest or "entries" not in manifest:
        p("[错误] 无法读取 manifest 或缺少 entries: %s" % manifest_path)
        return 2
    entries = manifest["entries"]

    name = args.name or derive_name(manifest_path)
    split = args.split or guess_split(name)
    out_index = args.out_index or os.path.join(EXP_D, "batch_index_%s.jsonl" % name)
    state_path = args.state or os.path.join(EXP_D, "batch_state_%s.json" % name)
    attempts_path = os.path.join(EXP_D, "batch_attempts_%s.jsonl" % name)
    batches_path = os.path.join(EXP_D, "batches.json")
    log_dir = os.path.join(EXP_D, "d1_logs_%s" % name)
    batch_prefix = args.batch_prefix or ("_d1_%s_" % split)
    families = {f.strip() for f in args.families.split(",") if f.strip()}

    p("[批跑器] manifest=%s" % manifest_path)
    p("[批跑器] name=%s split=%s families=%s prefix=%s" % (name, split, sorted(families) or "ALL", batch_prefix))
    p("[批跑器] index=%s" % out_index)
    p("[批跑器] state=%s" % state_path)
    p("[批跑器] attempts=%s" % attempts_path)

    # --- 重复种子检测 ---
    leak_seeds, leak_ids = check_cross_split_duplicates(entries)
    cross_ok = not leak_seeds and not leak_ids
    p("[重复检测] 跨 split: %s (dup_seed=%d, dup_scenario_id=%d)"
      % ("PASS" if cross_ok else "FAIL", len(leak_seeds), len(leak_ids)))
    if not cross_ok:
        p("  cross-split 泄漏 seed: %s" % leak_seeds)
        p("  cross-split 泄漏 scenario_id: %s" % leak_ids)

    sel_pre = select_entries(entries, args.split, families)
    dup_seeds, dup_ids, legal_pairs, pair_id_entries = check_within_set_duplicates(sel_pre)
    within_ok = not dup_seeds and not dup_ids
    p("[重复检测] 待跑集合(过滤后 %d 条): %s (dup_seed=%d, dup_scenario_id=%d, 合法配对=%d, 带 pair_id 条目=%d)"
      % (len(sel_pre), "PASS" if within_ok else "FAIL", len(dup_seeds), len(dup_ids),
         len(legal_pairs), pair_id_entries))
    if legal_pairs:
        p("  合法配对 seed（同 seed 且 pair_id 唯一非空）: %s" % legal_pairs)
    if not within_ok:
        p("  重复 seed: %s" % dup_seeds)
        p("  重复 scenario_id: %s" % dup_ids)
        p("[错误] 待跑集合内存在重复 seed/scenario_id，拒绝启动以避免重复种子污染。")
        return 2

    # --- offset/limit ---
    sel = sel_pre[args.offset:]
    if args.limit is not None:
        sel = sel[:args.limit]
    p("[批跑器] 本次待跑 %d 条（offset=%d limit=%s）" % (len(sel), args.offset, args.limit))

    state = load_json(state_path) or {"name": name, "completed": {}}
    completed = state.get("completed", {})
    if not os.path.isdir(EXP_D):
        p("[错误] 目录不存在: %s" % EXP_D)
        return 2

    # --- 断点续跑：跳过已完成 ---
    to_run = []
    for entry in sel:
        sid = entry["scenario_id"]
        if sid in completed:
            c = completed[sid]
            p("[跳过] %s 已完成 (final_state=%s category=%s run_id=%s)"
              % (sid, c.get("final_state"), c.get("category"), c.get("run_id")))
            continue
        to_run.append(entry)
    p("[批跑器] 跳过 %d 条，实际执行 %d 条" % (len(sel) - len(to_run), len(to_run)))

    if args.dry_run:
        if args.hot_swap:
            print_hot_swap_dry_run(to_run, args, batch_prefix)
        else:
            p("[dry-run] 将执行的命令：")
            for entry in to_run:
                p("  " + shlex.join(build_cmd(entry, args, batch_prefix)))
        return 0

    # --- hot-swap 常驻热切换：分组一次启动栈，组内逐回合只 set 参数+复位+start ---
    if args.hot_swap:
        serve_log_dir = os.path.join(EXP_D, "logs")
        run_hot_swap(to_run, args, name, split, families, batch_prefix, manifest_path,
                     out_index, state_path, attempts_path, state, completed, serve_log_dir)
        s = summarize_and_write(name, manifest_path, split, families, out_index, batches_path)
        eq_fail = any(v == "FAIL" for v in s["equations"].values())
        return 1 if eq_fail else 0

    for i, entry in enumerate(to_run):
        sid = entry["scenario_id"]
        p("-" * 64)
        p("[批跑器] (%d/%d) 运行 %s" % (i + 1, len(to_run), sid))
        accepted = None
        for k in range(1, args.retries + 2):  # 首次 + retries 次重试
            log_path = os.path.join(log_dir, "%s.att%d.log" % (sid, k))
            att = run_one_entry(entry, args, batch_prefix, log_path)
            # 每次尝试都追加一行到 attempts 日志（原始失败不被覆盖）
            append_jsonl(attempts_path, {
                "batch_name": name, "attempt_index": k, "ts": now_iso(),
                **{key: att[key] for key in
                   ("scenario_id", "split", "family", "seed", "cmd", "returncode", "timed_out",
                    "batch_dir", "run_dir", "final_state", "final_state_name", "category",
                    "reason", "reset_ok", "start_ok", "inject_declared", "inject_note",
                    "infra", "error", "log_path")},
            })
            p("[尝试] %s att%d -> batch_dir=%s run_dir=%s final_state=%s reason=%s infra=%s"
              % (sid, k, att["batch_dir"], att["run_dir"], att["final_state"], att["reason"], att["infra"]))
            # 只要存在 run 记录就追加 index（infra 结局同样保留，不静默抹去）
            if att["run_dir"]:
                append_jsonl(out_index, build_index_row(entry, att, k, name))
            if att["infra"] and k <= args.retries:
                p("[重试] %s att%d 为基础设施失败，进行第 %d 次重试" % (sid, k, k + 1))
                continue
            accepted = att
            break
        # 记录状态（无论最终是 infra 还是有效结局，均标记该 scenario 完成）
        meta = load_json(os.path.join(accepted["run_dir"], "meta.json")) if accepted.get("run_dir") else None
        completed[sid] = {
            "run_id": (meta or {}).get("run_id"), "final_state": accepted["final_state"],
            "category": accepted["category"], "reason": accepted["reason"],
            "attempts": k, "finished_at": now_iso(),
        }
        state["name"] = name
        state["manifest"] = manifest_path
        state["split"] = split
        state["families"] = sorted(families)
        state["updated_at"] = now_iso()
        state["completed"] = completed
        write_json_atomic(state_path, state)

    # --- 汇总：从 index 计算三类分母并核对恒等式 ---
    s = summarize_and_write(name, manifest_path, split, families, out_index, batches_path)

    eq_fail = any(v == "FAIL" for v in s["equations"].values())
    return 1 if eq_fail else 0


if __name__ == "__main__":
    sys.exit(main())
