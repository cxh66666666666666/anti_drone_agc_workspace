#!/usr/bin/env python3
"""
@file: replay_buffer.py
@brief: 经验回放缓冲区 — 存储转移及下一状态可用动作；支持优先级回放
@date: 2026-07-21
"""

import random
from collections import deque, namedtuple

Experience = namedtuple(
    "Experience",
    ["state", "action", "reward", "next_state", "done",
     "next_allowed_actions", "bootstrap_discount"],
    defaults=[None])


class ReplayBuffer:
    """
    @brief: 固定容量循环缓冲区，支持 push / sample / __len__
    """

    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done,
             next_allowed_actions=None, bootstrap_discount=None,
             priority_hint=1.0):
        """存入一条经验"""
        exp = Experience(state, action, reward, next_state, done,
                         next_allowed_actions, bootstrap_discount)
        self.buffer.append(exp)

    def sample(self, batch_size):
        """
        @brief: 随机采样 batch_size 条经验
        @return: Experience 命名元组列表，或 None（缓冲区不足）
        """
        if len(self.buffer) < batch_size:
            return None
        batch = random.sample(self.buffer, batch_size)
        return batch

    def __len__(self):
        return len(self.buffer)


class SumTree:
    """二叉树结构，用于优先级采样 O(log N)"""

    def __init__(self, capacity):
        self.capacity = capacity
        self.tree = [0.0] * (2 * capacity)
        self.ptr = 0

    def add(self, priority):
        """在 ptr 位置写入优先级，后移指针"""
        idx = self.ptr + self.capacity
        self.update(idx, priority)
        self.ptr = (self.ptr + 1) % self.capacity

    def update(self, idx, priority):
        """更新叶子节点优先级并向上传播"""
        diff = priority - self.tree[idx]
        self.tree[idx] = priority
        while idx > 1:
            idx //= 2
            self.tree[idx] += diff

    def get(self, prefix_sum):
        """按累积和查找叶子节点，返回 (树索引, 优先级)"""
        idx = 1
        while idx < self.capacity:
            left = 2 * idx
            if prefix_sum <= self.tree[left]:
                idx = left
            else:
                prefix_sum -= self.tree[left]
                idx = left + 1
        return idx, self.tree[idx]

    @property
    def total_priority(self):
        return self.tree[1]


class PrioritizedReplayBuffer:
    """
    @brief: 优先级经验回放缓冲区（PER）
    @detail: 使用 SumTree 实现概率采样，采样概率 ∝ priority^alpha
    """

    def __init__(self, capacity=10000, alpha=0.6, beta=0.4):
        self.tree = SumTree(capacity)
        self.capacity = capacity
        self.alpha = alpha
        self.beta = beta
        self.beta_increment = 1e-4
        self.epsilon = 1e-6
        self.buffer = [None] * capacity
        self.count = 0
        self.ptr = 0

    def push(self, state, action, reward, next_state, done,
             next_allowed_actions=None, bootstrap_discount=None,
             priority_hint=1.0):
        """存入经验，初始优先级为当前最大优先级或 1.0"""
        exp = Experience(state, action, reward, next_state, done,
                         next_allowed_actions, bootstrap_discount)
        self.buffer[self.ptr] = exp
        if self.count > 0:
            max_priority = max(self.tree.tree[self.tree.capacity:])
        else:
            max_priority = 1.0
        hinted_priority = max(1.0, priority_hint) ** self.alpha
        self.tree.add(max(max_priority, hinted_priority))
        self.ptr = (self.ptr + 1) % self.capacity
        self.count = min(self.count + 1, self.capacity)

    def sample(self, batch_size):
        """按优先级采样，返回 (batch, indices, weights)"""
        if self.count < batch_size:
            return None

        batch = []
        indices = []
        priorities = []
        segment = self.tree.total_priority / batch_size

        for i in range(batch_size):
            low = segment * i
            high = segment * (i + 1)
            prefix_sum = random.uniform(low, high)
            idx, priority = self.tree.get(prefix_sum)
            indices.append(idx)
            priorities.append(priority)
            batch.append(self.buffer[idx - self.tree.capacity])

        # 计算重要性采样权重
        probs = [p / self.tree.total_priority for p in priorities]
        weights = [(self.count * prob) ** (-self.beta) for prob in probs]
        max_w = max(weights)
        weights = [w / max_w for w in weights]

        self.beta = min(1.0, self.beta + self.beta_increment)
        return batch, indices, weights

    def update_priorities(self, indices, td_errors):
        """更新优先级：priority = |td_error| + epsilon"""
        for idx, td_error in zip(indices, td_errors):
            priority = abs(td_error) + self.epsilon
            self.tree.update(idx, priority ** self.alpha)

    def state_dict(self):
        """Return replay contents and sampling state for training resume."""
        return {
            "capacity": self.capacity,
            "alpha": self.alpha,
            "beta": self.beta,
            "beta_increment": self.beta_increment,
            "epsilon": self.epsilon,
            "buffer": [tuple(item) if item is not None else None
                       for item in self.buffer],
            "count": self.count,
            "ptr": self.ptr,
            "tree": self.tree.tree,
            "tree_ptr": self.tree.ptr,
        }

    def load_state_dict(self, state):
        """Restore a replay buffer created with the same capacity."""
        if int(state["capacity"]) != self.capacity:
            raise ValueError(
                "replay buffer capacity mismatch: checkpoint=%d current=%d" %
                (state["capacity"], self.capacity))
        self.alpha = float(state["alpha"])
        self.beta = float(state["beta"])
        self.beta_increment = float(state["beta_increment"])
        self.epsilon = float(state["epsilon"])
        self.buffer = [Experience(*item) if item is not None else None
                       for item in state["buffer"]]
        self.count = int(state["count"])
        self.ptr = int(state["ptr"])
        self.tree.tree = list(state["tree"])
        self.tree.ptr = int(state["tree_ptr"])

    def __len__(self):
        return self.count
