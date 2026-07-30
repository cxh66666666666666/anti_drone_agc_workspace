#!/usr/bin/env python3
"""
@file: replay_buffer.py
@brief: 经验回放缓冲区 — 存储转移及下一状态可用动作
@date: 2026-07-21
"""

import random
from collections import deque, namedtuple

Experience = namedtuple(
    "Experience",
    ["state", "action", "reward", "next_state", "done",
     "next_allowed_actions"])


class ReplayBuffer:
    """
    @brief: 固定容量循环缓冲区，支持 push / sample / __len__
    """

    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done,
             next_allowed_actions=None):
        """存入一条经验"""
        exp = Experience(state, action, reward, next_state, done,
                         next_allowed_actions)
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
