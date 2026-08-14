#!/usr/bin/env python3
"""
@file: dqn_agent.py
@brief: DQN 智能体 — QNetwork + DQNAgent（epsilon-greedy, target network, save/load）
@date: 2026-07-21
"""

import random
import torch
import torch.nn as nn
import torch.optim as optim


class DuelingQNetwork(nn.Module):
    """
    @brief: Dueling Q 网络 — 共享特征层 + value/advantage 分流
    @detail: state_dim → 256 → ReLU → 256 → ReLU → 256 → ReLU → V(s) + A(s,a) - mean(A)
    """

    def __init__(self, state_dim, action_dim):
        super().__init__()
        # 共享特征层
        self.feature = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
        )
        # 价值流 V(s)
        self.value = nn.Linear(256, 1)
        # 优势流 A(s, a)
        self.advantage = nn.Linear(256, action_dim)

    def forward(self, x):
        feat = self.feature(x)
        value = self.value(feat)          # [batch, 1]
        advantage = self.advantage(feat)  # [batch, action_dim]
        # Q(s,a) = V(s) + A(s,a) - mean(A(s,·))
        return value + advantage - advantage.mean(dim=-1, keepdim=True)


# 向后兼容别名
QNetwork = DuelingQNetwork


class DQNAgent:
    """
    @brief: DQN 智能体，维护在线网络 + 目标网络
    """

    def __init__(self, state_dim, action_dim, lr=1e-3, gamma=0.99):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.initial_lr = lr
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # 在线网络（训练更新）
        self.q_network = QNetwork(state_dim, action_dim).to(self.device)
        # 目标网络（计算 TD target）
        self.target_network = QNetwork(state_dim, action_dim).to(self.device)
        self.sync_target()  # 初始同步

        self.optimizer = optim.Adam(self.q_network.parameters(), lr=lr)
        self.loss_fn = nn.SmoothL1Loss()

    def select_action(self, state, epsilon=0.0, allowed_actions=None):
        """
        @brief: epsilon-greedy 动作选择
        @param state: list 或 tensor, shape [state_dim]
        @param epsilon: 随机动作概率
        @return: int 动作索引
        """
        allowed = (list(range(self.action_dim)) if allowed_actions is None
                   else list(allowed_actions))
        if not allowed:
            raise ValueError("allowed_actions must not be empty")
        if random.random() < epsilon:
            return random.choice(allowed)

        # 转为 tensor 前向推理
        with torch.no_grad():
            state_t = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            q_values = self.q_network(state_t)
            allowed_t = torch.LongTensor(allowed).to(self.device)
            allowed_q = q_values[0].index_select(0, allowed_t)
            return allowed[int(allowed_q.argmax().item())]

    def update(self, batch, weights=None):
        """
        @brief: 从经验批次学习，执行一步梯度下降
        @param batch: Experience 命名元组列表
        @param weights: 重要性采样权重列表（PER），None 表示均匀权重
        @return: float loss 值
        """
        # 解包 batch → tensor
        states = torch.FloatTensor([e.state for e in batch]).to(self.device)
        actions = torch.LongTensor([e.action for e in batch]).unsqueeze(1).to(self.device)
        rewards = torch.FloatTensor([e.reward for e in batch]).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor([e.next_state for e in batch]).to(self.device)
        dones = torch.FloatTensor([1.0 if e.done else 0.0 for e in batch]).unsqueeze(1).to(self.device)

        # 当前 Q 值
        q_values = self.q_network(states).gather(1, actions)

        # Double DQN TD target：用 q_network 选动作，target_network 估值
        with torch.no_grad():
            # 用 q_network 获取 next_states 的 Q 值并 mask 不可用动作
            next_q_values = self.q_network(next_states)
            for row, experience in enumerate(batch):
                allowed = experience.next_allowed_actions
                if allowed is not None:
                    blocked = [a for a in range(self.action_dim)
                               if a not in allowed]
                    if blocked:
                        next_q_values[row, blocked] = float("-inf")
            # 选最佳动作索引
            best_actions = next_q_values.argmax(dim=1, keepdim=True)
            # 用 target_network 获取对应动作的 Q 值
            next_q = self.target_network(next_states).gather(1, best_actions)
            target = rewards + self.gamma * next_q * (1.0 - dones)

        # 计算逐元素 loss，应用 IS 权重
        element_loss = self.loss_fn(q_values, target)
        if weights is not None:
            weights_t = torch.FloatTensor(weights).unsqueeze(1).to(self.device)
            loss = (element_loss * weights_t).mean()
        else:
            loss = element_loss

        # 反向传播
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.q_network.parameters(), max_norm=10.0)
        self.optimizer.step()

        return float(loss.item())

    def sync_target(self):
        """将在线网络权重复制到目标网络"""
        self.target_network.load_state_dict(self.q_network.state_dict())

    def schedule_lr(self, episode, total_episodes):
        """
        @brief: 余弦退火学习率调度
        @param episode: 当前 episode 编号（从 0 开始）
        @param total_episodes: 总 episode 数
        """
        import math
        lr_min = self.initial_lr * 0.01
        lr_max = self.initial_lr
        lr = lr_min + 0.5 * (lr_max - lr_min) * (1.0 + math.cos(math.pi * episode / total_episodes))
        for param_group in self.optimizer.param_groups:
            param_group["lr"] = lr
        return lr

    def save(self, path):
        """保存在线网络权重"""
        torch.save(self.q_network.state_dict(), path)

    def load(self, path):
        """加载权重到在线网络 + 目标网络"""
        try:
            state_dict = torch.load(path, map_location=self.device, weights_only=True)
        except TypeError:
            state_dict = torch.load(path, map_location=self.device)
        self.q_network.load_state_dict(state_dict)
        self.sync_target()
