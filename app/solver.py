"""LED 混光配方台的 CP-SAT 求解核心。

决策变量
========
每路通道 i 的驱动级 ``d_i`` 为 0～255 的整数。

对每个颜色分量 c ∈ {R, G, B}，记::

    s_c = Σ_i reading[i][c] * d_i        （整数）

则该色实际线性分量为 ``s_c / 255``。所有约束都乘上 255 后写在整数变量
``s_c`` 上，因此模型完全是整数线性的，配方本身不做任何四舍五入或裁剪。

求解分两阶段：
  1. 最小化总功率的分子 ``Σ power_i * d_i``；
  2. 在总功率等于最优值的前提下，以 256 为基，对“按 ID 字符串升序”
     排列的驱动级序列做字典序最小化（每位数字 0～255 严格小于基 256，
     加权和与字典序一一对应；6 路时最大为 256^6-1，int64 安全）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ortools.sat.python import cp_model


@dataclass(frozen=True)
class Channel:
    id: str
    rgb: tuple[int, int, int]
    power_mw: int


class SolverError(RuntimeError):
    """求解器内部异常（“无可行解”不在此列，无可行解返回 None）。"""


@dataclass(frozen=True)
class Solution:
    # 与入参 channels 同序的驱动级
    drives: list[int]
    # Σ reading * drive，各色整数分子（实际值 = /255）
    scaled_rgb: list[int]
    actual_rgb: list[float]
    # 实际值 - 目标值
    errors: list[float]
    # Σ power * drive（实际总功率 = /255 mW）
    power_num: int
    power_mw: float
    # 字典序 tie-break 使用的 ID 顺序
    sorted_ids: list[str]


def solve(
    channels: list[Channel],
    target: tuple[int, int, int],
    tolerance: int,
    time_limit_seconds: float = 10.0,
) -> Solution | None:
    """返回最优方案；约束不可行时返回 ``None``。"""
    n = len(channels)
    # tie-break：ID 按字符串字典序升序
    order = sorted(range(n), key=lambda k: channels[k].id)

    def build_base_model() -> (
        tuple[cp_model.CpModel, list[cp_model.IntVar], list[cp_model.IntVar], cp_model.IntVar]
    ):
        m = cp_model.CpModel()
        drives = [m.new_int_var(0, 255, f"drive_{i}") for i in range(n)]

        scaled: list[cp_model.IntVar] = []
        for c in range(3):
            # 理论上界：6 路 × 1000 × 255；域自带 0 下界
            s = m.new_int_var(0, 255 * 1000 * n, f"scaled_{c}")
            m.add(s == sum(channels[i].rgb[c] * drives[i] for i in range(n)))
            # 实际分量须落在目标 ± 容差内（不等号两边同乘 255）
            m.add(s >= 255 * (target[c] - tolerance))
            m.add(s <= 255 * (target[c] + tolerance))
            # 同时须落在 0～1000 内（下界 0 已由变量域保证，不做任何裁剪）
            m.add(s <= 255 * 1000)
            scaled.append(s)

        power_num = m.new_int_var(
            0, 255 * sum(ch.power_mw for ch in channels), "power_num"
        )
        m.add(power_num == sum(channels[i].power_mw * drives[i] for i in range(n)))
        return m, drives, scaled, power_num

    def run(model: cp_model.CpModel, objective: cp_model.IntVar) -> tuple[Any, int]:
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = time_limit_seconds
        solver.parameters.num_search_workers = 8
        model.minimize(objective)
        return solver, solver.solve(model)

    # ---- 阶段 1：最小总功率 ----
    m1, _, _, p1 = build_base_model()
    solver1, st1 = run(m1, p1)
    if st1 == cp_model.INFEASIBLE:
        return None
    if st1 != cp_model.OPTIMAL:
        raise SolverError(
            f"阶段 1（最小功率）未能在 {time_limit_seconds:g}s 内确认最优解，"
            f"求解器状态：{solver1.status_name(st1)}"
        )
    min_power = solver1.value(p1)

    # ---- 阶段 2：同功率下字典序最小 ----
    m2, drives2, scaled2, p2 = build_base_model()
    # min_power 已是全局最小值，<= 即强制等于最优功率
    m2.add(p2 <= min_power)

    weights = [0] * n
    for rank, idx in enumerate(order):
        weights[idx] = 256 ** (n - 1 - rank)
    lex_key = sum(weights[i] * drives2[i] for i in range(n))

    solver2, st2 = run(m2, lex_key)
    if st2 != cp_model.OPTIMAL:
        # 不满足“字典序最小”的方案也不将就返回，避免给出非规定方案
        raise SolverError(
            f"阶段 2（字典序）未能在 {time_limit_seconds:g}s 内确认最优解，"
            f"求解器状态：{solver2.status_name(st2)}"
        )

    dvals = [solver2.value(drives2[i]) for i in range(n)]
    svals = [solver2.value(scaled2[c]) for c in range(3)]
    pval = solver2.value(p2)
    return Solution(
        drives=dvals,
        scaled_rgb=svals,
        actual_rgb=[v / 255.0 for v in svals],
        errors=[(svals[c] - 255 * target[c]) / 255.0 for c in range(3)],
        power_num=pval,
        power_mw=pval / 255.0,
        sorted_ids=[channels[i].id for i in order],
    )
