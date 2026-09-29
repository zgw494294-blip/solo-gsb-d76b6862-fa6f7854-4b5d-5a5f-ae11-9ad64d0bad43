"""调光路径规划：从起始驱动级等时过渡到目标色的最小总功率完整路径。

模型
====
通道 i（n = 3～6）满档线性读数 ``reading[i][c] ∈ [0, 1000]``，起始驱动级
``start[i] ∈ [0, 255]`` 给定。等时步骤数 K（2～5），每步每路驱动变化上限
Δ（1～255），共用整数容差 tol（0～20，与配方求解一致）。

决策变量：第 k 步（k = 1..K）通道 i 的驱动级 ``d[k][i] ∈ [0, 255]`` 整数。

起点实际色沿用配方台的未裁剪公式（不取整、不裁剪）::

    A0_c = (Σ_i reading[i][c] · start[i]) / 255 = S0_c / 255

第 k 步参考色为 A0 到目标色 target 按 k/K 的线性插值（不先取整）::

    ref_k_c = A0_c + (target_c − A0_c) · k / K

记第 k 步实际线性分量为 ``s_k_c / 255``，其中
``s_k_c = Σ_i reading[i][c] · d[k][i]``（整数分子）。窗口约束
``|s_k_c/255 − ref_k_c| ≤ tol`` 两边同乘 255·K 后完全整数化（无浮点变量）::

    |K·s_k_c − T_k_c| ≤ 255·K·tol,   T_k_c = (K−k)·S0_c + 255·k·target_c

另有 ``0 ≤ s_k_c ≤ 255·1000``（不裁剪）与爬坡约束
``|d[k][i] − d[k−1][i]| ≤ Δ``（令 d[0][i] = start[i]，即首步相对起点受限）。

目标：最小化各步功率之和的整数分子 ``Σ_k Σ_i power_i · d[k][i]``
（实际总功率 mW = 该值 / 255，分母为正常数，不影响最优性）。

为加速最优性证明，正式求解前先做 K 次**预求解**：第 k 步无视爬坡约束
时的最小功率 ``lb_k``（窗口约束与正式模型一致，规模与单步配方求解相同，
通常瞬间完成）。``lb_k`` 是第 k 步功率的合法下界，求和即总功率下界；
若某一步的窗口本身不可行，则整条路径必然不可行，可直接判定。
预求解未证明最优时该步下界退化为 0（仅减弱加速效果，不影响正确性）。

结论三态：
  * ``optimal``    —— CP-SAT 判定 OPTIMAL，返回已证明总功率最小的完整路径；
  * ``infeasible`` —— 判定 INFEASIBLE，不存在任何完整路径，不返回局部步骤；
  * ``incomplete`` —— 搜索未证明最优（含超时、仅找到可行解等），不返回任何路径。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from ortools.sat.python import cp_model

from .solver import Channel

# 状态：已证明最优 / 无完整路径 / 搜索未完成
PathStatus = Literal["optimal", "infeasible", "incomplete"]


@dataclass(frozen=True)
class PathStep:
    # 与入参 channels 同序的驱动级
    drives: list[int]
    # Σ reading · drive，各色整数分子（实际值 = /255）
    scaled_rgb: list[int]
    actual_rgb: list[float]
    # 未取整的插值参考色
    ref_rgb: list[float]
    # 实际值 − 参考色
    errors: list[float]
    # 各色是否落在 参考±容差 且 0～1000 内（按整数分子精确判定）
    in_range: list[bool]
    # Σ power · drive（该步实际功率 = /255 mW）
    power_num: int
    power_mw: float


@dataclass(frozen=True)
class PathResult:
    status: PathStatus
    # 仅 optimal 时有值；infeasible / incomplete 一律为空（不返回局部步骤）
    steps: list[PathStep]
    # 起点（第 0 步）信息，与入参 channels 同序
    start_drives: list[int]
    start_actual_rgb: list[float]
    # 各步功率之和（仅 optimal 时有值）
    total_power_num: int | None
    total_power_mw: float | None
    message: str


def solve_path(
    channels: list[Channel],
    start_drives: list[int],
    target: tuple[int, int, int],
    steps: int,
    max_step_delta: int,
    tolerance: int,
    time_limit_seconds: float = 10.0,
) -> PathResult:
    """联合求解完整调光路径。取值合法性由调用方（schema）先行校验。"""
    n = len(channels)
    k_steps = steps
    # 起点实际色的整数分子（未裁剪公式）
    s0 = [
        sum(channels[i].rgb[c] * start_drives[i] for i in range(n))
        for c in range(3)
    ]
    start_actual = [v / 255.0 for v in s0]

    def empty(status: PathStatus, msg: str) -> PathResult:
        return PathResult(
            status=status,
            steps=[],
            start_drives=list(start_drives),
            start_actual_rgb=start_actual,
            total_power_num=None,
            total_power_mw=None,
            message=msg,
        )

    def window_constraints(
        m: cp_model.CpModel, k: int, drives: list[cp_model.IntVar]
    ) -> tuple[list[cp_model.IntVar], list[int]]:
        """第 k 步（1 起始）的颜色窗口约束。

        返回（各色整数分子变量, 各色参考色常数 T_k_c）。
        """
        row: list[cp_model.IntVar] = []
        ref_row: list[int] = []
        for c in range(3):
            # 理论上界：6 路 × 1000 × 255；域自带 0 下界
            s = m.new_int_var(0, 255 * 1000 * n, f"s_{k}_{c}")
            m.add(
                s == sum(channels[i].rgb[c] * drives[i] for i in range(n))
            )
            t = (k_steps - k) * s0[c] + 255 * k * target[c]
            m.add(k_steps * s >= t - 255 * k_steps * tolerance)
            m.add(k_steps * s <= t + 255 * k_steps * tolerance)
            # 同时须落在 0～1000 内（下界 0 已由变量域保证，不做任何裁剪）
            m.add(s <= 255 * 1000)
            row.append(s)
            ref_row.append(t)
        return row, ref_row

    # ---- 预求解：每步无视爬坡约束的最小功率下界 ----
    per_step_lb: list[int] = []
    presolve_limit = max(1.0, time_limit_seconds / k_steps)
    for k in range(1, k_steps + 1):
        pm = cp_model.CpModel()
        pd = [pm.new_int_var(0, 255, f"pd_{k}_{i}") for i in range(n)]
        window_constraints(pm, k, pd)
        pp = pm.new_int_var(
            0, 255 * sum(ch.power_mw for ch in channels), f"pp_{k}"
        )
        pm.add(pp == sum(channels[i].power_mw * pd[i] for i in range(n)))
        pm.minimize(pp)
        ps = cp_model.CpSolver()
        ps.parameters.max_time_in_seconds = presolve_limit
        ps.parameters.num_search_workers = 8
        pst = ps.solve(pm)
        if pst == cp_model.INFEASIBLE:
            return empty(
                "infeasible",
                f"无完整路径：第 {k} 步的参考色窗口（±容差 且 0～1000）"
                "本身就不存在可行驱动级组合（CP-SAT 已证明），整条路径必然"
                "不可行，不返回任何局部步骤。可尝试放宽容差、调整终点色或"
                "增加/更换通道。",
            )
        # 预求解未证明最优时下界退化为 0（只影响加速，不影响正确性）
        per_step_lb.append(ps.value(pp) if pst == cp_model.OPTIMAL else 0)

    m = cp_model.CpModel()
    d = [
        [m.new_int_var(0, 255, f"d_{k}_{i}") for i in range(n)]
        for k in range(k_steps)
    ]

    # 爬坡约束：相邻两步（首步与起点）每路驱动变化不超过 Δ
    for i in range(n):
        for k in range(k_steps):
            prev = start_drives[i] if k == 0 else d[k - 1][i]
            m.add(d[k][i] - prev <= max_step_delta)
            m.add(prev - d[k][i] <= max_step_delta)

    # 每步颜色窗口（整数化，见模块 docstring）
    scaled: list[list[cp_model.IntVar]] = []
    refs: list[list[int]] = []  # 每步每色的整数常数 T_k_c
    for k in range(1, k_steps + 1):
        row, ref_row = window_constraints(m, k, d[k - 1])
        scaled.append(row)
        refs.append(ref_row)

    # 目标：各步功率之和的整数分子最小；每步功率带预求解下界
    step_powers: list[cp_model.IntVar] = []
    for k in range(k_steps):
        p_k = m.new_int_var(
            per_step_lb[k],
            255 * sum(ch.power_mw for ch in channels),
            f"p_{k}",
        )
        m.add(p_k == sum(channels[i].power_mw * d[k][i] for i in range(n)))
        step_powers.append(p_k)
    power_num = m.new_int_var(
        sum(per_step_lb),
        k_steps * 255 * sum(ch.power_mw for ch in channels),
        "power_num",
    )
    m.add(power_num == sum(step_powers))
    m.minimize(power_num)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit_seconds
    solver.parameters.num_search_workers = 8
    st = solver.solve(m)

    if st == cp_model.INFEASIBLE:
        return empty(
            "infeasible",
            "无完整路径：在给定起始驱动级、步骤数、每步变化上限与容差下，"
            "不存在任何满足全部步骤窗口约束（参考色 ± 容差 且 0～1000）的"
            "驱动级序列（CP-SAT 已证明不可行），不返回任何局部步骤。"
            "可尝试放宽容差、增大每步变化上限、增加步骤数或调整终点色。",
        )
    if st != cp_model.OPTIMAL:
        # 未证明最优（含仅找到可行解）：不返回任何路径
        return empty(
            "incomplete",
            f"搜索未完成（求解器状态：{solver.status_name(st)}），未能在 "
            f"{time_limit_seconds:g}s 内证明各步功率之和最小的完整路径，"
            "不返回任何路径。可增大求解时限或调整参数后重试。",
        )

    steps_out: list[PathStep] = []
    for k in range(k_steps):
        dvals = [solver.value(d[k][i]) for i in range(n)]
        svals = [solver.value(scaled[k][c]) for c in range(3)]
        pnum = sum(channels[i].power_mw * dvals[i] for i in range(n))
        steps_out.append(
            PathStep(
                drives=dvals,
                scaled_rgb=svals,
                actual_rgb=[v / 255.0 for v in svals],
                ref_rgb=[
                    refs[k][c] / (255.0 * k_steps) for c in range(3)
                ],
                errors=[
                    (k_steps * svals[c] - refs[k][c]) / (255.0 * k_steps)
                    for c in range(3)
                ],
                in_range=[
                    abs(k_steps * svals[c] - refs[k][c])
                    <= 255 * k_steps * tolerance
                    and 0 <= svals[c] <= 255 * 1000
                    for c in range(3)
                ],
                power_num=pnum,
                power_mw=pnum / 255.0,
            )
        )
    total = solver.value(power_num)
    return PathResult(
        status="optimal",
        steps=steps_out,
        start_drives=list(start_drives),
        start_actual_rgb=start_actual,
        total_power_num=total,
        total_power_mw=total / 255.0,
        message=(
            "求解成功：各步功率之和已最小（CP-SAT 已证明最优），"
            "路径完整覆盖全部等时步骤。"
        ),
    )
