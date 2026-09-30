"""容错配方规划：可承受任意单路 LED 熄灭的最小功率配方。

模型
====
通道 i（n = 3～6）满档线性读数 ``reading[i][c] ∈ [0, 1000]``，目标线性
RGB ``target``、共用整数容差 tol（0～20），以及故障后**每路**允许调整的
最大驱动级差 ``limit``（1～255）。

决策变量：
  * 正常配方驱动级 ``d0[i] ∈ [0, 255]``；
  * 对每个可能熄灭的通道 f（f = 0..n−1），一路备用配方驱动级
    ``df[f][i] ∈ [0, 255]``。

所有配方（正常与 n 组备用）的实际线性分量都沿用配方台**未裁剪、不取整**
公式（见 ``app/solver.py``）。令 ``s = Σ_i reading[i][c] · d_i``，约束
（两边同乘 255，完全整数化）::

    255·(target_c − tol) ≤ s_c ≤ 255·(target_c + tol)
    0 ≤ s_c ≤ 255·1000

故障 f 的备用配方额外满足：

  * 故障路必须熄灭：``df[f][f] = 0``；
  * 其余各路相对正常值的调整不超过上限：
    ``|df[f][i] − d0[i]| ≤ limit``（i ≠ f）。

目标
====
在**全部 n 组备用配方都存在**的正常方案中，最小化正常总功率的整数分子
``Σ_i power_i · d0[i]``（实际 mW = 该值 / 255，分母为正的常数，不影响
最优性）；同功率时按 **ID 字符串升序**的正常驱动序列取字典序最小。
备用配方只需可行，不参与最优化（实现上也不进入目标函数）。

求解分两阶段：
  1. 在“正常 + 全部备用”的联合模型上最小化正常功率；
  2. 追加正常功率等于最优值，以 256 为基把按 ID 升序的正常驱动序列
     编码为单个整数目标做字典序最小化（每位数字 0～255 严格小于基 256，
     加权和与字典序一一对应；6 路时最大 256^6−1，int64 安全）。

结论三态：
  * ``optimal``    —— CP-SAT 判定 OPTIMAL，返回正常配方与全部备用配方；
  * ``infeasible`` —— 判定 INFEASIBLE，不存在具备全部备用配方的正常方案，
                      不返回任何部分结果；
  * ``incomplete`` —— 搜索未证明最优（含超时、仅找到可行解等），
                      不返回任何配方（包括正常配方）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from ortools.sat.python import cp_model

from .solver import Channel

# 状态：已证明最优 / 无完整容错方案 / 搜索未完成
RobustStatus = Literal["optimal", "infeasible", "incomplete"]


@dataclass(frozen=True)
class RobustRecipe:
    """一组配方（正常或某路故障后的备用）。"""

    # 与入参 channels 同序的驱动级
    drives: list[int]
    # Σ reading · drive，各色整数分子（实际值 = /255）
    scaled_rgb: list[int]
    actual_rgb: list[float]
    # 实际值 − 目标值
    errors: list[float]
    # 各色是否落在 目标±容差 且 0～1000 内（按整数分子精确判定）
    in_range: list[bool]
    # Σ power · drive（实际功率 = /255 mW）
    power_num: int
    power_mw: float


@dataclass(frozen=True)
class RobustPlan:
    status: RobustStatus
    # 与入参 channels 同序
    channel_ids: list[str]
    # 字典序 tie-break 使用的 ID 顺序
    sorted_ids: list[str]
    # 仅 optimal 时有值；infeasible / incomplete 一律为空（不返回部分备用）
    normal: RobustRecipe | None
    # 仅 optimal 时有值，与 channels 同序，第 f 项为第 f 路熄灭后的备用配方
    fallbacks: list[RobustRecipe]
    message: str


def _build_recipe_vars(
    m: cp_model.CpModel,
    channels: list[Channel],
    n: int,
    tag: str,
) -> tuple[list[cp_model.IntVar], list[cp_model.IntVar], cp_model.IntVar]:
    """在模型 m 中加入一组配方变量（驱动级、三色整数分子、功率分子）。"""
    drives = [m.new_int_var(0, 255, f"{tag}_d_{i}") for i in range(n)]
    scaled: list[cp_model.IntVar] = []
    for c in range(3):
        # 理论上界：6 路 × 1000 × 255；域自带 0 下界
        s = m.new_int_var(0, 255 * 1000 * n, f"{tag}_s_{c}")
        m.add(s == sum(channels[i].rgb[c] * drives[i] for i in range(n)))
        scaled.append(s)
    power_num = m.new_int_var(
        0, 255 * sum(ch.power_mw for ch in channels), f"{tag}_p"
    )
    m.add(power_num == sum(channels[i].power_mw * drives[i] for i in range(n)))
    return drives, scaled, power_num


def _add_color_window(
    m: cp_model.CpModel,
    scaled: list[cp_model.IntVar],
    target: tuple[int, int, int],
    tolerance: int,
) -> None:
    """实际分量落在 目标±容差 且 0～1000 内（整数化，下界 0 已由变量域保证）。"""
    for c in range(3):
        m.add(scaled[c] >= 255 * (target[c] - tolerance))
        m.add(scaled[c] <= 255 * (target[c] + tolerance))
        m.add(scaled[c] <= 255 * 1000)


def solve_robust(
    channels: list[Channel],
    target: tuple[int, int, int],
    tolerance: int,
    max_adjust_delta: int,
    time_limit_seconds: float = 10.0,
) -> RobustPlan:
    """联合求解具备全部单路熄灭备用配方的正常方案。取值合法性由 schema 先行校验。"""
    n = len(channels)
    ids = [ch.id for ch in channels]
    # tie-break：ID 按字符串字典序升序
    order = sorted(range(n), key=lambda k: channels[k].id)

    def empty(status: RobustStatus, msg: str) -> RobustPlan:
        return RobustPlan(
            status=status,
            channel_ids=list(ids),
            sorted_ids=[channels[i].id for i in order],
            normal=None,
            fallbacks=[],
            message=msg,
        )

    def build_model() -> tuple[
        cp_model.CpModel,
        list[cp_model.IntVar],
        list[cp_model.IntVar],
        cp_model.IntVar,
        list[list[cp_model.IntVar]],
        list[list[cp_model.IntVar]],
    ]:
        m = cp_model.CpModel()
        # 正常配方
        d0, s0, p0 = _build_recipe_vars(m, channels, n, "normal")
        _add_color_window(m, s0, target, tolerance)

        # 逐路熄灭备用配方
        fb_drives: list[list[cp_model.IntVar]] = []
        fb_scaled: list[list[cp_model.IntVar]] = []
        for f in range(n):
            df, sf, _pf = _build_recipe_vars(m, channels, n, f"fb_{f}")
            _add_color_window(m, sf, target, tolerance)
            # 故障路必须为 0
            m.add(df[f] == 0)
            # 其余各路相对正常值的调整不超过上限
            for i in range(n):
                if i == f:
                    continue
                m.add(df[i] - d0[i] <= max_adjust_delta)
                m.add(d0[i] - df[i] <= max_adjust_delta)
            fb_drives.append(df)
            fb_scaled.append(sf)
        return m, d0, s0, p0, fb_drives, fb_scaled

    def run(model: cp_model.CpModel, objective: cp_model.IntVar) -> tuple[object, int]:
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = time_limit_seconds
        solver.parameters.num_search_workers = 8
        model.minimize(objective)
        return solver, solver.solve(model)

    # ---- 阶段 1：正常配方总功率最小（须同时具备全部备用配方）----
    m1, _, _, p1, _, _ = build_model()
    solver1, st1 = run(m1, p1)
    if st1 == cp_model.INFEASIBLE:
        return empty(
            "infeasible",
            "无完整容错配方：不存在这样的正常驱动级组合——使每一路单独熄灭后，"
            "其余各路在允许的最大调整级差内，都能把未裁剪的实际 RGB 拉回"
            "目标 ± 容差（且 0～1000）内。CP-SAT 已证明该联合约束不可行，"
            "不返回任何部分备用方案。可尝试放大调整上限、放宽容差、调整"
            "目标色或增加/更换通道。",
        )
    if st1 != cp_model.OPTIMAL:
        return empty(
            "incomplete",
            f"搜索未完成（求解器状态：{solver1.status_name(st1)}），未能在 "
            f"{time_limit_seconds:g}s 内证明正常配方总功率最小且全部备用配方"
            "可行，不返回任何配方（包括正常配方）。可增大求解时限或调整参数"
            "后重试。",
        )
    min_power = solver1.value(p1)

    # ---- 阶段 2：同功率下正常驱动序列按 ID 升序取字典序最小 ----
    m2, d2, s2, p2, fb_d2, fb_s2 = build_model()
    # min_power 已是全局最小值，<= 即强制等于最优功率
    m2.add(p2 <= min_power)
    weights = [0] * n
    for rank, idx in enumerate(order):
        weights[idx] = 256 ** (n - 1 - rank)
    lex_key = sum(weights[i] * d2[i] for i in range(n))

    solver2, st2 = run(m2, lex_key)
    if st2 != cp_model.OPTIMAL:
        # 不满足“字典序最小”的方案也不将就返回
        return empty(
            "incomplete",
            f"搜索未完成（阶段 2 字典序 tie-break，求解器状态："
            f"{solver2.status_name(st2)}），未能在 {time_limit_seconds:g}s 内"
            "确认同功率下字典序最小的正常配方，不返回任何配方（包括正常配方）。"
            "可增大求解时限或调整参数后重试。",
        )

    def make_recipe(dvals: list[int], svals: list[int]) -> RobustRecipe:
        pnum = sum(channels[i].power_mw * dvals[i] for i in range(n))
        return RobustRecipe(
            drives=dvals,
            scaled_rgb=svals,
            actual_rgb=[v / 255.0 for v in svals],
            errors=[(svals[c] - 255 * target[c]) / 255.0 for c in range(3)],
            in_range=[
                255 * (target[c] - tolerance) <= svals[c]
                <= 255 * (target[c] + tolerance)
                and 0 <= svals[c] <= 255 * 1000
                for c in range(3)
            ],
            power_num=pnum,
            power_mw=pnum / 255.0,
        )

    d0vals = [solver2.value(v) for v in d2]
    s0vals = [solver2.value(v) for v in s2]
    normal = make_recipe(d0vals, s0vals)
    fallbacks: list[RobustRecipe] = []
    for f in range(n):
        dfvals = [solver2.value(v) for v in fb_d2[f]]
        sfvals = [solver2.value(v) for v in fb_s2[f]]
        fallbacks.append(make_recipe(dfvals, sfvals))

    return RobustPlan(
        status="optimal",
        channel_ids=list(ids),
        sorted_ids=[channels[i].id for i in order],
        normal=normal,
        fallbacks=fallbacks,
        message=(
            "求解成功：正常配方总功率已最小（CP-SAT 已证明最优），且每一路"
            "单独熄灭后，其余各路在允许的最大调整级差内都存在可行备用配方；"
            "同功率下正常驱动序列按 ID 字典序最小。"
        ),
    )
