"""LED 满档读数标定（由实测样本反求各路满档线性 RGB）。

给定 n 路通道（3～6）与 k 组实测样本（3～12），样本 s 记录各路的整数
驱动级 ``d[s][i] ∈ [0, 255]`` 以及实测线性 RGB``m[s][c] ∈ [0, 6000]``。

设通道 i 的满档读数为待求整数 ``x[i][c] ∈ [0, 1000]``（每路三色不全为
零），则样本 s 在颜色 c 上的**精确**线性值为::

    y[s][c] = (Σ_i x[i][c] · d[s][i]) / 255 = S[s][c] / 255

实测值按四舍五入（半值向上）解释：``m = round_half_up(y)``，即::

    m - 1/2 ≤ y < m + 1/2

两边乘 2·255 = 510，全部写成整数线性约束（不引入浮点变量）::

    255·(2m - 1) ≤ 2·S ≤ 255·(2m + 1) - 1

m = 0 时该式等价于 S ≤ 127（即 y < 0.5），自然成立，无需特判。

结论三态（+ 搜索未完成），用 CP-SAT 两次求解给出**可证明**的判定：

  1. 先求一个可行参数 x*。无解（INFEASIBLE）→ 样本矛盾；搜索超时未完成
     → unknown，**不得**报告矛盾或唯一；
  2. 再在原约束上追加“至少一个格子与 x* 不同”的析取约束。判定不可行
     （INFEASIBLE）→ x* 是唯一合法参数（标定唯一）；找到第二个解 →
     标定不唯一；搜索未完成 → unknown。

只有状态为 unique 时才把 x* 返回给调用方展示/写回。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from ortools.sat.python import cp_model

# 状态：唯一 / 矛盾 / 不唯一 / 搜索未完成
CalibStatus = Literal["unique", "contradictory", "non_unique", "unknown"]


@dataclass(frozen=True)
class CalibSample:
    # 与通道表同序的各路驱动级 0～255
    drives: tuple[int, ...]
    # 实测线性 RGB，四舍五入后的整数 0～6000
    measured: tuple[int, int, int]


@dataclass(frozen=True)
class CalibResult:
    status: CalibStatus
    # 与入参 channels 同序的 (id, r, g, b)；仅 unique 时有值
    channels: list[tuple[str, int, int, int]]
    message: str


def round_half_up(value: float) -> int:
    """四舍五入（小数部分 ≥ 0.5 一律向上，不采用 banker's rounding）。"""
    import math

    return int(math.floor(value + 0.5))


def predicted_rgb(
    readings: list[tuple[int, int, int]], drives: tuple[int, ...]
) -> tuple[int, int, int]:
    """按标定模型的解释口径，由满档读数与驱动级计算实测（四舍五入）RGB。"""
    out: list[int] = []
    for c in range(3):
        s = sum(readings[i][c] * drives[i] for i in range(len(drives)))
        out.append(round_half_up(s / 255.0))
    return out[0], out[1], out[2]


def calibrate(
    channel_ids: list[str],
    samples: list[CalibSample],
    time_limit_seconds: float = 10.0,
) -> CalibResult:
    """反求各路满档 RGB。长度/取值合法性由调用方（schema）先行校验。"""
    n = len(channel_ids)
    k = len(samples)

    def build_model() -> tuple[cp_model.CpModel, dict[tuple[int, int], cp_model.IntVar]]:
        m = cp_model.CpModel()
        # x[(i, c)]：通道 i 的颜色 c 满档读数，0～1000 整数
        x: dict[tuple[int, int], cp_model.IntVar] = {}
        for i in range(n):
            for c in range(3):
                x[(i, c)] = m.new_int_var(0, 1000, f"x_{i}_{c}")
            # 各路三色不全为零
            m.add(sum(x[(i, c)] for c in range(3)) >= 1)

        for s_idx, smp in enumerate(samples):
            for c in range(3):
                measured = smp.measured[c]
                # S = Σ x[i][c] · d[i]；理论上界 n·1000·255
                sv = m.new_int_var(0, n * 1000 * 255, f"S_{s_idx}_{c}")
                m.add(
                    sv
                    == sum(
                        x[(i, c)] * smp.drives[i] for i in range(n)
                    )
                )
                # round_half_up(S/255) = measured 的整数线性化：
                # 255·(2m-1) ≤ 2S ≤ 255·(2m+1)-1
                twice = m.new_int_var(0, 2 * n * 1000 * 255, f"t_{s_idx}_{c}")
                m.add(twice == 2 * sv)
                m.add(twice >= 255 * (2 * measured - 1))
                m.add(twice <= 255 * (2 * measured + 1) - 1)
        return m, x

    def run(model: cp_model.CpModel) -> tuple[Any, cp_model.CpSolver]:
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = time_limit_seconds
        solver.parameters.num_search_workers = 8
        return solver.solve(model), solver

    # ---- 第 1 次：是否存在任何合法参数 ----
    m1, x1 = build_model()
    st1, solver1 = run(m1)
    if st1 == cp_model.INFEASIBLE:
        return CalibResult(
            status="contradictory",
            channels=[],
            message=(
                "样本矛盾：不存在任何 0～1000 整数满档 RGB（各路三色不全为零）"
                "能同时满足全部样本在四舍五入解释下的实测值。请核对驱动级与"
                "实测读数录入，或剔除误测样本后重试。"
            ),
        )
    if st1 not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return CalibResult(
            status="unknown",
            channels=[],
            message=(
                f"标定搜索未完成（第 1 次求解状态：{solver1.status_name(st1)}），"
                "既不能判定样本矛盾，也不能证明参数唯一。请增加求解时限或"
                "调整样本后重试。"
            ),
        )

    candidate = [
        tuple(solver1.value(x1[(i, c)]) for c in range(3)) for i in range(n)
    ]

    # ---- 第 2 次：在原约束上寻找一个不同于 candidate 的合法参数 ----
    m2, x2 = build_model()
    diff_terms: list[cp_model.IntVar] = []
    for i in range(n):
        for c in range(3):
            # b = 1 表示该格子与候选解不同（用 (x == cand) + b == 1 表达）
            b = m2.new_bool_var(f"diff_{i}_{c}")
            same = m2.new_bool_var(f"same_{i}_{c}")
            m2.add(x2[(i, c)] == candidate[i][c]).only_enforce_if(same)
            m2.add(x2[(i, c)] != candidate[i][c]).only_enforce_if(b)
            m2.add(b + same == 1)
            diff_terms.append(b)
    # 至少一个格子不同 → 存在第二个合法参数
    m2.add(sum(diff_terms) >= 1)

    st2, solver2 = run(m2)
    if st2 in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return CalibResult(
            status="non_unique",
            channels=[],
            message=(
                "标定不唯一：存在多组合法的各路满档 RGB 都能解释全部样本"
                "（模型已找到两个不同的合法参数）。请增加样本数量或覆盖更多"
                "驱动级组合（尤其是各路单独点亮的样本），使方程组足以唯一"
                "确定每路读数。"
            ),
        )
    if st2 == cp_model.INFEASIBLE:
        return CalibResult(
            status="unique",
            channels=[
                (channel_ids[i], candidate[i][0], candidate[i][1], candidate[i][2])
                for i in range(n)
            ],
            message=(
                "标定唯一：CP-SAT 已证明该满档 RGB 是能解释全部样本的唯一"
                "合法整数参数（0～1000、各路三色不全为零）。可一键写回当前"
                "通道表（保留原功率）。"
            ),
        )
    return CalibResult(
        status="unknown",
        channels=[],
        message=(
            f"标定搜索未完成（已找到合法参数，但第 2 次求解状态："
            f"{solver2.status_name(st2)}），无法证明其是否唯一。请增加求解"
            "时限或补充样本后重试。"
        ),
    )
