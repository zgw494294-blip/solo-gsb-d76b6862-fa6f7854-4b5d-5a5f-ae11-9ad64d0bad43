"""故障容错配方规划：正常配方 + 每一路单独熄灭后的备用配方。

模型
====
通道 i（n = 3～6）满档线性读数 ``reading[i][c] ∈ [0, 1000]``，满档功率
``power_i``；目标线性 RGB ``target``，共用整数容差 ``tol``，故障后每路允许
调整的最大驱动级差 ``max_adjust``（1～255）。

共有 n+1 个场景：

* 场景 0 —— 正常：驱动级 ``d[0][i] ∈ [0, 255]``；
* 场景 f（f = 1..n）—— 第 f 路（按入参序号，f-1 即通道下标）熄灭：
  ``d[f][f-1] = 0``，其余 ``d[f][i] ∈ [0, 255]``。

每个场景都须满足与普通配方完全相同的颜色窗口约束（未裁剪公式，不取整）::

    s_q_c = Σ_i reading[i][c] · d[q][i]
    255·(target_c − tol) ≤ s_q_c ≤ 255·(target_c + tol)
    0 ≤ s_q_c ≤ 255·1000

备用场景相对正常方案还有调整幅度约束（故障路恒为 0，天然满足）::

    |d[f][i] − d[0][i]| ≤ max_adjust

目标：在全部 n+1 个场景均可行的正常方案中，最小化正常总功率的整数分子
``Σ_i power_i · d[0][i]``（实际 mW = 该值 / 255，分母为正常数，不影响
最优性）；同功率时，按 **ID 字符串升序**排列的正常驱动级序列取字典序
最小（与普通配方相同的 256 基数编码）。备用配方只需可行，不参与目标。

正式求解前先做 n+1 次**单场景预求解**：正常场景的最小功率作为主模型正常
功率变量的合法下界以加速最优性证明；逐个熄灭场景在只有该路为 0 的约束下
（无视调整幅度）若不可行，则整体方案必然不可行，可直接判定，且能明确报告
是哪一路熄灭后无可行配方。预求解未证明最优时正常下界退化为 0（仅减弱加速
效果，不影响正确性）。

结论三态：
  * ``optimal``    —— CP-SAT 判定 OPTIMAL，返回正常方案与全部备用配方；
  * ``infeasible`` —— 判定 INFEASIBLE（含预求解证明的单场景不可行），
                      不返回正常方案，也**不返回任何部分备用配方**；
  * ``incomplete`` —— 搜索未证明最优（含超时、仅找到可行解等），同样不返回
                      任何方案。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from ortools.sat.python import cp_model

from .solver import Channel

# 状态：已证明最优 / 无完整容错方案 / 搜索未完成
RobustStatus = Literal["optimal", "infeasible", "incomplete"]

# n+1 个场景中“正常场景”的下标
NORMAL = 0
# 备用场景 f（1 起始）对应被熄灭的通道下标 f-1


@dataclass(frozen=True)
class RobustVariant:
    """一个场景（正常或某路熄灭）的配方结果。"""

    # 与入参 channels 同序的驱动级（备用场景故障路为 0）
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
    # 相对正常方案各路驱动级之差（正常场景全为 0）
    deltas: list[int]


@dataclass(frozen=True)
class RobustResult:
    status: RobustStatus
    # 仅 optimal 时有值；infeasible / incomplete 一律为空（不返回部分备用）
    normal: RobustVariant | None
    backups: list[RobustVariant]
    # 字典序 tie-break 使用的 ID 顺序
    sorted_ids: list[str]
    message: str


def solve_robust(
    channels: list[Channel],
    target: tuple[int, int, int],
    tolerance: int,
    max_adjust: int,
    time_limit_seconds: float = 10.0,
) -> RobustResult:
    """联合求解正常配方与逐路熄灭备用配方。取值合法性由 schema 先行校验。"""
    n = len(channels)
    order = sorted(range(n), key=lambda k: channels[k].id)
    max_power_num = 255 * sum(ch.power_mw for ch in channels)

    def empty(status: RobustStatus, msg: str) -> RobustResult:
        return RobustResult(
            status=status,
            normal=None,
            backups=[],
            sorted_ids=[],
            message=msg,
        )

    def add_color_window(
        m: cp_model.CpModel,
        drives: list[cp_model.IntVar],
        prefix: str,
    ) -> tuple[list[cp_model.IntVar], cp_model.IntVar]:
        """加颜色窗口（目标±容差且 0～1000，未裁剪）与功率定义。

        返回（各色整数分子变量, 功率整数分子变量）。
        """
        scaled: list[cp_model.IntVar] = []
        for c in range(3):
            # 理论上界：6 路 × 1000 × 255；域自带 0 下界
            s = m.new_int_var(0, 255 * 1000 * n, f"{prefix}_s_{c}")
            m.add(s == sum(channels[i].rgb[c] * drives[i] for i in range(n)))
            # 实际分量须落在目标 ± 容差内（不等号两边同乘 255）
            m.add(s >= 255 * (target[c] - tolerance))
            m.add(s <= 255 * (target[c] + tolerance))
            # 同时须落在 0～1000 内（下界 0 已由变量域保证，不做任何裁剪）
            m.add(s <= 255 * 1000)
            scaled.append(s)
        p = m.new_int_var(0, max_power_num, f"{prefix}_p")
        m.add(p == sum(channels[i].power_mw * drives[i] for i in range(n)))
        return scaled, p

    # ---- 预求解：各场景单独的可行性 / 正常最小功率下界 ----
    normal_lb = 0
    presolve_limit = max(1.0, time_limit_seconds / (n + 1))
    for q in range(n + 1):
        pm = cp_model.CpModel()
        pd = [pm.new_int_var(0, 255, f"pd_{q}_{i}") for i in range(n)]
        if q != NORMAL:
            # 备用预求解：只固定熄灭路为 0，暂不加调整幅度（那是耦合约束）
            pm.add(pd[q - 1] == 0)
        _, pp = add_color_window(pm, pd, f"pre_{q}")
        pm.minimize(pp)
        ps = cp_model.CpSolver()
        ps.parameters.max_time_in_seconds = presolve_limit
        ps.parameters.num_search_workers = 8
        pst = ps.solve(pm)
        if pst == cp_model.INFEASIBLE:
            if q == NORMAL:
                return empty(
                    "infeasible",
                    "无容错配方：正常场景在给定容差下就不存在任何可行驱动级"
                    "组合（CP-SAT 已证明，未裁剪公式）。可尝试放宽容差、"
                    "调整目标色或增加/更换通道。",
                )
            fid = channels[q - 1].id
            return empty(
                "infeasible",
                f"无容错配方：通道“{fid}”单独熄灭（驱动级固定为 0）后，"
                "即使不限制其余各路的调整幅度，也不存在任何使三色分量同时"
                "落入目标 ± 容差（且 0～1000、不裁剪）的驱动级组合"
                "（CP-SAT 已证明），因此无法为该路准备备用配方。"
                "可尝试放宽容差、调整目标色或增加/更换通道。",
            )
        if q == NORMAL and pst == cp_model.OPTIMAL:
            normal_lb = ps.value(pp)
        # 预求解未证明最优时正常下界退化为 0（只影响加速，不影响正确性）

    # ---- 主模型：n+1 个场景联合 ----
    m = cp_model.CpModel()
    d: list[list[cp_model.IntVar]] = []
    for q in range(n + 1):
        row = [m.new_int_var(0, 255, f"d_{q}_{i}") for i in range(n)]
        d.append(row)

    # 熄灭约束
    for f in range(1, n + 1):
        m.add(d[f][f - 1] == 0)

    # 调整幅度约束：备用各路与正常方案之差不超过上限
    # （故障路恒为 0，|0 − d0| ≤ 255 对任意 d0 天然成立，仍统一写出）
    for f in range(1, n + 1):
        for i in range(n):
            m.add(d[f][i] - d[NORMAL][i] <= max_adjust)
            m.add(d[NORMAL][i] - d[f][i] <= max_adjust)

    # 各场景颜色窗口与功率
    scaled: list[list[cp_model.IntVar]] = []
    powers: list[cp_model.IntVar] = []
    for q in range(n + 1):
        sq, pq = add_color_window(m, d[q], f"q_{q}")
        scaled.append(sq)
        powers.append(pq)

    # 目标 1：正常总功率最小（带预求解下界以加速最优性证明）
    normal_power = powers[NORMAL]
    m.add(normal_power >= normal_lb)
    m.minimize(normal_power)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit_seconds
    solver.parameters.num_search_workers = 8
    st = solver.solve(m)

    if st == cp_model.INFEASIBLE:
        return empty(
            "infeasible",
            "无容错配方：在给定容差与故障后每路最大调整级差下，不存在一套"
            "正常驱动级能使其自身以及每一路单独熄灭后的备用驱动级"
            "（故障路为 0、其余各路与正常值之差不超过上限）全部满足"
            "目标 ± 容差且 0～1000 的颜色窗口（CP-SAT 已证明不可行）。"
            "可尝试放宽容差、增大故障后调整上限、调整目标色或增加/更换通道。",
        )
    if st != cp_model.OPTIMAL:
        return empty(
            "incomplete",
            f"搜索未完成（求解器状态：{solver.status_name(st)}），未能在 "
            f"{time_limit_seconds:g}s 内证明正常总功率最小的完整容错方案，"
            "不返回正常方案或任何备用配方。可增大求解时限或调整参数后重试。",
        )
    min_power = solver.value(normal_power)

    # ---- 阶段 2：同功率下正常驱动级序列（ID 升序）字典序最小 ----
    m.add(normal_power <= min_power)
    weights = [0] * n
    for rank, idx in enumerate(order):
        weights[idx] = 256 ** (n - 1 - rank)
    # 每位数字 0～255 严格小于基数 256，加权和与字典序一一对应；
    # 6 路时最大 256^6-1 ≈ 2.8×10^14，int64 安全
    lex_key = sum(weights[i] * d[NORMAL][i] for i in range(n))
    m.minimize(lex_key)

    solver2 = cp_model.CpSolver()
    solver2.parameters.max_time_in_seconds = time_limit_seconds
    solver2.parameters.num_search_workers = 8
    st2 = solver2.solve(m)
    if st2 != cp_model.OPTIMAL:
        # 不满足“字典序最小”的方案也不将就返回，避免给出非规定方案
        return empty(
            "incomplete",
            f"搜索未完成（阶段 2 求解器状态：{solver2.status_name(st2)}），"
            f"未能在 {time_limit_seconds:g}s 内确认同功率下字典序最小的"
            "正常驱动级序列，不返回正常方案或任何备用配方。",
        )

    def variant(q: int) -> RobustVariant:
        dvals = [solver2.value(d[q][i]) for i in range(n)]
        svals = [solver2.value(scaled[q][c]) for c in range(3)]
        pnum = sum(channels[i].power_mw * dvals[i] for i in range(n))
        d0 = [solver2.value(d[NORMAL][i]) for i in range(n)]
        return RobustVariant(
            drives=dvals,
            scaled_rgb=svals,
            actual_rgb=[v / 255.0 for v in svals],
            errors=[
                (svals[c] - 255 * target[c]) / 255.0 for c in range(3)
            ],
            in_range=[
                255 * (target[c] - tolerance) <= svals[c]
                <= 255 * (target[c] + tolerance)
                and 0 <= svals[c] <= 255 * 1000
                for c in range(3)
            ],
            power_num=pnum,
            power_mw=pnum / 255.0,
            deltas=[dvals[i] - d0[i] for i in range(n)]
            if q != NORMAL
            else [0] * n,
        )

    normal_var = variant(NORMAL)
    backups = [variant(f) for f in range(1, n + 1)]
    return RobustResult(
        status="optimal",
        normal=normal_var,
        backups=backups,
        sorted_ids=[channels[i].id for i in order],
        message=(
            "求解成功：正常总功率已在“全部单路熄灭备用配方均存在”的可行"
            "正常方案中最小（CP-SAT 已证明最优），相同功率下正常驱动级"
            "序列按 ID 字典序最小；每一路单独熄灭都给出满足调整上限的"
            "可行备用配方。"
        ),
    )
