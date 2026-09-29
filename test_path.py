"""调光路径规划的冒烟测试（不依赖 pytest，可直接 python3 运行）。

重点：
  * 三态结论（optimal / infeasible / incomplete 的接口语义）正确；
  * 最优性用「全空间穷举 + 动态规划」独立核对（2 路 × 256² 可枚举）；
  * 窗口约束按整数分子精确核对（参考色不先取整）；
  * 爬坡约束覆盖首步与起点；
  * schema 对非法路径参数一律 422。
"""

import itertools
import sys

sys.path.insert(0, "/workspace")

from pydantic import ValidationError

from app.path_solver import solve_path
from app.schemas import PathSolveRequest
from app.solver import Channel

SAMPLE = [
    Channel("R", (1000, 120, 40), 3200),
    Channel("G", (160, 1000, 90), 2800),
    Channel("B", (60, 80, 1000), 3400),
    Channel("W", (820, 800, 760), 6500),
]


def scaled0(channels, start):
    return [
        sum(ch.rgb[c] * d for ch, d in zip(channels, start)) for c in range(3)
    ]


def check_path(name, channels, start, target, steps, delta, tol):
    """求解并逐项精确核对约束与返回值自洽性。"""
    res = solve_path(channels, start, target, steps, delta, tol)
    assert res.status == "optimal", f"{name}: 期望 optimal，实得 {res.status}（{res.message}）"
    assert len(res.steps) == steps
    n = len(channels)
    s0 = scaled0(channels, start)
    assert res.start_drives == list(start)
    for c in range(3):
        assert abs(res.start_actual_rgb[c] - s0[c] / 255.0) < 1e-12

    total = 0
    for k, st in enumerate(res.steps, start=1):
        # 1) 驱动级取值与爬坡约束（首步相对起点）
        prev = start if k == 1 else res.steps[k - 2].drives
        for i in range(n):
            assert 0 <= st.drives[i] <= 255
            assert abs(st.drives[i] - prev[i]) <= delta, (
                f"{name}: 第{k}步通道{i}变化 {abs(st.drives[i]-prev[i])} > {delta}"
            )
        # 2) 整数分子与功率自洽
        for c in range(3):
            s = sum(channels[i].rgb[c] * st.drives[i] for i in range(n))
            assert s == st.scaled_rgb[c]
        pnum = sum(channels[i].power_mw * st.drives[i] for i in range(n))
        assert pnum == st.power_num
        assert abs(st.power_mw - pnum / 255.0) < 1e-12
        total += pnum
        # 3) 窗口约束（整数精确）：|K·s − T| ≤ 255·K·tol 且 0 ≤ s ≤ 255000
        for c in range(3):
            t = (steps - k) * s0[c] + 255 * k * target[c]
            assert abs(steps * st.scaled_rgb[c] - t) <= 255 * steps * tol
            assert 0 <= st.scaled_rgb[c] <= 255 * 1000
            assert st.in_range[c] is True
            # 4) 参考色不先取整：ref = T/(255K)，误差 = 实际 − 参考
            assert abs(st.ref_rgb[c] - t / (255.0 * steps)) < 1e-12
            assert abs(st.actual_rgb[c] - st.scaled_rgb[c] / 255.0) < 1e-12
            assert abs(st.errors[c] - (st.actual_rgb[c] - st.ref_rgb[c])) < 1e-9
    # 5) 各步功率之和 = 总功率
    assert total == res.total_power_num
    assert abs(res.total_power_mw - total / 255.0) < 1e-12
    print(f"[OK] {name}: {steps} 步, 总功率 {total}/255="
          f"{round(total / 255, 4)} mW, 各步驱动 "
          f"{[st.drives for st in res.steps]}")
    return res


def brute_force_min_power(channels, start, target, steps, delta, tol):
    """独立核对：逐层枚举每步可行驱动元组，再按爬坡约束做动态规划。

    返回最小总功率分子；不存在完整路径时返回 None。
    """
    n = len(channels)
    s0 = scaled0(channels, start)
    per_step = []  # 每步 {驱动元组: 功率分子}
    for k in range(1, steps + 1):
        feas = {}
        for combo in itertools.product(range(256), repeat=n):
            ok = True
            for c in range(3):
                s = sum(channels[i].rgb[c] * combo[i] for i in range(n))
                t = (steps - k) * s0[c] + 255 * k * target[c]
                if not (abs(steps * s - t) <= 255 * steps * tol
                        and 0 <= s <= 255 * 1000):
                    ok = False
                    break
            if ok:
                feas[combo] = sum(
                    channels[i].power_mw * combo[i] for i in range(n)
                )
        per_step.append(feas)
    if any(not f for f in per_step):
        return None
    # 首步：与起点的爬坡约束
    prev_cost = {
        combo: p for combo, p in per_step[0].items()
        if all(abs(combo[i] - start[i]) <= delta for i in range(n))
    }
    for k in range(1, steps):
        cur = {}
        for combo, p in per_step[k].items():
            best = None
            for pc, cost in prev_cost.items():
                if all(abs(combo[i] - pc[i]) <= delta for i in range(n)):
                    v = cost + p
                    if best is None or v < best:
                        best = v
            if best is not None:
                cur[combo] = best
        prev_cost = cur
        if not prev_cost:
            return None
    return min(prev_cost.values()) if prev_cost else None


def check_against_brute(name, channels, start, target, steps, delta, tol):
    res = solve_path(channels, start, target, steps, delta, tol)
    want = brute_force_min_power(channels, start, target, steps, delta, tol)
    if want is None:
        assert res.status == "infeasible" and res.steps == [], (
            f"{name}: 穷举判定不可行，求解器却给出 {res.status}"
        )
        print(f"[OK] {name}: 穷举与求解器一致判定无完整路径")
        return
    assert res.status == "optimal", f"{name}: 期望 optimal，实得 {res.status}"
    assert res.total_power_num == want, (
        f"{name}: 总功率分子 {res.total_power_num} ≠ 穷举最优 {want}"
    )
    print(f"[OK] {name}: 总功率分子 {want} 与穷举+DP 独立核对一致")


# 1) 页面示例通道：4 路 × 3 步，全约束精确核对
check_path("示例4路3步", SAMPLE, [0, 0, 0, 0], (600, 420, 180), 3, 100, 12)

# 1b) 同例：5 步、小变化上限（爬坡约束逐步生效）
check_path("示例4路5步小步长", SAMPLE, [10, 200, 0, 40], (600, 420, 180), 5, 45, 12)

# 1c) 6 路、起点低亮度（起点实际色约 (434, 391, 318)，各步窗口均在 0～1000 内）
six = SAMPLE + [Channel("W2", (700, 720, 750), 5000),
                Channel("Amber", (950, 600, 60), 2600)]
check_path("6路2步", six, [30, 30, 30, 30, 30, 30], (500, 500, 500), 2, 255, 15)

# 2) 最优性独立核对（2 路可全枚举；solver 本身不限制路数）
#    用例沿「起点→终点驱动级直线」构造，保证完整路径存在
two = [Channel("A", (500, 300, 100), 2000), Channel("B", (100, 400, 900), 3000)]
check_against_brute("穷举核对-2路2步", two, [10, 200], (255, 286, 396), 2, 50, 1)
check_against_brute("穷举核对-2路3步", two, [250, 0], (255, 353, 569), 3, 50, 1)
check_against_brute("穷举核对-2路4步零容差终点",
                    [Channel("A", (510, 0, 0), 1500), Channel("B", (0, 510, 510), 2500)],
                    [0, 0], (300, 200, 200), 4, 255, 4)

# 3) 不可行：每步变化上限太小，K 步内到不了终点窗口
res = solve_path(
    [Channel("R", (1000, 0, 0), 1000),
     Channel("G", (0, 1000, 0), 1000),
     Channel("B", (0, 0, 1000), 1000)],
    [0, 0, 0], (500, 500, 500), 2, 10, 12,
)
assert res.status == "infeasible", res.status
assert res.steps == [] and res.total_power_num is None and res.total_power_mw is None
print("[OK] 步长不足=无完整路径，不返回任何局部步骤")

# 3b) 不可行：容差 0 且参考色无法用整数驱动级精确命中
res = solve_path(
    [Channel("R", (1000, 0, 0), 1000),
     Channel("G", (0, 1000, 0), 1000),
     Channel("B", (0, 0, 1000), 1000)],
    [0, 0, 0], (333, 333, 333), 2, 255, 0,
)
assert res.status == "infeasible", res.status
assert res.steps == []
print("[OK] 零容差非整除=无完整路径")

# 4) 起点即终点（整除目标）、容差 0：唯一路径是保持不动
res = solve_path(
    [Channel("R", (1000, 0, 0), 1000),
     Channel("G", (0, 1000, 0), 1000),
     Channel("B", (0, 0, 1000), 1000)],
    [51, 102, 153], (200, 400, 600), 3, 5, 0,
)
assert res.status == "optimal", res.status
for st in res.steps:
    assert st.drives == [51, 102, 153], st.drives
    assert st.errors == [0.0, 0.0, 0.0], st.errors
assert res.total_power_num == 3 * (1000 * (51 + 102 + 153))
print("[OK] 起点即终点零容差: 各步保持不动, 误差为 0")

# 5) 参考色不先取整：起点实际色为分数，窗口按分数参考色判定
#    起点 (1,0,0) → A0 = (1000/255, 0, 0)；K=2 时第 1 步参考 R = 500/255 ≈ 1.9608
res = solve_path(
    [Channel("R", (1000, 0, 0), 1000),
     Channel("G", (0, 1000, 0), 1000),
     Channel("B", (0, 0, 1000), 1000)],
    [1, 0, 0], (0, 0, 0), 2, 255, 2,
)
assert res.status == "optimal", res.status
assert abs(res.steps[0].ref_rgb[0] - 500 / 255.0) < 1e-12
assert abs(res.steps[1].ref_rgb[0] - 0.0) < 1e-12
# 第 1 步窗口 |2s − 1000| ≤ 1020 允许 s ∈ {0, 1000}（d ∈ {0,1}），
# 最小功率选 d = 0；误差 = 实际 − 未取整参考 = −500/255
assert res.steps[0].drives[0] == 0
assert abs(res.steps[0].errors[0] - (-500 / 255.0)) < 1e-9
print("[OK] 分数参考色不取整: 第1步参考 R = 500/255 ≈ 1.961, 误差按未取整参考计算")

# 6) schema 非法参数全部 422
def reject(name, **body):
    try:
        PathSolveRequest(**body)
    except ValidationError:
        print(f"[OK] {name}: 422 拒绝")
        return
    raise AssertionError(f"{name}: 应拒绝却通过")


CH3 = [
    {"id": "R", "r": 1000, "g": 0, "b": 0, "power_mw": 1000},
    {"id": "G", "r": 0, "g": 1000, "b": 0, "power_mw": 1000},
    {"id": "B", "r": 0, "g": 0, "b": 1000, "power_mw": 1000},
]
BASE = dict(
    channels=CH3,
    start_drives={"R": 0, "G": 0, "B": 0},
    target_r=500, target_g=500, target_b=500,
    steps=3, max_step_delta=100, tolerance=12,
)
PathSolveRequest(**BASE)  # 基线合法
reject("通道数 2", **{**BASE, "channels": CH3[:2],
                     "start_drives": {"R": 0, "G": 0}})
reject("ID 重复", **{**BASE,
                     "channels": [CH3[0], CH3[0], CH3[2]]})
reject("缺少起始驱动级", **{**BASE, "start_drives": {"R": 0, "G": 0}})
reject("起始驱动级多余ID", **{**BASE,
                              "start_drives": {"R": 0, "G": 0, "B": 0, "X": 1}})
reject("起始驱动级 256", **{**BASE,
                            "start_drives": {"R": 256, "G": 0, "B": 0}})
reject("起始驱动级 -1", **{**BASE,
                           "start_drives": {"R": -1, "G": 0, "B": 0}})
reject("步骤数 1", **{**BASE, "steps": 1})
reject("步骤数 6", **{**BASE, "steps": 6})
reject("变化上限 0", **{**BASE, "max_step_delta": 0})
reject("变化上限 256", **{**BASE, "max_step_delta": 256})
reject("终点 1001", **{**BASE, "target_r": 1001})
reject("终点 -1", **{**BASE, "target_g": -1})
reject("容差 21", **{**BASE, "tolerance": 21})

print("\n全部通过 ✔")
