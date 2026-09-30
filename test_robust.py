"""容错配方规划的冒烟测试（不依赖 pytest，可直接 python3 运行）。

重点：
  * 三态结论（optimal / infeasible / incomplete 的接口语义）正确；
  * 最优性用「全空间穷举」独立核对（2 路 × 256² 可枚举）；
  * 正常与每一路熄灭备用配方都落在目标 ± 容差且 0～1000 内（未裁剪公式）；
  * 故障路恒为 0、其余各路与正常值之差不超过调整上限；
  * 同功率正常驱动序列按 ID 升序取字典序最小；
  * 无完整方案/未证明最优时不返回任何部分备用；
  * schema 对非法参数一律 422。
"""

import itertools
import sys

sys.path.insert(0, "/workspace")

from pydantic import ValidationError

from app.robust_solver import solve_robust
from app.schemas import RobustSolveRequest
from app.solver import Channel


def in_window(channels, drives, target, tol):
    for c in range(3):
        s = sum(channels[i].rgb[c] * drives[i] for i in range(len(channels)))
        if not (255 * (target[c] - tol) <= s <= 255 * (target[c] + tol)
                and 0 <= s <= 255 * 1000):
            return False
    return True


def check_robust(name, channels, target, tol, limit):
    """求解并逐项精确核对正常/备用配方的全部约束与返回值自洽性。"""
    n = len(channels)
    res = solve_robust(channels, target, tol, limit)
    assert res.status == "optimal", f"{name}: 期望 optimal，实得 {res.status}（{res.message}）"
    assert res.normal is not None and len(res.fallbacks) == n
    assert res.channel_ids == [ch.id for ch in channels]
    assert res.sorted_ids == [ch.id for ch in sorted(channels, key=lambda c: c.id)]

    nd = res.normal.drives
    assert len(nd) == n and all(0 <= d <= 255 for d in nd)
    assert in_window(channels, nd, target, tol), f"{name}: 正常配方超差"
    assert all(res.normal.in_range)
    for c in range(3):
        s = sum(channels[i].rgb[c] * nd[i] for i in range(n))
        assert res.normal.scaled_rgb[c] == s
        assert abs(res.normal.actual_rgb[c] - s / 255.0) < 1e-12
        assert abs(res.normal.errors[c] - (s / 255.0 - target[c])) < 1e-9
    pnum = sum(channels[i].power_mw * nd[i] for i in range(n))
    assert res.normal.power_num == pnum
    assert abs(res.normal.power_mw - pnum / 255.0) < 1e-12

    for f, fb in enumerate(res.fallbacks):
        fd = fb.drives
        assert len(fd) == n and all(0 <= d <= 255 for d in fd)
        # 故障路必须为 0
        assert fd[f] == 0, f"{name}: 通道 {channels[f].id} 熄灭后驱动级非 0"
        # 其余各路与正常值之差不超过上限
        for i in range(n):
            if i == f:
                continue
            assert abs(fd[i] - nd[i]) <= limit, (
                f"{name}: {channels[f].id} 熄灭后通道 {channels[i].id} "
                f"调整 {abs(fd[i] - nd[i])} > {limit}"
            )
        # 备用配方同样落在目标 ± 容差且 0～1000 内
        assert in_window(channels, fd, target, tol), f"{name}: {channels[f].id} 备用配方超差"
        assert all(fb.in_range)
        for c in range(3):
            s = sum(channels[i].rgb[c] * fd[i] for i in range(n))
            assert fb.scaled_rgb[c] == s
            assert abs(fb.actual_rgb[c] - s / 255.0) < 1e-12
            assert abs(fb.errors[c] - (s / 255.0 - target[c])) < 1e-9
        fp = sum(channels[i].power_mw * fd[i] for i in range(n))
        assert fb.power_num == fp
        assert abs(fb.power_mw - fp / 255.0) < 1e-12
    print(f"[OK] {name}: 正常 {nd}, 功率 {pnum}/255={round(pnum/255, 4)} mW; "
          f"备用 {[fb.drives for fb in res.fallbacks]}")
    return res


def brute_robust(channels, target, tol, limit):
    """独立穷举：枚举正常配方与每路备用配方，求（功率, ID序驱动）最小者。

    返回 ((power_num, lex_drives), drives)；无完整方案返回 None。
    """
    n = len(channels)

    def win(d):
        return in_window(channels, d, target, tol)

    # 每路故障下的可行备用集合
    fb_sets = []
    for f in range(n):
        opts = {
            combo for combo in itertools.product(range(256), repeat=n)
            if combo[f] == 0 and win(combo)
        }
        if not opts:
            return None
        fb_sets.append(opts)

    order = sorted(range(n), key=lambda k: channels[k].id)
    best = None
    for d0 in itertools.product(range(256), repeat=n):
        if not win(d0):
            continue
        ok = True
        for f in range(n):
            if not any(
                all(i == f or abs(combo[i] - d0[i]) <= limit for i in range(n))
                for combo in fb_sets[f]
            ):
                ok = False
                break
        if not ok:
            continue
        p = sum(channels[i].power_mw * d0[i] for i in range(n))
        key = (p, [d0[i] for i in order])
        if best is None or key < best[0]:
            best = (key, list(d0))
    return best


def check_against_brute(name, channels, target, tol, limit):
    res = solve_robust(channels, target, tol, limit)
    want = brute_robust(channels, target, tol, limit)
    if want is None:
        assert res.status == "infeasible" and res.normal is None and res.fallbacks == [], (
            f"{name}: 穷举判定无完整方案，求解器却给出 {res.status}"
        )
        print(f"[OK] {name}: 穷举与求解器一致判定无完整容错配方（不返回部分备用）")
        return
    assert res.status == "optimal", f"{name}: 期望 optimal，实得 {res.status}"
    order = sorted(range(len(channels)), key=lambda k: channels[k].id)
    got_key = (
        res.normal.power_num,
        [res.normal.drives[i] for i in order],
    )
    assert got_key == want[0], f"{name}: {got_key} ≠ 穷举最优 {want[0]}"
    print(f"[OK] {name}: （功率 {want[0][0]}, 字典序 {want[0][1]}）与穷举独立核对一致")


# 1) 4 路高冗余示例（中性目标，Δ=60）
sample4 = [
    Channel("R", (1000, 200, 100), 3200),
    Channel("G", (200, 1000, 150), 2800),
    Channel("B", (100, 150, 1000), 3400),
    Channel("W", (850, 830, 800), 6500),
]
check_robust("示例4路Δ60", sample4, (500, 500, 500), 20, 60)
# 1b) 同例：放宽调整上限后正常配方可以更省（仅核对约束自洽）
check_robust("示例4路Δ120", sample4, (500, 500, 500), 20, 120)
# 1c) 3 路近白宽光谱通道（容差较宽时缺一路可由另两路补偿）
white3 = [
    Channel("W1", (900, 880, 850), 3000),
    Channel("W2", (860, 900, 870), 2800),
    Channel("W3", (830, 860, 900), 3400),
]
check_robust("3路近白Δ120", white3, (450, 450, 450), 20, 120)
# 1d) 6 路、更紧的调整上限
six = sample4 + [
    Channel("W2x", (700, 720, 750), 5000),
    Channel("Amber", (950, 600, 60), 2600),
]
check_robust("6路Δ30", six, (500, 500, 500), 20, 30)

# 2) 最优性 + 字典序独立核对（2 路可全枚举；solver 不限制路数）
two = [Channel("A", (500, 500, 500), 2000), Channel("B", (500, 500, 500), 3000)]
check_against_brute("穷举核对-2路容差0", two, (200, 200, 200), 0, 80)
check_against_brute("穷举核对-2路容差3", two, (200, 200, 200), 3, 50)
check_against_brute("穷举核对-2路容差1", two, (300, 300, 300), 1, 120)
# 穷举为不可行（Δ 太小）
check_against_brute("穷举核对-2路Δ不足", two, (400, 400, 400), 0, 100)
# 同功率不同功率系数 → 字典序把驱动压向更小的 A 通道
check_against_brute("穷举核对-2路同功率",
                    [Channel("A", (500, 500, 500), 1000),
                     Channel("B", (500, 500, 500), 1000)],
                    (200, 200, 200), 0, 255)

# 2b) ID 逆序输入不影响“按 ID 字符串升序”的字典序
two_rev = list(reversed(two))
res = check_robust("2路ID逆序", two_rev, (200, 200, 200), 0, 80)
assert res.sorted_ids == ["A", "B"]

# 3) 纯 RGB 三路、窄容差目标：丢失任一路都无法补偿 => 无完整方案
pure = [
    Channel("R", (1000, 0, 0), 1000),
    Channel("G", (0, 1000, 0), 1000),
    Channel("B", (0, 0, 1000), 1000),
]
res = solve_robust(pure, (400, 400, 400), 5, 255)
assert res.status == "infeasible", res.status
assert res.normal is None and res.fallbacks == []
print("[OK] 纯RGB三路缺一路无法补偿=无完整容错配方，不返回部分备用")

# 4) 边界：目标 (0,0,0) 容差 0 => 正常与全部备用都是全 0，功率 0
res = solve_robust(sample4, (0, 0, 0), 0, 1)
assert res.status == "optimal", res.status
assert res.normal.drives == [0, 0, 0, 0]
assert res.normal.power_num == 0
for fb in res.fallbacks:
    assert fb.drives == [0, 0, 0, 0] and fb.power_num == 0
print("[OK] 零目标零容差: 正常与全部备用均全零, 功率 0")

# 5) Δ 恰好等于所需调整量：(400,400,400) 容差0，两路互备需各上 204，正常 102/102
res = solve_robust(two, (400, 400, 400), 0, 102)
assert res.status == "optimal", res.status
assert res.normal.drives == [102, 102], res.normal.drives
for f, fb in enumerate(res.fallbacks):
    assert fb.drives[f] == 0 and fb.drives[1 - f] == 204
print("[OK] Δ 恰好可达: 正常 102/102，备用一路 204")

# 6) 正常配方本身超差无可行（与单配方求解一致）：零容差非整除
res = solve_robust(
    pure, (333, 0, 0), 0, 255,
)
assert res.status == "infeasible", res.status
print("[OK] 正常配方本身不可行=无完整容错配方")

# 7) schema 非法参数全部 422
def reject(name, **body):
    try:
        RobustSolveRequest(**body)
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
    target_r=400, target_g=400, target_b=400,
    tolerance=10, max_adjust_delta=80,
)
RobustSolveRequest(**BASE)  # 基线合法
reject("通道数 2", **{**BASE, "channels": CH3[:2]})
reject("通道数 7", **{**BASE, "channels": CH3 + [
    {"id": f"C{i}", "r": 1, "g": 1, "b": 1, "power_mw": 1000} for i in range(4)
]})
reject("ID 重复", **{**BASE, "channels": [CH3[0], CH3[0], CH3[2]]})
reject("满档RGB全零", **{**BASE,
                        "channels": [{"id": "Z", "r": 0, "g": 0, "b": 0,
                                      "power_mw": 1000}] + CH3[1:]})
reject("调整上限 0", **{**BASE, "max_adjust_delta": 0})
reject("调整上限 256", **{**BASE, "max_adjust_delta": 256})
reject("目标 1001", **{**BASE, "target_r": 1001})
reject("目标 -1", **{**BASE, "target_b": -1})
reject("容差 21", **{**BASE, "tolerance": 21})
reject("功率 0", **{**BASE,
                    "channels": [{"id": "R", "r": 1000, "g": 0, "b": 0,
                                  "power_mw": 0}] + CH3[1:]})

print("\n全部通过 ✔")
