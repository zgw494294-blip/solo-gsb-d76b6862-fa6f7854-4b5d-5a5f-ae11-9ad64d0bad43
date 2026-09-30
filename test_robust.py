"""故障容错配方规划的冒烟测试（不依赖 pytest，可直接 python3 运行）。

重点：
  * 三态结论（optimal / infeasible / incomplete 的接口语义）正确；
  * 正常配方的最优性（最小正常总功率 + ID 升序字典序）用独立穷举核对：
    备用场景中故障路必须为 0，故正常驱动级必有 d0_i ≤ max_adjust，
    正常候选只有 (max_adjust+1)^n 个；逐故障场景枚举 256² 个备用对，
    即可独立判定候选是否存在满足调整上限的备用配方（3 路时完整精确）；
  * 所有备用配方逐项核对：故障路为 0、调整级差不超限、颜色窗口（整数
    分子精确判定，未裁剪）、0～1000、功率自洽；
  * 单场景预求解证明的不可行（某路熄灭后无法配色）能被识别；
  * schema 对非法参数一律 422。
"""

import itertools
import random
import sys

sys.path.insert(0, "/workspace")

from pydantic import ValidationError

from app.robust_solver import solve_robust
from app.schemas import RobustSolveRequest
from app.solver import Channel, solve as solve_plain

SAMPLE = [
    Channel("R", (1000, 120, 40), 3200),
    Channel("G", (160, 1000, 90), 2800),
    Channel("B", (60, 80, 1000), 3400),
    Channel("W", (820, 800, 760), 6500),
]


def window_ok(scaled, target, tol):
    return all(
        255 * (target[c] - tol) <= scaled[c] <= 255 * (target[c] + tol)
        and scaled[c] <= 255 * 1000
        for c in range(3)
    )


def drives_scaled(channels, drives):
    return [
        sum(channels[i].rgb[c] * drives[i] for i in range(len(channels)))
        for c in range(3)
    ]


def check_result(name, channels, target, tol, adj, res):
    """对 optimal 结果逐项核对全部场景约束与自洽性。"""
    assert res.status == "optimal", f"{name}: 期望 optimal，实得 {res.status}（{res.message}）"
    assert res.normal is not None and len(res.backups) == len(channels)
    n = len(channels)
    variants = [res.normal] + res.backups

    for q, v in enumerate(variants):
        assert len(v.drives) == n and len(v.deltas) == n
        scaled = drives_scaled(channels, v.drives)
        assert scaled == v.scaled_rgb, (q, scaled, v.scaled_rgb)
        assert all(0 <= d <= 255 for d in v.drives)
        assert window_ok(scaled, target, tol), (q, scaled)
        assert v.in_range == [True, True, True]
        pnum = sum(channels[i].power_mw * v.drives[i] for i in range(n))
        assert pnum == v.power_num
        assert abs(v.power_mw - pnum / 255.0) < 1e-12
        assert all(abs(x - s / 255.0) < 1e-12
                   for x, s in zip(v.actual_rgb, scaled))
        if q == 0:
            # 正常场景：deltas 全 0
            assert v.deltas == [0] * n
        else:
            # 故障路必须为 0；其余各路与正常值之差不超过上限
            assert v.drives[q - 1] == 0
            for i in range(n):
                assert abs(v.drives[i] - res.normal.drives[i]) <= adj, (
                    f"{name}: 故障{q} 通道{i} 调整 {v.deltas[i]} 超过上限 {adj}"
                )
            assert v.deltas == [
                v.drives[i] - res.normal.drives[i] for i in range(n)
            ]
    return variants


def _prefix2d(grid):
    """256×256 0/1 网格的二维前缀和。"""
    ps = [[0] * 257 for _ in range(257)]
    for x in range(256):
        row_sum = 0
        for y in range(256):
            row_sum += grid[x][y]
            ps[x + 1][y + 1] = ps[x][y + 1] + row_sum
    return ps


def _rect_has(ps, x0, y0, x1, y1):
    """矩形 [x0,x1]×[y0,y1] 内是否存在可行点（坐标裁剪到 0～255）。"""
    x0 = max(0, min(255, x0)); x1 = max(0, min(255, x1))
    y0 = max(0, min(255, y0)); y1 = max(0, min(255, y1))
    if x0 > x1 or y0 > y1:
        return False
    return (
        ps[x1 + 1][y1 + 1] - ps[x0][y1 + 1]
        - ps[x1 + 1][y0] + ps[x0][y0]
    ) > 0


def brute_force_robust(channels, target, tol, adj):
    """3 路独立穷举预言机：返回 (最优正常驱动, 最优功率分子)；无解返回 None。

    关键剪枝：第 i 路熄灭时备用驱动为 0，且 |0 − d0_i| ≤ adj，故正常方案
    必有 d0_i ≤ adj，正常候选仅 (adj+1)^n 个。每个故障场景把两条存活
    通道的 256² 可行对存入二维 0/1 网格，用二维前缀和做“d0 的邻域盒内是否
    存在可行备用”的常数时间查询。
    """
    n = len(channels)
    assert n == 3
    order = sorted(range(n), key=lambda k: channels[k].id)
    alive = [[i for i in range(n) if i != f] for f in range(n)]

    # 每个故障场景：两条存活通道的 256×256 可行网格（故障路为 0）
    prefixes = []
    for f in range(n):
        i1, i2 = alive[f]
        grid = [[0] * 256 for _ in range(256)]
        any_feas = False
        for x in range(256):
            for y in range(256):
                drives = [0] * n
                drives[i1] = x
                drives[i2] = y
                if window_ok(drives_scaled(channels, drives), target, tol):
                    grid[x][y] = 1
                    any_feas = True
        if not any_feas:
            return None  # 单场景本身不可行 → 整体必然不可行
        prefixes.append((i1, i2, _prefix2d(grid)))

    best = None  # (功率分子, ID 序驱动序列, 驱动)
    for d0 in itertools.product(range(adj + 1), repeat=n):
        if not window_ok(drives_scaled(channels, d0), target, tol):
            continue
        ok = True
        for i1, i2, ps in prefixes:
            if not _rect_has(ps, d0[i1] - adj, d0[i2] - adj,
                             d0[i1] + adj, d0[i2] + adj):
                ok = False
                break
        if not ok:
            continue
        pnum = sum(channels[i].power_mw * d0[i] for i in range(n))
        key = (pnum, [d0[i] for i in order])
        if best is None or key < best[0]:
            best = (key, list(d0))
    return None if best is None else (best[1], best[0][0])


def check_against_brute(name, channels, target, tol, adj, expect_feasible):
    res = solve_robust(channels, target, tol, adj, time_limit_seconds=10)
    want = brute_force_robust(channels, target, tol, adj)
    if want is None:
        assert expect_feasible is False
        assert res.status == "infeasible", f"{name}: 穷举无解，求解器却 {res.status}"
        assert res.normal is None and res.backups == []
        print(f"[OK] {name}: 穷举与求解器一致判定无完整容错方案（{adj=}）")
        return
    assert expect_feasible is True
    assert res.status == "optimal", f"{name}: 期望 optimal，实得 {res.status}"
    check_result(name, channels, target, tol, adj, res)
    want_drives, want_power = want
    assert res.normal.power_num == want_power, (
        f"{name}: 正常功率分子 {res.normal.power_num} ≠ 穷举最优 {want_power}"
    )
    order = sorted(range(len(channels)), key=lambda k: channels[k].id)
    got_seq = [res.normal.drives[i] for i in order]
    assert got_seq == [want_drives[i] for i in order], (got_seq, want_drives)
    print(f"[OK] {name}: 正常 {res.normal.drives}（功率 {want_power}/255，"
          f"按ID序 {got_seq}）与穷举最优一致；备用 "
          f"{[b.drives for b in res.backups]}")


def main():
    # 1) 同色白光灯 ×3（满档读数相同、功率不同），目标 (31,31,31) 容差 1：
    #    实际色 = 1000·(驱动级之和)/255，窗口 [30,32] 恰要求驱动级之和 = 8
    #    （和 7 → 27.45 超下限；和 9 → 35.29 超上限），可手工 + 穷举核对
    white3 = [
        Channel("A", (1000, 1000, 1000), 3000),
        Channel("B", (1000, 1000, 1000), 2500),
        Channel("C", (1000, 1000, 1000), 4000),
    ]
    # adj=8：正常方案把 8 级都放在最便宜的 B 上（0,8,0）；
    # 任一故障下另外两路可在 ±8 内重新分配出和 8
    check_against_brute("同色白光-adj8", white3, (31, 31, 31), 1, 8, True)
    # 同例 ID 逆序输入，tie-break/最优性不变
    check_against_brute("同色白光-adj8-ID逆序", list(reversed(white3)),
                        (31, 31, 31), 1, 8, True)
    # adj=2：正常每路必须 ≤2（故障路要降到 0），正常和至多 6 < 8 → 不可行。
    # 但单场景预求解都可行（两路满档足够配出 (31,31,31)），不可行性来自耦合
    res = solve_robust(white3, (31, 31, 31), 1, 2)
    assert res.status == "infeasible" and res.normal is None and res.backups == []
    print("[OK] 调整上限过小=无完整容错方案（耦合不可行，非单场景不可行）")

    # 2) 纯 RGB 三路：任一颜色路熄灭后无人能补该色 → 预求解即证明不可行
    rgb3 = [
        Channel("R", (1000, 0, 0), 1000),
        Channel("G", (0, 1000, 0), 1000),
        Channel("B", (0, 0, 1000), 1000),
    ]
    res = solve_robust(rgb3, (500, 500, 500), 12, 255)
    assert res.status == "infeasible", res.status
    assert res.normal is None and res.backups == []
    print("[OK] 纯RGB缺色=无完整容错方案，不返回任何正常/备用驱动级")

    # 3) 正常场景本身不可行
    res = solve_robust(rgb3, (500, 500, 500), 1, 255)
    assert res.status == "infeasible" and res.backups == []
    print("[OK] 正常场景无可行配方=容错方案不可行")

    # 4) 4 路示例通道（穷举 256^4 代价高，做全约束精确核对而非最优性穷举）
    res = solve_robust(SAMPLE, (500, 500, 500), 60, 80)
    assert res.status == "optimal", res.message
    check_result("示例4路容错-adj80", SAMPLE, (500, 500, 500), 60, 80, res)
    print(f"[OK] 示例4路容错-adj80: 正常 {res.normal.drives} "
          f"功率 {res.normal.power_num}/255；逐路备用 "
          f"{[b.drives for b in res.backups]}")

    # 4b) 同例 adj=255 可行；adj=30 不可行（补偿级差不够）
    res255 = solve_robust(SAMPLE, (500, 500, 500), 60, 255)
    assert res255.status == "optimal"
    check_result("示例4路容错-adj255", SAMPLE, (500, 500, 500), 60, 255, res255)
    res30 = solve_robust(SAMPLE, (500, 500, 500), 60, 30)
    assert res30.status == "infeasible" and res30.normal is None
    print("[OK] 示例4路: adj255 有完整方案，adj30 无完整方案")

    # 4c) adj=255 的正常最优必须与普通配方最优一致（备用无额外限制时）
    plain = solve_plain(SAMPLE, (500, 500, 500), 60)
    assert plain is not None
    assert res255.normal.drives == plain.drives, (
        res255.normal.drives, plain.drives
    )
    assert res255.normal.power_num == plain.power_num
    print("[OK] adj=255 时容错正常方案与普通最优配方（含字典序）完全一致")

    # 5) 手工构造的非平凡 3 路实例：通道接近白光但读数/功率各不相同。
    #    目标为低亮度白；正常驱动级必须 ≤ adj（故障路要降到 0）。
    #    目标/容差由穷举预言机确认可解，覆盖窄容差（2）与严格色阶。
    crafted = [
        (
            "近白光-读数各异",
            [Channel("A", (1000, 950, 900), 3000),
             Channel("B", (880, 1000, 920), 2500),
             Channel("C", (940, 900, 1000), 4000)],
            (26, 25, 27), 2, 12,
        ),
        (
            "近白光-ID逆序(X/Y/Z)",
            [Channel("Z", (1000, 800, 700), 2000),
             Channel("Y", (700, 1000, 800), 5000),
             Channel("X", (800, 700, 1000), 3500)],
            (20, 20, 20), 2, 12,
        ),
        (
            "同白光-大上限验证字典序",
            [Channel("A", (1000, 1000, 1000), 3000),
             Channel("B", (1000, 1000, 1000), 2500),
             Channel("C", (1000, 1000, 1000), 4000)],
            # 实际色 = 1000·驱动级之和/255；窗口 [54,56] 恰要求和为 14
            (55, 55, 55), 1, 20,
        ),
    ]
    for cname, chs, target, tol, adj in crafted:
        # 先确认预言机认为有解（窗口按整数分子判定，容差需真的覆盖离散色阶）
        want = brute_force_robust(chs, target, tol, adj)
        assert want is not None, f"{cname}: 构造实例在穷举下无解，需调整目标/容差"
        check_against_brute(cname, chs, target, tol, adj, True)

    # 5b) 随机 3 路实例用大上限 + 较宽容差提高命中率：仍用穷举预言机全面核对
    rng = random.Random(20260930)
    found = 0
    tried = 0
    while found < 2 and tried < 120:
        tried += 1
        chs = [
            Channel(cid, tuple(rng.randint(300, 1000) for _ in range(3)),
                    rng.randint(100, 9000))
            for cid in ("A", "B", "C")
        ]
        target = (rng.randint(120, 300), rng.randint(120, 300),
                  rng.randint(120, 300))
        adj = rng.choice((40, 60, 90))
        tol = rng.choice((8, 12, 20))
        want = brute_force_robust(chs, target, tol, adj)
        if want is None:
            continue
        check_against_brute(f"随机3路实例#{found + 1}", chs, target, tol, adj, True)
        found += 1
    assert found == 2, f"随机搜索只找到 {found} 个可行实例（试了 {tried} 次）"

    # 6) 6 路可解且全部备用约束成立
    six = SAMPLE + [Channel("W2", (700, 720, 750), 5000),
                    Channel("Amber", (950, 600, 60), 2600)]
    res = solve_robust(six, (500, 500, 500), 60, 120)
    assert res.status == "optimal", res.message
    check_result("6路容错", six, (500, 500, 500), 60, 120, res)
    print(f"[OK] 6路容错: 正常 {res.normal.drives}，{len(res.backups)} 个备用配方齐备")

    # 7) schema 非法参数全部 422
    CH4 = [
        {"id": "R", "r": 1000, "g": 120, "b": 40, "power_mw": 3200},
        {"id": "G", "r": 160, "g": 1000, "b": 90, "power_mw": 2800},
        {"id": "B", "r": 60, "g": 80, "b": 1000, "power_mw": 3400},
        {"id": "W", "r": 820, "g": 800, "b": 760, "power_mw": 6500},
    ]
    BASE = dict(
        channels=CH4,
        target_r=500, target_g=500, target_b=500,
        tolerance=12, max_adjust=60,
    )
    RobustSolveRequest(**BASE)  # 基线合法

    def reject(name, **body):
        try:
            RobustSolveRequest(**body)
        except ValidationError:
            print(f"[OK] {name}: 422 拒绝")
            return
        raise AssertionError(f"{name}: 应拒绝却通过")

    reject("调整上限 0", **{**BASE, "max_adjust": 0})
    reject("调整上限 256", **{**BASE, "max_adjust": 256})
    reject("缺少调整上限", **{k: v for k, v in BASE.items() if k != "max_adjust"})
    reject("通道数 2", **{**BASE, "channels": CH4[:2]})
    reject("通道数 7", **{**BASE, "channels": CH4 + CH4[:3]})
    reject("ID 重复", **{**BASE,
                        "channels": [CH4[0]] + CH4[1:3] + [CH4[0]] + CH4[3:]})
    reject("目标 1001", **{**BASE, "target_r": 1001})
    reject("容差 21", **{**BASE, "tolerance": 21})
    reject("容差 -1", **{**BASE, "tolerance": -1})

    print("\n全部通过 ✔")


if __name__ == "__main__":
    main()
