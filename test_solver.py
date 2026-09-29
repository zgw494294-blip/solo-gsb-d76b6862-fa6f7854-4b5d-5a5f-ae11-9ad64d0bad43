"""求解器正确性冒烟测试（不依赖 pytest）。"""

import sys

sys.path.insert(0, "/workspace")

from app.solver import Channel, solve  # noqa: E402


def actual(channels, drives):
    rgb = [0, 0, 0]
    power = 0
    for ch, d in zip(channels, drives):
        for c in range(3):
            rgb[c] += ch.rgb[c] * d
        power += ch.power_mw * d
    return rgb, power


def check(name, channels, target, tol, expect_feasible):
    sol = solve(channels, target, tol, time_limit_seconds=5)
    if not expect_feasible:
        assert sol is None, f"{name}: 期望无解，但返回了方案"
        print(f"[OK] {name}: 正确判定无可行解")
        return
    assert sol is not None, f"{name}: 期望有解，但返回 None"

    # 1) 约束核对（用整数分子精确核对）
    rgb_num, power_num = actual(channels, sol.drives)
    assert rgb_num == sol.scaled_rgb, (rgb_num, sol.scaled_rgb)
    assert power_num == sol.power_num
    for c in range(3):
        a = rgb_num[c] / 255
        assert target[c] - tol - 1e-9 <= a <= target[c] + tol + 1e-9, (c, a)
        assert 0 <= a <= 1000
    assert sol.power_mw == power_num / 255

    # 2) 穷举小例子验证最优性 + 字典序（3 路 × 256^3 可行，直接枚举）
    if len(channels) == 3:
        best = None
        for d0 in range(256):
            for d1 in range(256):
                for d2 in range(256):
                    ds = [d0, d1, d2]
                    num, pn = actual(channels, ds)
                    ok = all(
                        255 * (target[c] - tol) <= num[c] <= 255 * (target[c] + tol)
                        and num[c] <= 255 * 1000
                        for c in range(3)
                    )
                    if not ok:
                        continue
                    order = sorted(range(3), key=lambda i: channels[i].id)
                    key = (pn, [ds[i] for i in order])
                    if best is None or key < best[0]:
                        best = (key, ds, num, pn)
        assert best is not None
        order = sorted(range(3), key=lambda i: channels[i].id)
        assert power_num == best[3], (power_num, best[3])
        assert [sol.drives[i] for i in order] == best[0][1], \
            ([sol.drives[i] for i in order], best[0][1])

    print(f"[OK] {name}: drives={sol.drives} (按ID序 {sol.sorted_ids} → "
          f"{[sol.drives[channels.index(next(x for x in channels if x.id==i))] for i in sol.sorted_ids]}), "
          f"actual={[round(x,3) for x in sol.actual_rgb]}, "
          f"power={sol.power_num}/255={round(sol.power_mw,4)} mW")


sample = [
    Channel("R", (1000, 120, 40), 3200),
    Channel("G", (160, 1000, 90), 2800),
    Channel("B", (60, 80, 1000), 3400),
    Channel("W", (820, 800, 760), 6500),
]

# 1) 页面示例（4 路，应可行）
check("示例4路", sample, (600, 420, 180), 12, True)

# 2) 小例子：3 路 + 穷举验证最优与字典序
small = [
    Channel("A", (900, 100, 50), 3000),
    Channel("B", (100, 900, 200), 2500),
    Channel("C", (300, 400, 800), 4000),
]
check("穷举对照3路", small, (300, 250, 200), 20, True)

# 2b) ID 逆序输入，tie-break 仍按 ID 字符串序
small_rev = list(reversed(small))
check("穷举对照3路-ID逆序", small_rev, (300, 250, 200), 20, True)

# 3) 无解：纯 RGB 三路，目标 (500,500,500) 容差 1 ——
#    1000*d/255 最接近 500 的值为 498.039 / 501.961，误差 1.961 > 1
check("不可行-非整除窄容差",
      [Channel("R", (1000, 0, 0), 1000),
       Channel("G", (0, 1000, 0), 1000),
       Channel("B", (0, 0, 1000), 1000)],
      (500, 500, 500), 1, False)

# 4) 不可行：容差 0，非整除目标（1000*d/255 无法等于 333）
check("不可容-整除",
      [Channel("R", (1000, 0, 0), 1000),
       Channel("G", (0, 1000, 0), 1000),
       Channel("B", (0, 0, 1000), 1000)],
      (333, 0, 0), 0, False)

# 5) 6 路可解
six = sample + [Channel("W2", (700, 720, 750), 5000),
                Channel("Amber", (950, 600, 60), 2600)]
check("6路", six, (500, 500, 500), 15, True)

# 6) 边界：全零目标容差0 => 所有驱动级 0，功率 0
check("零目标零容差", sample, (0, 0, 0), 0, True)

print("\n全部通过 ✔")
