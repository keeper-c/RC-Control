"""
Simulation of offset coordinate calculation (抵偿坐标计算)
Compares buggy (missing parentheses) vs corrected formula.
"""
import math

def offset_buggy(inst_x, inst_y, self_x, self_y, target_x, target_y):
    """Original buggy code: k = -(target.x - inst.x / target.y - inst.y)"""
    x0, y0 = self_x, self_y
    k = -(target_x - inst_x / target_y - inst_y)  # BUG: precedence error
    b = (target_x * inst_y - inst_x * target_y) / (inst_y - target_y)
    ox = (b + k * k * x0 - k * y0) / (1 + k * k)
    oy = (y0 + k * b - k * x0) / (1 + k * k)
    return ox, oy, k, b

def offset_fixed(inst_x, inst_y, self_x, self_y, target_x, target_y):
    """Corrected formula, handles horizontal/vertical via special cases."""
    x0, y0 = self_x, self_y
    dx = target_x - inst_x
    dy = target_y - inst_y

    # Vertical line (x = const): dy != 0, dx == 0
    if abs(dy) > abs(dx) * 1e-6 and abs(dx) < 0.001:
        ox = inst_x
        oy = y0
        return ox, oy, float('inf'), inst_x  # k = inf, b = x_const

    # Horizontal line (y = const): dy == 0
    if abs(dy) < 0.001:
        ox = x0
        oy = inst_y
        return ox, oy, 0.0, inst_y  # k = 0 (x = b)

    # Normal case
    k = -dx / dy
    b = (target_x * inst_y - inst_x * target_y) / dy
    ox = (b + k * k * x0 - k * y0) / (1 + k * k)
    oy = (y0 + k * b - k * x0) / (1 + k * k)
    return ox, oy, k, b

def dist_to_line(px, py, k, b):
    if abs(k) > 1e6:  # vertical line x = b
        return abs(px - b)
    if abs(k) < 0.001 and abs(b) < 1e6:  # horizontal line y = b
        return abs(py - b)
    return abs(px + k * py - b) / math.sqrt(1 + k * k)


print("=" * 70)
print("  抵偿坐标算法仿真")
print("=" * 70)

test_cases = [
    ((0, 0), (3, 2), (10, 10), "斜线, 机器人偏离"),
    ((0, 0), (-2, 3), (5, 5),  "斜线, 机器人在线左侧"),
    ((0, 5), (5, 0), (10, 5),  "水平线 y=5"),
    ((3, 0), (8, 6), (3, 10),  "垂直线 x=3"),
    ((2, 2), (4, 1), (8, 8),   "一般斜线"),
]

for (ix, iy), (sx, sy), (tx, ty), label in test_cases:
    print(f"\n{'─'*60}")
    print(f"  {label}")
    print(f"  inst=({ix},{iy})  target=({tx},{ty})  robot=({sx},{sy})")
    print(f"{'─'*60}")

    # Try buggy (may crash)
    try:
        ox_b, oy_b, k_b, b_b = offset_buggy(ix, iy, sx, sy, tx, ty)
        on_traj_b = dist_to_line(ox_b, oy_b, k_b, b_b) < 0.001
        print(f"  Buggy:  k={k_b:10.4f}  b={b_b:10.4f}  offset=({ox_b:8.4f},{oy_b:8.4f})  on_traj={on_traj_b}")
    except ZeroDivisionError:
        print(f"  Buggy:  ZeroDivisionError (ty == iy) — crash!")

    # Fixed
    ox_f, oy_f, k_f, b_f = offset_fixed(ix, iy, sx, sy, tx, ty)
    dist_f = dist_to_line(ox_f, oy_f, k_f, b_f)
    on_traj_f = dist_f < 0.001

    if abs(k_f) > 1e6:
        print(f"  Fixed:  k=INF (垂直线)  x={b_f:8.4f}  offset=({ox_f:8.4f},{oy_f:8.4f})  on_traj={on_traj_f}")
    elif abs(k_f) < 0.001:
        print(f"  Fixed:  k=0   (水平线)  y={b_f:8.4f}  offset=({ox_f:8.4f},{oy_f:8.4f})  on_traj={on_traj_f}")
    else:
        print(f"  Fixed:  k={k_f:10.4f}  b={b_f:10.4f}  offset=({ox_f:8.4f},{oy_f:8.4f})  on_traj={on_traj_f}")

    # Verify perpendicular property
    print(f"  dist from robot to offset = {math.hypot(sx-ox_f, sy-oy_f):.6f}")
    print(f"  dist from robot to line   = {dist_to_line(sx, sy, k_f, b_f):.6f}")
    print(f"  is perpendicular projection?  {abs(math.hypot(sx-ox_f, sy-oy_f) - dist_to_line(sx, sy, k_f, b_f)) < 0.001}")

    # Show expected
    if abs(tx - ix) < 0.001:
        exp_k = "INF (垂直线)"
        exp_b = ix  # x = const
        print(f"  期望轨迹: x = {exp_b}")
    elif abs(ty - iy) < 0.001:
        exp_k = "0 (水平线)"
        exp_b = iy  # y = const
        print(f"  期望轨迹: y = {exp_b}")
    else:
        exp_k_val = -(tx - ix) / (ty - iy)
        exp_b_val = (tx * iy - ix * ty) / (iy - ty)
        print(f"  期望轨迹: k_exp={exp_k_val:.4f}, b_exp={exp_b_val:.4f}")

print(f"\n{'='*70}")
print("  结论:")
print("  1. k = -(target.x - inst.x / target.y - inst.y)  有括号优先级错误")
print("     除法优先于减法, 实际计算的是 -(target.x - inst.x/target.y - inst.y)")
print("  2. b 和 offset x/y 公式代数推导正确, 但用了错误的 k")
print("  3. 水平线(ty==iy)和垂直线(tx==ix)会除零, 需要特殊处理")
print(f"{'='*70}")
