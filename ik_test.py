#!/usr/bin/env python3
"""
ランダム姿勢 軌道計画テスト（MoveIt不要 / 純粋Python版）

・順運動学 (FK) を変換行列で自前計算
・関節空間での線形補間で軌道生成
・円柱・球体パーツ間の幾何学的干渉チェック
"""
import numpy as np
import random
import math
import os

# ============================================================
# URDF パラメータ
# ============================================================
JOINT_LIMIT    = 1.57      # rad (±90°)
ARM_CYL_RADIUS = 0.04     # m  円柱半径
ARM_CYL_LENGTH = 0.30     # m  円柱長さ
SPHERE_RADIUS  = 0.06     # m  関節球体半径
BODY_RADIUS    = 0.20     # m  胴体半径

NUM_SEGMENTS   = 6        # 円柱パーツ数
NUM_GIMBALS    = 5        # ジンバル関節数
NUM_WAYPOINTS  = 20       # 軌道のウェイポイント数

# arm_L1 付け根の body_link ローカル座標とオイラー角 (URDF から)
ARM_L1_ORIGIN  = np.array([0.0, 0.2, -0.3])
ARM_L1_RPY     = (math.pi / 2, 0.0, 0.0)

JOINT_NAMES = []
for g in range(1, 6):
    for axis in ['roll', 'pitch', 'yaw']:
        JOINT_NAMES.append(f'arm_L1_g{g}_{axis}_joint')

# ============================================================
# 変換行列ユーティリティ
# ============================================================
def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def make_tf(R, t):
    """4x4 同次変換行列"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def rot2quat(R):
    """回転行列からクォータニオン (x, y, z, w) を計算"""
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2 
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S 
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2 
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S 
        qz = (R[0, 2] + R[2, 0]) / S 
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2 
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S 
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S 
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2 
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return np.array([qx, qy, qz, qw])

# ============================================================
# 順運動学 (Forward Kinematics)
# ============================================================
def compute_fk(joint_angles):
    """
    15個の関節角度 → 各セグメント（円柱6本 + 球体5個）の
    始点・終点を返す。
    
    Returns: list of (start_xyz, end_xyz, type, radius)
    """
    assert len(joint_angles) == 15

    # body_link → arm_L1 付け根
    r, p, y = ARM_L1_RPY
    R0 = rot_z(y) @ rot_y(p) @ rot_x(r)
    T = make_tf(R0, ARM_L1_ORIGIN)

    segments = []
    joint_idx = 0

    for seg in range(NUM_SEGMENTS):
        # 円柱の始点
        cyl_start = T[:3, 3].copy()
        # 円柱は local Z 方向に伸びる
        cyl_end = (T @ np.array([0, 0, ARM_CYL_LENGTH, 1]))[:3]
        segments.append((cyl_start, cyl_end, 'cylinder', ARM_CYL_RADIUS))

        # 円柱の先端へ移動
        T = T @ make_tf(np.eye(3), np.array([0, 0, ARM_CYL_LENGTH]))

        # ジンバル関節（最後の円柱の後にはない）
        if seg < NUM_GIMBALS:
            # 球体（関節位置 = 現在の T の原点）
            sphere_center = T[:3, 3].copy()
            segments.append((sphere_center, sphere_center, 'sphere', SPHERE_RADIUS))

            # Roll → Pitch → Yaw
            roll  = joint_angles[joint_idx]
            pitch = joint_angles[joint_idx + 1]
            yaw   = joint_angles[joint_idx + 2]
            joint_idx += 3

            R_gimbal = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
            T = T @ make_tf(R_gimbal, np.zeros(3))

    return segments


def get_ee_position(joint_angles):
    """手先（最終円柱の先端）の位置"""
    segs = compute_fk(joint_angles)
    # 最後の cylinder の end
    for s in reversed(segs):
        if s[2] == 'cylinder':
            return s[1]
    return np.zeros(3)

def print_fk_details(joint_angles):
    """指定した関節角度における各ジンバル関節の座標、回転行列、クォータニオンを表示"""
    print()
    print('【各ジンバル関節の絶対姿勢 (FK詳細)】')
    r, p, y = ARM_L1_RPY
    R0 = rot_z(y) @ rot_y(p) @ rot_x(r)
    T = make_tf(R0, ARM_L1_ORIGIN)

    joint_idx = 0
    for seg in range(NUM_GIMBALS):
        # 先に円柱の長さ分移動
        T = T @ make_tf(np.eye(3), np.array([0, 0, ARM_CYL_LENGTH]))

        roll  = joint_angles[joint_idx]
        pitch = joint_angles[joint_idx + 1]
        yaw   = joint_angles[joint_idx + 2]
        joint_idx += 3

        R_gimbal = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
        T = T @ make_tf(R_gimbal, np.zeros(3))

        R_current = T[:3, :3]
        pos_current = T[:3, 3]
        q = rot2quat(R_current)

        print(f'  [ジンバル {seg + 1}] (arm_L1_g{seg + 1})')
        print(f'    位置 (XYZ)       : [{pos_current[0]:+6.3f}, {pos_current[1]:+6.3f}, {pos_current[2]:+6.3f}]')
        print(f'    クォータニオン   : [x={q[0]:+6.3f}, y={q[1]:+6.3f}, z={q[2]:+6.3f}, w={q[3]:+6.3f}]')
        print(f'    回転行列 (R)     :')
        for row in R_current:
            print(f'      [{row[0]:+6.3f}, {row[1]:+6.3f}, {row[2]:+6.3f}]')

# ============================================================
# 干渉チェック
# ============================================================
def segment_distance(p1, p2, p3, p4):
    """線分 p1-p2 と 線分 p3-p4 の最短距離"""
    d1 = p2 - p1
    d2 = p4 - p3
    r  = p1 - p3

    a = np.dot(d1, d1)
    b = np.dot(d1, d2)
    c = np.dot(d2, d2)
    d = np.dot(d1, r)
    e = np.dot(d2, r)

    denom = a * c - b * b
    if denom < 1e-10:
        s = 0.0
        t = np.clip(e / c if c > 1e-10 else 0.0, 0, 1)
    else:
        s = np.clip((b * e - c * d) / denom, 0, 1)
        t = np.clip((a * e - b * d) / denom, 0, 1)

    closest1 = p1 + s * d1
    closest2 = p3 + t * d2
    return np.linalg.norm(closest1 - closest2)


def check_self_collision(segments):
    """
    非隣接パーツ間の干渉チェック。
    チェーン構造 cyl(i)-sphere(i+1)-cyl(i+2)-sphere(i+3) のため、
    物理的に接続されたパーツ（±3 インデックス以内）は除外する。
    """
    NEIGHBOR_SKIP = 4  # cyl→sphere→cyl→sphere = 3 要素分は物理的に隣接
    for i in range(len(segments)):
        for j in range(i + NEIGHBOR_SKIP, len(segments)):
            si = segments[i]
            sj = segments[j]

            # 距離計算
            if si[2] == 'sphere' and sj[2] == 'sphere':
                dist = np.linalg.norm(si[0] - sj[0])
                min_dist = si[3] + sj[3]
            elif si[2] == 'sphere':
                dist = segment_distance(sj[0], sj[1], si[0], si[0])
                min_dist = sj[3] + si[3]
            elif sj[2] == 'sphere':
                dist = segment_distance(si[0], si[1], sj[0], sj[0])
                min_dist = si[3] + sj[3]
            else:
                dist = segment_distance(si[0], si[1], sj[0], sj[1])
                min_dist = si[3] + sj[3]

            if dist < min_dist:
                return True, i, j, dist, min_dist

    return False, -1, -1, 0.0, 0.0


# ============================================================
# 胴体との干渉チェック
# ============================================================
def check_body_collision(segments):
    """各パーツが胴体（原点中心、半径 BODY_RADIUS の円柱）に干渉しないか"""
    for i, seg in enumerate(segments):
        if seg[2] == 'sphere':
            # 球体中心から Y 軸までの距離（胴体は body_link の Z 軸方向）
            dist_xy = math.sqrt(seg[0][0]**2 + seg[0][2]**2)
            if dist_xy < BODY_RADIUS + seg[3]:
                return True, i
        else:
            # 円柱の中点
            mid = (seg[0] + seg[1]) / 2.0
            dist_xy = math.sqrt(mid[0]**2 + mid[2]**2)
            if dist_xy < BODY_RADIUS + seg[3]:
                return True, i
    return False, -1


# ============================================================
# メイン処理
# ============================================================
def main():
    MAX_ATTEMPTS = 10
    max_range = JOINT_LIMIT * 0.4   # ±36°

    print('=' * 55)
    print('  ランダム姿勢  軌道計画テスト（純粋Python版）')
    print('=' * 55)
    print(f'  関節可動域       : ±{math.degrees(JOINT_LIMIT):.0f}°')
    print(f'  ランダム範囲     : ±{math.degrees(max_range):.0f}°')
    print(f'  円柱パーツ半径   : {ARM_CYL_RADIUS*100:.0f} cm')
    print(f'  関節球体半径     : {SPHERE_RADIUS*100:.0f} cm')
    print(f'  胴体半径         : {BODY_RADIUS*100:.0f} cm')
    print()

    # --- 1. 干渉しない目標を見つけるまでリトライ ---
    init_joints = [0.0] * len(JOINT_NAMES)
    goal_joints = None

    for attempt in range(1, MAX_ATTEMPTS + 1):
        candidate = [random.uniform(-max_range, max_range) for _ in JOINT_NAMES]
        segs = compute_fk(candidate)
        has_self, si, sj, dist, min_d = check_self_collision(segs)
        has_body, bi = check_body_collision(segs)

        if has_self or has_body:
            reason = f'自己干渉(パーツ{si}-{sj})' if has_self else f'胴体干渉(パーツ{bi})'
            print(f'  試行 {attempt}/{MAX_ATTEMPTS}: {reason} → リトライ')
            continue

        goal_joints = candidate
        print(f'  試行 {attempt}/{MAX_ATTEMPTS}: ✔ 干渉なしの姿勢を発見！')
        break

    if goal_joints is None:
        print(f'  ★ {MAX_ATTEMPTS}回試行しましたが干渉のない姿勢が見つかりませんでした。')
        return

    # --- 2. 目標関節角度を表示 ---
    print()
    print('【目標関節角度】')
    for name, val in zip(JOINT_NAMES, goal_joints):
        print(f'  {name}: {math.degrees(val):+8.2f}°')

    # --- 3. FK で初期位置・目標位置を計算 ---
    init_pos = get_ee_position(init_joints)
    goal_pos = get_ee_position(goal_joints)
    move_dist = np.linalg.norm(goal_pos - init_pos)

    print()
    print('【順運動学 (FK) 結果】')
    print(f'  初期手先位置 : [{init_pos[0]:.4f}, {init_pos[1]:.4f}, {init_pos[2]:.4f}]')
    print(f'  目標手先位置 : [{goal_pos[0]:.4f}, {goal_pos[1]:.4f}, {goal_pos[2]:.4f}]')
    print(f'  手先移動距離 : {move_dist:.4f} m')

    print_fk_details(goal_joints)

    # --- 4. 軌道生成（関節空間の線形補間） ---
    print()
    print('【軌道計画】')
    print(f'  方式: 関節空間線形補間 ({NUM_WAYPOINTS} ウェイポイント)')
    print(f'  初期状態 [全関節 0°] → ランダム目標')

    trajectory = []
    collision_at = -1

    for wp in range(NUM_WAYPOINTS + 1):
        alpha = wp / NUM_WAYPOINTS
        interp = [init_joints[j] + alpha * (goal_joints[j] - init_joints[j])
                  for j in range(len(JOINT_NAMES))]

        segs  = compute_fk(interp)
        ee    = get_ee_position(interp)
        has_c, _, _, _, _ = check_self_collision(segs)
        has_b, _ = check_body_collision(segs)

        trajectory.append({
            'alpha': alpha,
            'joints': interp,
            'ee': ee.copy(),
            'collision': has_c or has_b,
        })

        if (has_c or has_b) and collision_at < 0:
            collision_at = wp

    # --- 5. 結果出力 ---
    print()
    if collision_at >= 0:
        pct = collision_at / NUM_WAYPOINTS * 100
        print(f'  ★ 実現不可能な軌道です（ウェイポイント {collision_at}/{NUM_WAYPOINTS} '
              f'= {pct:.0f}% 地点で干渉発生）')
        print()
    else:
        print('  ✔ 全ウェイポイントで干渉なし — 軌道は実現可能です！')
        print()

    print('【軌道サマリー】')
    print(f'  {"WP":>4}  {"進捗":>5}  {"手先X":>8}  {"手先Y":>8}  {"手先Z":>8}  {"状態":>6}')
    print(f'  {"----":>4}  {"-----":>5}  {"--------":>8}  {"--------":>8}  {"--------":>8}  {"------":>6}')

    for i, wp in enumerate(trajectory):
        if i % max(1, NUM_WAYPOINTS // 5) == 0 or i == NUM_WAYPOINTS:
            status = '⚠干渉' if wp['collision'] else '  OK'
            ee = wp['ee']
            print(f'  {i:4d}  {wp["alpha"]*100:5.1f}%  {ee[0]:+8.4f}  {ee[1]:+8.4f}  {ee[2]:+8.4f}  {status}')

    print()
    print('=' * 55)

    # --- 6. 3Dアニメーション生成 ---
    print()
    print('【3Dアニメーション生成中...】')
    save_path = create_animation(trajectory, init_joints, goal_joints)
    print(f'  保存先: {save_path}')
    print('  完了！')


def create_animation(trajectory, init_joints, goal_joints):
    """matplotlib で軌道の2Dアニメーション（上面図 + 側面図）を生成し GIF で保存"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle

    # 全フレームのセグメントデータを事前計算
    all_frames = []
    ee_trail = []
    for wp in trajectory:
        segs = compute_fk(wp['joints'])
        ee = wp['ee']
        ee_trail.append(ee.copy())
        all_frames.append(segs)

    # 描画範囲を決定
    all_pts = []
    for segs in all_frames:
        for s in segs:
            all_pts.append(s[0])
            all_pts.append(s[1])
    all_pts = np.array(all_pts)
    margin = 0.3
    x_min, x_max = all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin
    y_min, y_max = all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin
    z_min, z_max = all_pts[:, 2].min() - margin, all_pts[:, 2].max() + margin

    # 色テーマ
    BG_COLOR    = '#1a1a2e'
    CYL_COLOR   = '#ff6b6b'
    SPH_COLOR   = '#4ecdc4'
    TRAIL_COLOR = '#ffd93d'
    BODY_COLOR  = '#6c757d'
    TEXT_COLOR  = '#e0e0e0'
    GRID_COLOR  = '#2a2a4a'

    fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(14, 7), facecolor=BG_COLOR)

    def draw_frame(frame_idx):
        for ax in (ax_top, ax_side):
            ax.cla()
            ax.set_facecolor(BG_COLOR)
            ax.tick_params(colors=TEXT_COLOR, labelsize=7)
            ax.grid(True, color=GRID_COLOR, linewidth=0.5, alpha=0.5)
            for spine in ax.spines.values():
                spine.set_color(GRID_COLOR)

        # --- 上面図 (X-Y) ---
        ax_top.set_xlim(x_min, x_max)
        ax_top.set_ylim(y_min, y_max)
        ax_top.set_xlabel('X [m]', color=TEXT_COLOR, fontsize=9)
        ax_top.set_ylabel('Y [m]', color=TEXT_COLOR, fontsize=9)
        ax_top.set_title('Top View (X-Y)', color=TEXT_COLOR, fontsize=12, fontweight='bold')
        ax_top.set_aspect('equal')

        # 胴体（円）
        body_circle = Circle((0, 0), BODY_RADIUS, fill=True,
                             facecolor=BODY_COLOR, edgecolor='white',
                             alpha=0.3, linewidth=1)
        ax_top.add_patch(body_circle)

        # --- 側面図 (Y-Z) ---
        ax_side.set_xlim(y_min, y_max)
        ax_side.set_ylim(z_min, z_max)
        ax_side.set_xlabel('Y [m]', color=TEXT_COLOR, fontsize=9)
        ax_side.set_ylabel('Z [m]', color=TEXT_COLOR, fontsize=9)
        ax_side.set_title('Side View (Y-Z)', color=TEXT_COLOR, fontsize=12, fontweight='bold')
        ax_side.set_aspect('equal')

        # 胴体（長方形近似）
        ax_side.axhspan(-BODY_RADIUS, BODY_RADIUS,
                        xmin=0, xmax=1, alpha=0.1, color=BODY_COLOR)

        # セグメント描画
        segs = all_frames[frame_idx]
        for seg in segs:
            if seg[2] == 'cylinder':
                # 上面図
                ax_top.plot([seg[0][0], seg[1][0]],
                            [seg[0][1], seg[1][1]],
                            color=CYL_COLOR, linewidth=4, solid_capstyle='round')
                # 側面図
                ax_side.plot([seg[0][1], seg[1][1]],
                             [seg[0][2], seg[1][2]],
                             color=CYL_COLOR, linewidth=4, solid_capstyle='round')
            else:
                # 球体
                ax_top.plot(seg[0][0], seg[0][1], 'o',
                            color=SPH_COLOR, markersize=8, zorder=5)
                ax_side.plot(seg[0][1], seg[0][2], 'o',
                             color=SPH_COLOR, markersize=8, zorder=5)

        # 手先マーカー (current)
        ee = ee_trail[frame_idx]
        ax_top.plot(ee[0], ee[1], '*', color=TRAIL_COLOR, markersize=15, zorder=10)
        ax_side.plot(ee[1], ee[2], '*', color=TRAIL_COLOR, markersize=15, zorder=10)

        # Start marker (START)
        s = ee_trail[0]
        s_label = f'START ({s[0]:+.2f}, {s[1]:+.2f}, {s[2]:+.2f})'
        ax_top.plot(s[0], s[1], 'o', color='#2ecc71', markersize=10, zorder=8,
                    markeredgecolor='white', markeredgewidth=1.5)
        ax_top.annotate(s_label, (s[0], s[1]), color='#2ecc71', fontsize=7,
                        fontweight='bold', xytext=(8, 8), textcoords='offset points')
        ax_side.plot(s[1], s[2], 'o', color='#2ecc71', markersize=10, zorder=8,
                     markeredgecolor='white', markeredgewidth=1.5)
        ax_side.annotate(s_label, (s[1], s[2]), color='#2ecc71', fontsize=7,
                         fontweight='bold', xytext=(8, 8), textcoords='offset points')

        # Goal marker (GOAL)
        g = ee_trail[-1]
        g_label = f'GOAL ({g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f})'
        ax_top.plot(g[0], g[1], 'D', color='#e74c3c', markersize=10, zorder=8,
                    markeredgecolor='white', markeredgewidth=1.5)
        ax_top.annotate(g_label, (g[0], g[1]), color='#e74c3c', fontsize=7,
                        fontweight='bold', xytext=(8, -12), textcoords='offset points')
        ax_side.plot(g[1], g[2], 'D', color='#e74c3c', markersize=10, zorder=8,
                     markeredgecolor='white', markeredgewidth=1.5)
        ax_side.annotate(g_label, (g[1], g[2]), color='#e74c3c', fontsize=7,
                         fontweight='bold', xytext=(8, -12), textcoords='offset points')

        # 手先の軌跡
        if frame_idx > 0:
            trail = np.array(ee_trail[:frame_idx + 1])
            ax_top.plot(trail[:, 0], trail[:, 1],
                        color=TRAIL_COLOR, linewidth=1.5, alpha=0.6, linestyle='--')
            ax_side.plot(trail[:, 1], trail[:, 2],
                         color=TRAIL_COLOR, linewidth=1.5, alpha=0.6, linestyle='--')

        # Title
        pct = frame_idx / max(len(all_frames) - 1, 1) * 100
        status = 'COLLISION' if trajectory[frame_idx]['collision'] else 'OK'
        fig.suptitle(
            f'Hexapod Arm L1 — WP {frame_idx}/{len(all_frames)-1}  '
            f'({pct:.0f}%)  [{status}]',
            color=TRAIL_COLOR, fontsize=14, fontweight='bold', y=0.98)

    anim = FuncAnimation(fig, draw_frame, frames=len(all_frames),
                         interval=250, repeat=True)

    save_path = os.path.join(os.path.expanduser('~'), 'robot', 'trajectory_animation.gif')
    anim.save(save_path, writer=PillowWriter(fps=4))
    plt.close(fig)
    return save_path


if __name__ == '__main__':
    main()
