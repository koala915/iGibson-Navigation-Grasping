"""Draws blind_spots.png from coverage_map.py; prints the blind areas and the option table."""
import json
import math
from pathlib import Path
import re
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import coverage_map as m  # noqa: E402

plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']
plt.rcParams['axes.unicode_minus'] = False
HERE = Path(__file__).resolve().parent
COLOURS = ['#E4E7EB', '#9AD8A6', '#FFD166', '#E23D3D']
# Real lap of 2026-10-30 (trace): seconds per metre and per radian, fitted below.
TRACE = sys.argv[1] if len(sys.argv) > 1 else None


def rates():
    t = json.load(open(TRACE))['trace']
    forward = turn = 0.
    secs = {'f': 0., 't': 0.}
    for a, b in zip(t, t[1:]):
        dt = b['odom_stamp'] - a['odom_stamp']
        d = math.hypot(b['pose'][0]-a['pose'][0], b['pose'][1]-a['pose'][1])
        y = abs(m.wrap(b['pose'][2]-a['pose'][2]))
        if dt > 3:
            continue
        if d > .02:
            forward += d
            secs['f'] += dt
        else:
            turn += y
            secs['t'] += dt
    return secs['f']/forward, secs['t']/turn, forward, turn


def seconds(points, spins, per_m, per_rad):
    poses = m.drive(points, spins)
    dist = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
    angle = sum(abs(m.wrap(b[2]-a[2])) for a, b in zip(poses, poses[1:]) if a[:2] == b[:2])
    return dist*per_m + angle*per_rad


def grid_of(points, spins):
    base, turning = m.cover(m.drive(points, spins))
    grid = np.full(m.U.shape, 3)
    grid[turning] = 2
    grid[base] = 1
    grid[~m.eligible] = 0
    return grid


def base_axes(ax, grid, points, title, big=False):
    ax.imshow(grid, origin='lower', extent=(0, m.umax-m.umin, 0, m.vmax-m.vmin),
              cmap=ListedColormap(COLOURS), vmin=0, vmax=3, interpolation='nearest')
    ax.add_patch(Rectangle((0, 0), m.umax-m.umin, m.vmax-m.vmin, fill=False, lw=2.5, ec='#22303C'))
    bx = [(p[0]-m.umin, p[1]-m.vmin) for p in m.box_corners]
    ax.fill(*zip(*bx), fc='#7A4FBF', ec='white', lw=1)
    xs = [p[0]-m.umin for p in points]
    ys = [p[1]-m.vmin for p in points]
    ax.plot(xs, ys, '-', color='#1256B8', lw=2.2 if big else 1.4, zorder=3)
    if big:
        ax.annotate('放置盒', (m.box[0]-m.umin, m.box[1]-m.vmin), xytext=(0, 26),
                    textcoords='offset points', ha='center', fontsize=12, color='#4B2E86',
                    fontweight='bold')
        for one, two in zip(points, points[1:]):
            mid = ((one[0]+two[0])/2-m.umin, (one[1]+two[1])/2-m.vmin)
            ang = math.atan2(two[1]-one[1], two[0]-one[0])
            ax.annotate('', xy=(mid[0]+.05*math.cos(ang), mid[1]+.05*math.sin(ang)), xytext=mid,
                        arrowprops=dict(arrowstyle='-|>', color='#1256B8', lw=2), zorder=4)
    ax.set_xlim(-.03, m.umax-m.umin+.03)
    ax.set_ylim(-.03, m.vmax-m.vmin+.03)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=15 if big else 11, fontweight='bold')
    ax.tick_params(labelsize=9 if big else 7)


def blind_areas(grid, value=3):
    n, labels, stats, centres = cv2.connectedComponentsWithStats((grid == value).astype(np.uint8), 8)
    out = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]*m.CELL**2
        if area >= .004:
            out.append((round(area*1e4), round(centres[i][0]*m.CELL, 2), round(centres[i][1]*m.CELL, 2),
                        round(stats[i, cv2.CC_STAT_WIDTH]*m.CELL, 2), round(stats[i, cv2.CC_STAT_HEIGHT]*m.CELL, 2)))
    return sorted(out, reverse=True)


if __name__ == '__main__':
    per_m, per_rad, f, t = rates()
    print('lap rates s/m %.1f s/rad %.1f over %.2f m %.2f rad' % (per_m, per_rad, f, t))
    ring = m.ring
    rows, panels = [], []
    for name, (pts, spins) in m.options.items():
        grid = grid_of(pts, spins)
        n = m.eligible.sum()
        never = 100*(grid == 3).sum()/n
        only_turn = 100*(grid == 2).sum()/n
        secs = seconds(pts, spins, per_m, per_rad)
        rows.append((name, round(float(never), 1), round(float(only_turn), 1), round(secs),
                     m.feasible(pts, spins)))
        print(rows[-1])
        panels.append((name, grid, pts, never, secs))

    fig, axes = plt.subplots(1, 5, figsize=(17, 4.2), dpi=120)
    for ax, (name, grid, pts, never, secs) in zip(axes, panels):
        base_axes(ax, grid, pts, '{}\n死角 {:.1f}%，約 {:.0f} 秒'.format(name, never, secs))
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(HERE/'options.png')

    grid = grid_of(ring, ())
    fig = plt.figure(figsize=(9.2, 9.6), dpi=130)
    ax = fig.add_subplot(111)
    base_axes(ax, grid, ring, '巡航一圈：後相機看得到哪裡（場地 2×2 m，和操作台同方向）', big=True)
    ax.legend(handles=[
        Patch(fc=COLOURS[1], label='直線段就看得到'), Patch(fc=COLOURS[2], label='只有轉彎途中掃過'),
        Patch(fc=COLOURS[3], label='整圈都看不到（死角）'), Patch(fc=COLOURS[0], label='不會放盒子（離牆 15 cm 內／放置盒旁）'),
        Line2D([0], [0], color='#1256B8', lw=2.2, label='巡航路線（逆時針）')],
        loc='upper center', bbox_to_anchor=(.5, -.05), ncol=2, fontsize=11, frameon=False)
    fig.tight_layout()
    fig.savefig(HERE/'blind_spots.png')
