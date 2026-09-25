#!/usr/bin/env python3
"""Generate an FK-only URDF whose collision meshes exist ONLY on the gripper links.

Why this file exists
--------------------
``FKComputer`` loads the robot into a headless PyBullet purely to compute forward
kinematics. It never calls ``stepSimulation``, so the only collision geometry that
is ever read is the ``getAABB`` of the six gripper links -- that is what the floor
guard's ``pad_bottom_z`` / ``min_gripper_link_z`` are made of.

Everything else costs startup time and RAM for nothing: the base, four mecanum
wheels, five arm links, the laser and the two cameras carry 59.5 MB of collision
STL that no code path queries. On a Jetson Nano reading from an SD card, parsing
them is the single slowest thing in the launch.

So this replaces those unqueried collision meshes with a placeholder box and
leaves the six gripper links untouched. ``--verify`` then proves the three
floor-guard metrics are unchanged across a pose sweep before you trust it.

The visual meshes are left in the file as they are -- FKComputer skips them with
URDF_IGNORE_VISUAL_SHAPES at load time, which needs no separate URDF.

    python3 make_deploy_urdf.py            # write yahboomcar_deploy.urdf
    python3 make_deploy_urdf.py --verify   # prove FK geometry is identical
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "yahboomcar.urdf"
OUT = HERE / "yahboomcar_deploy.urdf"

# Mirrors deploy_contract.GRIPPER_JOINT_MULTIPLIERS. Duplicated deliberately: this
# script must run without importing a version-specific contract module, and
# --verify cross-checks that the two lists still agree.
GRIPPER_JOINTS = ("grip_joint", "rlink_joint2", "rlink_joint3",
                  "llink_joint1", "llink_joint2", "llink_joint3")

# Placeholder size for a stripped link. Deliberately NOT a degenerate point: a zero
# box makes PyBullet warn, and a plausible-looking one would invite someone to
# trust it. 1 cm is obviously not a wheel.
PLACEHOLDER_BOX = "0.01 0.01 0.01"

HEADER = """
  GENERATED FILE -- do not hand-edit. Source: yahboomcar.urdf
  Rebuild with: python3 make_deploy_urdf.py

  FORWARD KINEMATICS ONLY. The collision geometry of every link EXCEPT the six
  gripper links ({keep}) has been replaced with a {box} m placeholder box.

  That makes this file WRONG for anything doing collision detection, contact
  queries, or stepSimulation. It is correct only for the FK + gripper-AABB use in
  FKComputer, which make_deploy_urdf.py --verify checks against the real URDF.
  Visual meshes are untouched here; FKComputer skips them via
  URDF_IGNORE_VISUAL_SHAPES.
"""


def keep_links(root: ET.Element) -> set:
    """Child link of every gripper joint -- the ones whose AABB the guard reads."""
    keep = set()
    for j in root.findall("joint"):
        if j.get("name") in GRIPPER_JOINTS:
            keep.add(j.find("child").get("link"))
    return keep


def build(src: Path = SRC, out: Path = OUT) -> dict:
    tree = ET.parse(src)
    root = tree.getroot()
    keep = keep_links(root)
    if len(keep) != len(GRIPPER_JOINTS):
        raise RuntimeError(
            f"expected {len(GRIPPER_JOINTS)} gripper links, found {sorted(keep)} -- "
            "the URDF joint names changed, and this script must be updated before "
            "it is trusted with the floor guard's geometry")

    stripped, kept, freed = [], [], 0.0
    for link in root.findall("link"):
        name = link.get("name")
        for col in link.findall("collision"):
            geom = col.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is None:
                continue
            if name in keep:
                kept.append(name)
                continue
            path = HERE / mesh.get("filename")
            if path.exists():
                freed += path.stat().st_size / 1048576
            geom.remove(mesh)
            ET.SubElement(geom, "box").set("size", PLACEHOLDER_BOX)
            stripped.append(name)

    root.insert(0, ET.Comment(HEADER.format(
        keep=" ".join(sorted(keep)), box=PLACEHOLDER_BOX)))
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return {"out": out, "stripped": stripped, "kept": sorted(set(kept)),
            "freed_mb": freed}


def verify(src: Path = SRC, out: Path = OUT) -> bool:
    """Load both URDFs and compare every metric the floor guard actually uses."""
    import numpy as np
    import pybullet as p

    sys.path.insert(0, str(HERE.parent / "v23"))
    import deploy_contract as dc

    if tuple(dc.GRIPPER_JOINT_MULTIPLIERS) != GRIPPER_JOINTS:
        print(f"[FAIL] gripper joint list drifted from the contract:\n"
              f"       contract  {tuple(dc.GRIPPER_JOINT_MULTIPLIERS)}\n"
              f"       this file {GRIPPER_JOINTS}")
        return False

    def load(path):
        c = p.connect(p.DIRECT)
        b = p.loadURDF(str(path), basePosition=list(dc.URDF_TO_TRAINING_FRAME),
                       useFixedBase=True, flags=p.URDF_IGNORE_VISUAL_SHAPES,
                       physicsClientId=c)
        n2i, s2f = {}, {}
        wanted = list(GRIPPER_JOINTS) + [f"arm_joint{i}" for i in range(1, 6)]
        for j in range(p.getNumJoints(b, physicsClientId=c)):
            full = p.getJointInfo(b, j, physicsClientId=c)[1].decode()
            n2i[full] = j
            for t in wanted:
                if full.endswith(t):
                    s2f[t] = full
        arm = [n2i[s2f[f"arm_joint{i}"]] for i in range(1, 6)]
        grip = [(n2i[s2f[s]], m) for s, m in dc.GRIPPER_JOINT_MULTIPLIERS.items()]
        pads = [n2i[s2f[dc.TCP_RIGHT_JOINT]], n2i[s2f[dc.TCP_LEFT_JOINT]]]
        return c, b, arm, grip, pads

    def metrics(handle, q, g):
        c, b, arm, grip, pads = handle
        for idx, v in zip(arm, q):
            p.resetJointState(b, idx, float(v), physicsClientId=c)
        for idx, mult in grip:
            p.resetJointState(b, idx, float(g) * mult, physicsClientId=c)
        pad_z = min(p.getAABB(b, i, physicsClientId=c)[0][2] for i in pads)
        lows = [(p.getAABB(b, i, physicsClientId=c)[0][2], i) for i, _ in grip]
        link_z, low_idx = min(lows)
        tcp = [np.asarray(p.getLinkState(b, i, computeForwardKinematics=1,
                                         physicsClientId=c)[0]) for i in pads]
        return pad_z, link_z, low_idx, tuple((tcp[0] + tcp[1]) / 2.0)

    a, d = load(src), load(out)

    rng = np.random.default_rng(0)
    lo = np.full(5, -1.57)
    hi = np.full(5, 1.57)
    # The two poses that actually matter first, then a broad random sweep.
    poses = [(np.zeros(5), 0.0),
             (np.array([0.0, -0.275, -1.42, -1.42, 0.0]), 0.0)]   # E1 grasp home
    poses += [(rng.uniform(lo, hi), float(rng.uniform(-0.5, 0.5)))
              for _ in range(400)]

    worst = 0.0
    for q, g in poses:
        ma, md = metrics(a, q, g), metrics(d, q, g)
        if ma[2] != md[2]:
            print(f"[FAIL] lowest gripper link differs at q={q} g={g}: "
                  f"{ma[2]} vs {md[2]}")
            return False
        for x, y in zip((ma[0], ma[1]) + ma[3], (md[0], md[1]) + md[3]):
            worst = max(worst, abs(float(x) - float(y)))

    p.disconnect(a[0])
    p.disconnect(d[0])
    ok = worst == 0.0
    print(f"[{'PASS' if ok else 'FAIL'}] {len(poses)} poses: max difference in "
          f"pad_bottom_z / min_gripper_link_z / gripper_center = {worst:.6g} m"
          + ("" if ok else "  -- NOT identical, do not deploy this URDF"))
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verify", action="store_true",
                    help="build, then prove the FK geometry is unchanged")
    args = ap.parse_args()

    info = build()
    print(f"[write] {info['out'].name}")
    print(f"        kept collision meshes on {len(info['kept'])} gripper links: "
          f"{' '.join(info['kept'])}")
    print(f"        stripped {len(info['stripped'])} links, "
          f"{info['freed_mb']:.1f} MB of collision STL no longer parsed at startup")
    if args.verify:
        return 0 if verify() else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
