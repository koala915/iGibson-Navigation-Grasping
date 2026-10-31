#!/usr/bin/env python3
"""Locked launcher for the v24 E1 replacement candidate.

The hardware controller remains the hardened v23 implementation because v24
changes the policy pair, not the deployment contract, E1 pose, URDF, or servo
mapping.  Keeping this file small prevents safety fixes from drifting between
two copied controllers.
"""

import sys
from pathlib import Path
from typing import Iterable, List, Optional


V24_DIR = Path(__file__).resolve().parent
V23_DIR = V24_DIR.parent / "v23"
MODEL = V24_DIR / "models" / "candidate_v24_e1_seed23404_bc_u10.zip"
VECNORMALIZE = V24_DIR / "models" / "candidate_v24_e1_seed23404_bc_u10_vec.pkl"
MANIFEST = V24_DIR / "manifest.json"
CONTRACT = "obs_28_incremental"

_PROTECTED = {
    "--model",
    "--vecnorm",
    "--contract",
    "--release-manifest",
    "--pose-tol-deg",
    "--entry-xy-mm",
    "--floor-finger-error-mm",
    "--tcp-forward-error-mm",
    "--jaw-track-fraction",
    "--jaw-max-lag-deg",
    "--s6-stall-grasp-steps",
}


def build_runtime_argv(user_args: Iterable[str]) -> List[str]:
    """Build v23 argv while refusing substitutions for the locked artifact tuple."""
    forwarded = list(user_args)
    for arg in forwarded:
        option = arg.split("=", 1)[0]
        if option in _PROTECTED:
            raise SystemExit(
                f"[FATAL] {option} is locked by grasp/v24/manifest.json. "
                "Use grasp/v23/x3plus_real_grasp.py directly for another pair."
            )

    return [
        "--model", str(MODEL),
        "--vecnorm", str(VECNORMALIZE),
        "--contract", CONTRACT,
        "--release-manifest", str(MANIFEST),
        # The physical E1 tuple used for v23's 3/3 supervised grasps. Keeping
        # this fixed isolates the v24 weight change during acceptance tests.
        "--pose-tol-deg", "3.0",
        "--entry-xy-mm", "10.0",
        "--floor-finger-error-mm", "15.0",
        "--tcp-forward-error-mm", "20.0",
        "--jaw-track-fraction", "0.5",
        "--jaw-max-lag-deg", "15.0",
        "--s6-stall-grasp-steps", "2",
        *forwarded,
    ]


def main(argv: Optional[Iterable[str]] = None) -> int:
    user_args = sys.argv[1:] if argv is None else list(argv)
    runtime_argv = build_runtime_argv(user_args)

    # x3plus_real_grasp imports deploy_contract as a sibling module.
    sys.path.insert(0, str(V23_DIR))
    try:
        import x3plus_real_grasp as runtime

        original_argv = sys.argv
        sys.argv = [str(V24_DIR / "run_candidate.py"), *runtime_argv]
        try:
            result = runtime.main()
            return int(result or 0)
        finally:
            sys.argv = original_argv
    finally:
        try:
            sys.path.remove(str(V23_DIR))
        except ValueError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
