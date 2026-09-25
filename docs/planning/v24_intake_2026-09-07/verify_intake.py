"""Read-only package inspection; never import handed-off code or unpickle models."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import zipfile


def inspect(package, repo):
    package = package.resolve()
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    read = lambda name: json.loads((package / name).read_text(encoding="utf-8"))
    checks = []
    listed = set()
    for line in (package / "SHA256SUMS.txt").read_text().splitlines():
        expected, name = line.split(None, 1)
        name = name.lstrip("*").replace("\\", "/")
        path = (package / name).resolve()
        path.relative_to(package)
        listed.add(name)
        checks.append({"file": name, "ok": path.is_file() and digest(path) == expected})
    metadata = read("evidence/training_metadata.json")
    protocol = read("evidence/preregistration.json")
    lock = read("evidence/selection_lock.json")
    claim = read("evidence/formal_claim.json")
    formal = read("evidence/formal_evaluation_seed90410.json")
    groups = {"bottle_cap": formal["bottle_cap"], "mixed_heights": formal["mixed_heights"]}
    groups.update(formal["grid"])
    rows = [r for group in groups.values() for r in group["rows"]]
    recount = {key: {"episodes": len(group["rows"]),
                     "successes": sum(r["success"] is True for r in group["rows"])}
               for key, group in groups.items()}
    artifacts = {"model": "model/v24_e1_model.zip",
                 "vecnormalize": "model/v24_e1_vecnormalize.pkl",
                 "protocol": "evidence/preregistration.json",
                 "training_metadata": "evidence/training_metadata.json",
                 "selection_lock": "evidence/selection_lock.json"}
    for name in formal["config"]["integrity_start"]:
        if name.endswith(".py"):
            artifacts[name] = "formal_evaluator_source/" + name
        elif name.endswith(".urdf"):
            artifacts[name] = "source_bundle/training/" + name
    binding = {name: digest(package / path) == formal["config"]["integrity_start"][name]
               for name, path in artifacts.items()}
    binding["formal_claim"] = digest(package / "evidence/formal_claim.json") == formal["config"]["formal_claim_sha256"]
    binding["registry"] = digest(package / "evidence/candidate_registry.json") == lock["registry"]["sha256"]
    binding["claim_lock"] = claim["selection_lock_sha256"] == digest(package / artifacts["selection_lock"])
    for name, filename in (("model", "model/v24_e1_model.zip"),
                           ("vecnormalize", "model/v24_e1_vecnormalize.pkl")):
        binding["selected_" + name] = lock["selected"][name]["sha256"] == digest(package / filename)
        binding["claim_" + name] = claim[name + "_sha256"] == digest(package / filename)
    sources = {}
    for name, item in metadata["source_bundle"].items():
        path = (package / item["path"].replace("\\", "/")).resolve()
        path.relative_to(package)
        sources[name] = path.is_file() and digest(path) == item["sha256"]
    parity = {}
    for incoming, local in (("source_bundle/training/deploy_contract.py", "grasp/v23/deploy_contract.py"),
                            ("source_bundle/training/x3plus/yahboomcar.urdf", "grasp/x3plus/yahboomcar.urdf")):
        parity[local] = ((package / incoming).read_bytes().replace(b"\r\n", b"\n")
                         == (repo / local).read_bytes().replace(b"\r\n", b"\n"))
    with zipfile.ZipFile(package / artifacts["model"]) as archive:
        data = json.loads(archive.read("data"))
        model = {"observation_shape": data["observation_space"]["_shape"],
                 "action_shape": data["action_space"]["_shape"],
                 "system_info": archive.read("system_info.txt").decode()}
    before = datetime.datetime.fromisoformat(protocol["created_at"])
    start = datetime.datetime.fromisoformat(metadata["started_at_utc"])
    return {
        "package": str(package), "checksum_entries": len(checks),
        "checksum_failures": [r for r in checks if not r["ok"]],
        "unlisted_files": [f.relative_to(package).as_posix() for f in package.rglob("*")
                           if f.is_file() and f.relative_to(package).as_posix() not in listed],
        "evidence_bindings": binding, "training_source_hashes": sources,
        "source_parity_lf_normalized": parity, "model_zip_metadata": model,
        "recount": recount, "total": len(rows), "successes": sum(r["success"] is True for r in rows),
        "unique_case_seeds": len({r["case_seed"] for r in rows}),
        "floor_penetration_episodes": sum(r["floor_penetration"] > 0 for r in rows),
        "floor_guard_interventions": sum(r["floor_guard_interventions"] for r in rows),
        "min_recorded_gripper_z_m": min(r["min_gripper_z"] for r in rows),
        "episodes_recorded_below_8mm": sum(r["min_gripper_z"] < .008 for r in rows),
        "preregistration_after_reported_training_start_seconds": (before-start).total_seconds(),
        "formal_integrity_start_equals_end": formal["config"]["integrity_start"] == formal["config"]["integrity_end"],
        "scope": "Static metadata and evidence recount only; no model inference, simulator replay or hardware validation.",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    args = parser.parse_args()
    print(json.dumps(inspect(args.package, Path(__file__).resolve().parents[3]), indent=2))
