"""Reuse only an exactly matching built gate; never replace the baseline pointer."""
import hashlib
import json
from pathlib import Path
import sys


def matches(workspace, tools):
    executable = workspace/"install/carrierbot_navigation/lib/carrierbot_navigation/fablab_guidance_node"
    if not executable.is_file():
        return False
    try:
        manifest = json.loads((workspace/"gate_build_manifest.json").read_text(encoding="utf-8"))
        names = ("behavior_gate_source/fablab_guidance_node.cpp", "behavior_gate_source/route_preview.hpp",
                 "behavior_gate_source/behavior_gate.hpp", "recovery_source/avoidance_recovery.hpp")
        return all(hashlib.sha256((tools/name).read_bytes()).hexdigest() == manifest.get(name) for name in names)
    except (OSError, ValueError):
        return False


def ensure(home, tools):
    home, tools = Path(home).resolve(), Path(tools).resolve()
    runs = home/"carrierbot_test_runs"
    own = runs/"behavior_huan_workspace.txt"
    for pointer in (own, runs/"behavior_workspace.txt"):
        if not pointer.is_file():
            continue
        workspace = Path(pointer.read_text(encoding="utf-8").strip()).resolve()
        if workspace.parent == home and workspace.name.startswith("carrierbot-lien-gate.") and matches(workspace, tools):
            runs.mkdir(exist_ok=True)
            pending = own.with_suffix(".pending")
            pending.write_text(str(workspace)+"\n", encoding="utf-8")
            pending.replace(own)
            print("OK: reuse matching Fablab gate; baseline pointer unchanged")
            return True
    print("A matching gate needs an isolated build; baseline pointer stays unchanged")
    return False


if __name__ == "__main__":
    sys.exit(0 if ensure(Path.home(), Path(__file__).parent) else 2)
