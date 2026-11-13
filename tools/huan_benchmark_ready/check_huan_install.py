"""Local dependency and provenance checks. No ROS node or actuator is started."""
import ast
import hashlib
import json
from pathlib import Path
import sys


def check():
    root = Path(__file__).resolve().parent
    try:
        import numpy
        import yaml
    except ImportError as error:
        raise RuntimeError("System Python needs numpy and PyYAML (Ubuntu packages python3-numpy python3-yaml): " + str(error))
    manifest = json.loads((root / "integration_manifest.json").read_text(encoding="utf-8"))
    for relative, expected in manifest["runtime_sha256"].items():
        path = root / relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError("Integration file missing or changed: " + relative + "; reinstall from huan_benchmark_ready")
        if path.suffix == ".py":
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    from huan_fusion_bridge import verify_vendor
    from fusion_fixture import TOOLSET
    if manifest["toolset"] != TOOLSET:
        raise RuntimeError("Wrong benchmark manifest")
    upstream = verify_vendor()
    print("OK: %s; %d reviewed Huan modules; numpy %s" %
          (TOOLSET, len(upstream["files"]), numpy.__version__))
    print("CONTROL: Fablab -> existing safety_guard -> wheels. No Huan Nav2 wheel publisher.")
    print("DEFAULT: original velocity filter; explicit --velocity-profile cpu-sparse is experimental.")
    return manifest


if __name__ == "__main__":
    try:
        check()
    except Exception as error:
        print("STOP: " + str(error), file=sys.stderr)
        sys.exit(1)
