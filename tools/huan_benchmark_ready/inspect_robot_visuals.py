#!/usr/bin/env python3
"""Read running model geometry and rendering logs. No simulator state changes."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
from zipfile import ZipFile, ZIP_DEFLATED


def collect(arguments, output, env, timeout=12):
    with output.open("w", encoding="utf-8") as stream:
        try:
            process = subprocess.run(arguments, stdout=stream, stderr=subprocess.STDOUT,
                                     text=True, env=env, timeout=timeout)
            return {"arguments": arguments, "returncode": process.returncode}
        except subprocess.TimeoutExpired:
            stream.write("\nDIAGNOSTIC QUERY TIMEOUT\n")
            return {"arguments": arguments, "timed_out": True}
        except OSError as error:
            stream.write(str(error))
            return {"arguments": arguments, "error": str(error)}


def main():
    root = Path.home() / "carrierbot_test_runs"
    root.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=time.strftime("robot_visual_%Y%m%d_%H%M%S_"), dir=str(root)))
    env = dict(os.environ, GAZEBO_MASTER_URI="http://127.0.0.1:11349")
    metadata = {"purpose": "Read model and render diagnostics only; no spawn/delete/move requests",
                "gazebo_master_uri": env["GAZEBO_MASTER_URI"], "commands": []}
    print("DANG DOC MO HINH XE VA LOG HIEN THI. Khong thay doi mo phong.\nLOG: " + str(output), flush=True)
    metadata["commands"].append(collect(
        ["gz", "model", "-w", "default", "-m", "carrierbot", "-i"],
        output / "running_model.txt", env))
    sessions = list(root.glob("lien_*/session.json"))
    if sessions:
        session = max(sessions, key=lambda p: p.stat().st_mtime)
        metadata["candidate_startup_folder"] = str(session.parent)
        for name in ("session.json", "robot.urdf", "spawn.log", "gazebo_gui.log"):
            source = session.parent / name
            if source.is_file():
                shutil.copy2(source, output / name)
        if (output / "robot.urdf").is_file():
            metadata["commands"].append(collect(
                ["gz", "sdf", "-p", str(output / "robot.urdf")],
                output / "converted_robot.sdf", env))
    for name in ("ogre.log", "gzclient.log", "gzsdf.log"):
        source = Path.home() / ".gazebo" / name
        if source.is_file():
            text = source.read_text(encoding="utf-8", errors="replace")
            (output / (name + ".tail.txt")).write_text(
                "\n".join(text.splitlines()[-120:])[-24000:], encoding="utf-8")
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    archive = output / "robot_visual_diagnostics.zip"
    files = [p for p in output.iterdir() if p.is_file()]
    with ZipFile(archive, "w", ZIP_DEFLATED) as bundle:
        for path in files:
            bundle.write(path, path.name)
    print("GUI FILE NAY: " + str(archive), flush=True)
    try:
        subprocess.Popen(["xdg-open", str(output)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        print("MO THU MUC: " + str(output), flush=True)


if __name__ == "__main__":
    main()
