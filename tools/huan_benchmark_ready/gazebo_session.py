"""Keep each Classic scene separate and reject overlapping ROS simulators."""
from pathlib import Path


def simulator_environment(base_environment, run_directory):
    environment = dict(base_environment)
    environment["IGN_PARTITION"] = "carrierbot_" + Path(run_directory).name
    return environment


def ros_domain_conflicts(environment, proc_root=Path("/proc")):
    """Read only Gazebo server processes and selected environment fields."""
    conflicts = []
    target_domain = int(environment.get("ROS_DOMAIN_ID", "0"))
    if not proc_root.is_dir():
        return conflicts
    for process in proc_root.iterdir():
        if not process.name.isdigit():
            continue
        try:
            arguments = process.joinpath("cmdline").read_bytes().split(b"\0")
            if not arguments or Path(arguments[0].decode(errors="replace")).name != "gzserver":
                continue
            values = {}
            for item in process.joinpath("environ").read_bytes().split(b"\0"):
                key, separator, value = item.partition(b"=")
                if separator and key in (b"ROS_DOMAIN_ID", b"GAZEBO_MASTER_URI", b"IGN_PARTITION"):
                    values[key.decode()] = value.decode(errors="replace")
            if int(values.get("ROS_DOMAIN_ID", "0")) != target_domain:
                continue
        except (OSError, ValueError):
            # A process may disappear between /proc enumeration and reading.
            continue
        conflicts.append({"pid": int(process.name),
                          "arguments": [a.decode(errors="replace") for a in arguments if a],
                          "gazebo_master_uri": values.get("GAZEBO_MASTER_URI"),
                          "ign_partition": values.get("IGN_PARTITION"),
                          "ros_domain_id": target_domain})
    return sorted(conflicts, key=lambda process: process["pid"])
