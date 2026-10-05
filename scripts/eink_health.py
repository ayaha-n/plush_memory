"""Bounded, nonfatal startup diagnostics for the e-ink tablet."""
import shlex

import rospy

from eink_memory_push import EINK_HOST, EINK_APP_DIR, DEVICE_ORIENTATIONS, ssh

# Succeeds iff the viewer is running (BusyBox ps: the command is column 5).
_VIEWER_RUNNING = "ps | awk '$5 ~ /(^|\\/)plush_memory_viewer$/ {found=1} END {exit !found}'"
_TIMEOUT_SEC = 10


def check_startup(orientation, clear_events=False, host=EINK_HOST):
    """Check once; failures only warn, the node keeps going.

    Event age cannot be inferred from the queue: report all pending event
    directories as possibly left over. Cleanup is opt-in and only permitted
    while the viewer is stopped, to avoid deleting an animation in progress.
    """
    try:
        app = shlex.quote(EINK_APP_DIR)
        command = f"""
cd {app} || exit 1
if test -f orientation; then echo orientation_present=1; else echo orientation_present=0; fi
if {_VIEWER_RUNNING}; then echo viewer_running=1; else echo viewer_running=0; fi
if test -r orientation.conf; then
    printf 'orientation_conf='; cat orientation.conf; printf '\\n'
else echo orientation_conf=; fi
if test -d events; then
    count=0
    for event in events/* events/.[!.]* events/..?*; do
        test -d "$event" && count=$((count + 1))
    done
    echo events=$count
else echo events=missing; fi
"""
        out = ssh(command, host, timeout=_TIMEOUT_SEC)
        fields = dict(line.split("=", 1) for line in out.stdout.splitlines() if "=" in line)
        rospy.loginfo(f"eink_health: key-auth SSH OK (root@{host})")
        for key, warning in (
                ("orientation_present", "viewer orientation file is missing; launch Plush Memory in AppLoad"),
                ("viewer_running", "plush_memory_viewer process is absent; launch Plush Memory in AppLoad")):
            if fields.get(key) != "1":
                rospy.logwarn(f"eink_health: {warning}")
        if fields.get("orientation_present") == fields.get("viewer_running") == "1":
            rospy.loginfo("eink_health: viewer process and orientation file found")
        actual = fields.get("orientation_conf", "").strip()
        if orientation not in DEVICE_ORIENTATIONS or actual != orientation:
            rospy.logwarn(f"eink_health: orientation.conf={actual or '(missing/empty)'} "
                          f"does not match ~eink_orientation={orientation}")
        else:
            rospy.loginfo(f"eink_health: orientation.conf matches {orientation}")
        events = fields.get("events", "")
        if not events.isdigit():
            rospy.logwarn("eink_health: events/ is missing; verify viewer installation")
        elif int(events):
            rospy.logwarn(f"eink_health: {events} pending event directories; "
                          "possibly left over from a previous run")
            if clear_events:
                # Recheck the process immediately before opt-in cleanup.
                ssh(f"cd {app} || exit 1; if {_VIEWER_RUNNING}; then exit 2; fi; "
                    "find events -mindepth 1 -maxdepth 1 -type d -exec rm -rf -- {} +",
                    host, timeout=_TIMEOUT_SEC)
                rospy.loginfo(f"eink_health: cleared pending events ({events} observed)")
        else:
            rospy.loginfo("eink_health: no pending events")
    except Exception as exc:
        detail = getattr(exc, "stderr", "") or str(exc)
        rospy.logwarn(f"eink_health: check/cleanup failed at root@{host}: {detail.strip()}; "
                      "check USB/Wi-Fi, SSH keys, AppLoad (cleanup requires viewer stopped); continuing")
