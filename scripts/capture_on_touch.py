#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, glob, subprocess, rospy, time
from std_msgs.msg import Bool

OUTPUT_DIR = os.path.expanduser("~/ros/catkin_ws/src/plush_memory/data/images/raw_picture")
DEVICE = rospy.get_param("/touch_capture/device", "/dev/video9")  
VIDEO_SIZE = rospy.get_param("/touch_capture/video_size", "848x480")    # "424x240" is also fine
INPUT_FMT = rospy.get_param("/touch_capture/input_format", "yuyv422")   
PREFIX = "person_image_"
FFMPEG = "ffmpeg"

def next_filename():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    files = sorted(glob.glob(os.path.join(OUTPUT_DIR, f"{PREFIX}[0-9]*.jpg")))
    last_num = 0
    if files:
        base = os.path.splitext(os.path.basename(files[-1]))[0]
        try:
            last_num = int(base.replace(PREFIX, ""))
        except ValueError:
            pass
    return os.path.join(OUTPUT_DIR, f"{PREFIX}{last_num+1:04d}.jpg")

def save_one():
    out_path = next_filename()
    cmd = [
        FFMPEG, "-y",
        "-f", "v4l2",
        "-input_format", INPUT_FMT,
        "-video_size", VIDEO_SIZE,
        "-i", DEVICE,
        "-frames:v", "1",
        out_path
    ]
    rospy.loginfo("Capturing -> %s", out_path)
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5.0)
        ok = (proc.returncode == 0 and os.path.exists(out_path))
        if ok:
            rospy.loginfo("Saved: %s", out_path)
        else:
            rospy.logerr("ffmpeg failed (code=%s)\n%s", proc.returncode, proc.stdout.decode("utf-8", "ignore"))
    except subprocess.TimeoutExpired:
        rospy.logerr("ffmpeg timeout (device busy? try: fuser -v /dev/video9)")
    except Exception as e:
        rospy.logerr("capture failed: %s", e)

def on_head(msg: Bool):
    if msg.data:
        save_one()

def on_hand(msg: Bool):
    if msg.data:
        save_one()

if __name__ == "__main__":
    rospy.init_node("touch_capture_simple", anonymous=False)
    rospy.Subscriber("/head_touch_trigger", Bool, on_head, queue_size=10)
    rospy.Subscriber("/hand_touch_trigger", Bool, on_hand, queue_size=10)
    rospy.loginfo("touch_capture_simple ready. device=%s size=%s fmt=%s", DEVICE, VIDEO_SIZE, INPUT_FMT)
    rospy.spin()
