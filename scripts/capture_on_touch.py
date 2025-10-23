#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, glob, subprocess, rospy, time, re
from std_msgs.msg import Bool

path_to_raw_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images/raw_picture"
path_to_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images"
DEVICE = rospy.get_param("/touch_capture/device", "/dev/video9")  
VIDEO_SIZE = rospy.get_param("/touch_capture/video_size", "848x480")    # "424x240" is also fine
INPUT_FMT = rospy.get_param("/touch_capture/input_format", "yuyv422")   
PREFIX = "raw_image_"
COM_PREFIX = "combined_drawing_"
FFMPEG = "ffmpeg"

def _collect_existing_ids():
    patterns = [
        os.path.join(path_to_dir, "combined_drawing_*.png"),
        os.path.join(path_to_dir, "generated_drawing_*_*.png"),  # *_<label>.png
        os.path.join(path_to_dir, "person_drawing_*.png"),
        os.path.join(path_to_raw_dir, f"{PREFIX}*.jpg"),
    ]
    ids = []
    for pat in patterns:
        for p in glob.glob(pat):
            b = os.path.splitext(os.path.basename(p))[0]
            # 末尾の連続数字を抜き出す（例: raw_image_1234 / combined_drawing_7 / generated_drawing_42_head）
            m = re.search(r'(\d+)(?:_[a-z]+)?$', b)
            if m:
                try:
                    ids.append(int(m.group(1)))
                except ValueError:
                    pass
    return ids

def next_filename():
    os.makedirs(path_to_raw_dir, exist_ok=True)
    #files = sorted(glob.glob(os.path.join(path_to_dir, f"{COM_PREFIX}[0-9]*.png")))
    ids = _collect_existing_ids()
    last_num = max(ids) if ids else 0
    #last_num = 0
    # if files:
    #     base = os.path.splitext(os.path.basename(files[-1]))[0]
    #     try:
    #         last_num = int(base.replace(COM_PREFIX, ""))
    #     except ValueError:
    #         pass
    #return os.path.join(path_to_raw_dir, f"{PREFIX}{last_num+1:04d}.jpg")
    return os.path.join(path_to_raw_dir, f"{PREFIX}{last_num+1}.jpg")

    
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
    rospy.init_node("capture_on_touch", anonymous=False)
    rospy.Subscriber("/head_touch_trigger", Bool, on_head, queue_size=10)
    rospy.Subscriber("/hand_touch_trigger", Bool, on_hand, queue_size=10)
    rospy.loginfo("capture_on_touch ready. device=%s size=%s fmt=%s", DEVICE, VIDEO_SIZE, INPUT_FMT)
    rospy.spin()
