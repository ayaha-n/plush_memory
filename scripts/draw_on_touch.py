#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import subprocess

import rospy
from std_msgs.msg import Bool
import illustration_and_combine_new
import capture_on_touch

PARTS = ["larm", "rarm", "lleg", "rleg", "head", "stomach"]

path_to_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images"
path_to_raw_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images/raw_picture"
edit_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT_EDIT")
api_key = os.getenv("AZURE_API_KEY")

headers = {
    "Authorization": f"Bearer {api_key}",
}

DEVICE = rospy.get_param("/touch_capture/device", "/dev/camera_d405")  
VIDEO_SIZE = rospy.get_param("/touch_capture/video_size", "1280x720")    # "424x240", "848x480" is also fine
INPUT_FMT = rospy.get_param("/touch_capture/input_format", "yuyv422")   
PREFIX = "raw_image_"
FFMPEG = "ffmpeg"

def make_callback(kind: str, style: str = "shepard"):
    def cb(data: Bool):
        save_picture_and_draw(kind, style)
    return cb

def _new_pid_from_path(p):
    return os.path.splitext(os.path.basename(p))[0].replace("raw_image_", "")

def save_picture_and_draw(label, style: str = "shepard"):
    raw_image = capture_on_touch.next_filename()
    cmd = [
        FFMPEG, "-y",
        "-f", "v4l2",
        "-input_format", INPUT_FMT,
        "-video_size", VIDEO_SIZE,
        "-i", DEVICE,
        "-frames:v", "1",
        raw_image
    ]
    rospy.loginfo("Capturing -> %s", raw_image)
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5.0)
        ok = (proc.returncode == 0 and os.path.exists(raw_image))
        if ok:
            rospy.loginfo("Saved: %s", raw_image)
        else:
            rospy.logerr("ffmpeg failed (code=%s)\n%s", proc.returncode, proc.stdout.decode("utf-8", "ignore"))
            return (None, None)
    except subprocess.TimeoutExpired:
        rospy.logerr("ffmpeg timeout (device busy? try: fuser -v /dev/camera_d405)")
        return (None, None)
    except Exception as e:
        rospy.logerr("capture failed: %s", e)
        return (None, None)

    pid_str = _new_pid_from_path(raw_image)
    pid_int = int(pid_str)
    person_drawing = os.path.join(path_to_dir, f"person_drawing_{pid_str}.png")
    combined_image = os.path.join(path_to_dir, f"combined_drawing_{pid_str}.png")
    
    #generate person_image (if it doesn't exist)
    if not os.path.exists(person_drawing):
        success = illustration_and_combine_new.save_image_from_api(
            "Please turn this person into a cartoon-style illustration. Absolutely avoid "
            "photorealism. Remove or alter distinctive marks, logos, and text.",
            raw_image,
            person_drawing,
            transparent=False,
        )
        if not success:
            rospy.logerr("Failed to create person_drawing for pid=%s", pid_str)
            return (None, pid_int)
    
    #generate combined_image (if it doesn't exist)
    if not os.path.exists(combined_image):
        if label == "larm":
            rospy.loginfo("use flipped image")
        illustration_and_combine_new.combine_with_bear(person_drawing, label).save(combined_image)
        print(f"Saved combined image: {combined_image}")

    # generate final image, in whichever style this node was launched with
    # (see ~display_target on touch_image_camera_new.py) — one API call, one
    # file, shared by both the HTML display and the e-ink viewer. Filed
    # under data/images/<style>/ rather than tagging the filename, so that
    # fallback/memory lookups (which just scan by filename pattern) never
    # cross-pick an image generated in a different style on some earlier
    # day this node ran with a different ~display_target.
    if illustration_and_combine_new.generate_styled(combined_image, pid_str, label, style) is None:
        rospy.logwarn("Failed to generate final image for pid=%s label=%s", pid_str, label)
        return (None, pid_int)

    return (pid_int, pid_int)

if __name__ == "__main__":
    rospy.init_node("draw_on_touch", anonymous=False)
    display_target = rospy.get_param("~display_target", "eink")
    image_style = {"html": "classic", "eink": "shepard"}.get(display_target, "shepard")
    rospy.loginfo(f"display_target = {display_target} (image_style = {image_style})")
    for p in PARTS:
        rospy.Subscriber(f"/{p}_touch_trigger", Bool, make_callback(p, image_style))

    rospy.loginfo("draw_on_touch ready. device=%s size=%s fmt=%s", DEVICE, VIDEO_SIZE, INPUT_FMT)
    rospy.spin()


