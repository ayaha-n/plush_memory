#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import subprocess

from PIL import Image
import rospy
from std_msgs.msg import Bool
import illustration_and_combine_new
import capture_on_touch

PARTS = ["larm", "rarm", "lleg", "rleg", "head", "stomach"]

path_to_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images"
path_to_raw_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images/raw_picture"
bear_image_path = os.path.join(path_to_dir, "yellow_bear.png")
bear_flipped_image_path = os.path.join(path_to_dir, "yellow_bear_flipped.png")
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

def make_callback(kind: str):
    def cb(data: Bool):
        save_picture_and_draw(kind)
    return cb

def _new_pid_from_path(p):
    return os.path.splitext(os.path.basename(p))[0].replace("raw_image_", "")

def save_picture_and_draw(label):
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
    except subprocess.TimeoutExpired:
        rospy.logerr("ffmpeg timeout (device busy? try: fuser -v /dev/camera_d405)")
    except Exception as e:
        rospy.logerr("capture failed: %s", e)

    pid = _new_pid_from_path(raw_image)
    person_drawing = os.path.join(path_to_dir, f"person_drawing_{pid}.png")
    combined_image = os.path.join(path_to_dir, f"combined_drawing_{pid}.png")
    
    #generate person_image (if it doesn't exist)
    if not os.path.exists(person_drawing):
        success = illustration_and_combine_new.save_image_from_api(
            "Please turn this person into a cartoon-style illustration.",
            raw_image,
            person_drawing
        )
        if not success:
            rospy.logerr("Failed to create person_drawing for pid=%s", pid)
            return  # ← ここを continue ではなく return
    
    #generate combined_image (if it doesn't exist)
    if not os.path.exists(combined_image):
        if label == "larm":
            img1 = Image.open(bear_flipped_image_path)
            rospy.loginfo("use flipped image")
        else:
            img1 = Image.open(bear_image_path)
        img2 = Image.open(person_drawing)
        combined_width = img1.width + img2.width
        combined_height = max(img1.height, img2.height)
        combined_img = Image.new('RGBA', (combined_width, combined_height))
        combined_img.paste(img1, (0, 0))
        combined_img.paste(img2, (img1.width, 0))
        combined_img.save(combined_image)
        print(f"Saved combined image: {combined_image}")

    # generate final image
    prompt=illustration_and_combine_new.prompts[label]
    output_path = os.path.join(path_to_dir, f"generated_drawing_{pid}_{label}.png")
    if not os.path.exists(output_path):
        illustration_and_combine_new.save_image_from_api(prompt, combined_image, output_path)
    else:
        print(f"Already exists: {output_path}")

if __name__ == "__main__":        
    rospy.init_node("draw_on_touch", anonymous=False)
    #rospy.Subscriber("/head_touch_trigger", Bool, head_callback, queue_size=10)
    #rospy.Subscriber("/hand_touch_trigger", Bool, hand_callback, queue_size=10)
    for p in PARTS:                                                                                 
        rospy.Subscriber(f"/{p}_touch_trigger", Bool, make_callback(p))

    rospy.loginfo("draw_on_touch ready. device=%s size=%s fmt=%s", DEVICE, VIDEO_SIZE, INPUT_FMT)
    rospy.spin()


