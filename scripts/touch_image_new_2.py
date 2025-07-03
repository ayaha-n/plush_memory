#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import asyncio
import websockets
import os
import re
import signal
import sys

clients = set()
image_dir = os.path.join(os.path.dirname(__file__), "../data/images")
trigger_states = {"hand": False, "head": False}
currently_showing_kind = None
current_display_task = None  # show+waitタスク管理

def get_available_ids(kind):
    pattern = fr"generated_image_(\d+)_{kind}\.png"
    return sorted(
        (match.group(1) for fname in os.listdir(image_dir)
         if (match := re.match(pattern, fname))),
        key=int
    )

async def publish_to_web(kind):
    global currently_showing_kind, current_display_task

    if currently_showing_kind == kind:
        rospy.loginfo(f"Already displaying {kind}, skipping new publish")
        return

    # 他kind表示中ならキャンセル
    if current_display_task and not current_display_task.done():
        current_display_task.cancel()
        rospy.loginfo(f"Cancelled current display task for {currently_showing_kind}")
        #await hide_all_images()

    currently_showing_kind = kind
    rospy.loginfo(f"currently_showing_kind: {currently_showing_kind}")

    current_display_task = asyncio.create_task(show_and_schedule_hide(kind))

async def show_and_schedule_hide(kind):
    global currently_showing_kind
    ids = get_available_ids(kind)
    if not ids:
        rospy.loginfo(f"No {kind} images available.")
        return

    id_string = ",".join(ids)
    message = f"SHOW_IMAGE:{kind}:{id_string}"
    for client in clients.copy():
        try:
            await client.send(message)
            rospy.loginfo(f"Sent {kind} image list")
        except:
            clients.discard(client)

    # 各画像ごとに表示から13.5秒後に消すタスクを作成
    for index, img_id in enumerate(ids):
        display_delay = index * 1.5
        hide_delay = 13.5 + display_delay
        asyncio.create_task(schedule_hide_image(img_id, hide_delay, kind))

    try:
        total_time = 9 * 1.5 + 13.5  # 最後の画像が消える頃まで
        await asyncio.sleep(total_time + 0.5)
        rospy.loginfo("All images scheduled for hide completed")
        if currently_showing_kind == kind and not trigger_states.get(kind, False):
            currently_showing_kind = None
            rospy.loginfo(f"currently_showing_kind reset to None")
        hyde_all_images()
    except asyncio.CancelledError:
        rospy.loginfo(f"Show+wait task for {kind} cancelled")

async def schedule_hide_image(img_id, delay, kind):
    await asyncio.sleep(delay)
    if currently_showing_kind == kind:
        for client in clients.copy():
            try:
                await client.send(f"HIDE_IMAGE:{img_id}")
                rospy.loginfo(f"Sent hide command for image {img_id}")
            except:
                clients.discard(client)

async def hide_all_images():
    message = "HIDE_IMAGE_ALL"
    for client in clients.copy():
        try:
            await client.send(message)
            rospy.loginfo("Sent hide all images command")
        except:
            clients.discard(client)

def callback_hand(data):
    trigger_states["hand"] = data.data
    if data.data:
        asyncio.run_coroutine_threadsafe(publish_to_web("hand"), loop)

def callback_head(data):
    trigger_states["head"] = data.data
    if data.data:
        asyncio.run_coroutine_threadsafe(publish_to_web("head"), loop)

async def handler(websocket, path):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)

def shutdown_handler(signum, frame):
    rospy.loginfo("Shutting down...")
    loop.stop()
    sys.exit(0)

async def main():
    rospy.init_node('touch_listener', anonymous=True)
    rospy.Subscriber("/head_touch_trigger", Bool, callback_head)
    rospy.Subscriber("/hand_touch_trigger", Bool, callback_hand)

    start_server = await websockets.serve(handler, "localhost", 8765)
    rospy.loginfo("WebSocket server started at ws://localhost:8765/")

    while not rospy.is_shutdown():
        await asyncio.sleep(0.1)

    start_server.close()
    await start_server.wait_closed()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, shutdown_handler)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
