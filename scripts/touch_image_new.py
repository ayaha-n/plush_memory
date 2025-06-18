#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import asyncio
import websockets
import os
import re

clients = set()
image_dir = os.path.join(os.path.dirname(__file__), "../data/images")

def get_available_ids(kind):
    pattern = fr"generated_image_(\d+)_{kind}\.png"
    return sorted(
        match.group(1)
        for fname in os.listdir(image_dir)
        if (match := re.match(pattern, fname))
    )

async def publish_to_web(kind):
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
            clients.remove(client)

def callback_head(data):
    if data.data:
        asyncio.run_coroutine_threadsafe(publish_to_web("head"), loop)

def callback_hand(data):
    if data.data:
        asyncio.run_coroutine_threadsafe(publish_to_web("hand"), loop)

async def handler(websocket, path):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)

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
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
