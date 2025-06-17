#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import asyncio
import websockets
import os
import re

clients = set()
image_dir = os.path.join(os.path.dirname(__file__), "../data/images")

def get_available_ids():
    files = os.listdir(image_dir)
    ids = []
    for fname in files:
        match = re.match(r"generated_image_(\d+)_head\.png", fname)
        if match:
            ids.append(match.group(1))
    return sorted(ids)

async def notify_clients():
    while not rospy.is_shutdown():
        await asyncio.sleep(0.1)

async def handler(websocket, path):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)

async def publish_to_web(data):
    if data.data:
        rospy.loginfo("head_touch=true")
        ids = get_available_ids()
        id_string = ",".join(ids)
        message = f"SHOW_IMAGE:{id_string}"
        for client in clients.copy():
            try:
                await client.send(message)
                rospy.loginfo("Sent image")
            except:
                clients.remove(client)

def callback(data):
    asyncio.run_coroutine_threadsafe(publish_to_web(data), loop)

async def main():
    rospy.init_node('head_touch_listener', anonymous=True)
    rospy.Subscriber("/head_touch", Bool, callback)

    start_server = await websockets.serve(handler, "localhost", 8765)
    rospy.loginfo("WebSocket server started at ws://localhost:8765/")

    # rospyのシャットダウンを監視しながらloopを走らせる
    while not rospy.is_shutdown():
        await asyncio.sleep(0.1)

    # シャットダウン時のクリーンアップ
    start_server.close()
    await start_server.wait_closed()

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
