#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import asyncio
import websockets
import os
import re

clients = set()
image_dir = os.path.join(os.path.dirname(__file__), "../data/images")
trigger_states = {"hand": False, "head": False}


def get_available_ids(kind):
    pattern = fr"generated_image_(\d+)_{kind}\.png"
    return sorted(
        (match.group(1) for fname in os.listdir(image_dir)
         if (match := re.match(pattern, fname))),
        key=int
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
     # 表示完了後のチェック
    total_time = len(ids) * 1.5  # 1.5秒 × 枚数
    await asyncio.sleep(total_time + 0.5)  # 安全に少し待つ
    if not trigger_states[kind]:
        # トリガーがOFFなら画像を消す
        for client in clients.copy():
            try:
                await client.send("HIDE_IMAGE")
                rospy.loginfo(f"Auto hide images for {kind}")
            except:
                clients.remove(client)
            
def callback_hand(data):
    trigger_states["hand"] = data.data  # 最新状態を記録
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
        clients.remove(websocket)

async def hide_images():
    message = "HIDE_IMAGE"
    for client in clients.copy():
        try:
            await client.send(message)
            rospy.loginfo("Sent hide image command")
        except:
            clients.remove(client)
            
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
