#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import asyncio
import websockets
import os
import re
import signal
import sys
import time
import random
import draw_on_touch

PARTS = ["larm", "rarm", "lleg", "rleg", "head", "stomach"]

clients = set()
image_dir = os.path.join(os.path.dirname(__file__), "../data/images")
raw_dir = os.path.join(os.path.dirname(__file__), "../data/images/raw_picture")
trigger_states = {p: False for p in PARTS}      
displaying_states = {p: False for p in PARTS}      
shown_ids = {p: [] for p in PARTS}      
appended_id = {p: None for p in PARTS}      
pending_hide = {p: False for p in PARTS}
before_sets = {p: set() for p in PARTS}
gen_session = 0

ENABLE_GENERATION = True

def _list_ids(kind: str):
    pat = re.compile(rf"generated_drawing_(\d+)_{kind}\.png$")
    ids = []
    for fname in os.listdir(image_dir):
        m = pat.match(fname)
        if m:
            try:
                ids.append(int(m.group(1)))
            except ValueError:
                pass
    return sorted(ids)

def _idset(kind: str):
    return set(_list_ids(kind))

async def _ws_broadcast(message: str):
    for ws in list(clients):
        try:
            await ws.send(message)
        except Exception:
            try:
                clients.remove(ws)
            except KeyError:
                pass

def _delete_raw_image(img_id: int) -> bool:
    if img_id is None or img_id < 0:
        return False
    path = os.path.join(raw_dir, f"raw_image_{img_id}.jpg")
    try:
        os.remove(path)
        rospy.loginfo(f"Deleted raw image: {path}")
        return True
    except FileNotFoundError:
        rospy.logwarn(f"Raw image not found for id={img_id}: {path}")
        return False
    except Exception as e:
        rospy.logwarn(f"Failed to delete raw image '{path}': {e}")
        return False
    
async def _generate_one_in_executor(kind: str):
    #generate image
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, draw_on_touch.save_picture_and_draw, kind)

def _detect_new_id(kind: str, before, timeout_sec, poll_interval=0.5) -> int:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        after = _idset(kind)
        diff = after - before
        if diff:
            return max(diff)
        time.sleep(poll_interval)
    return -1

def _pick_fallback_id(kind: str):
    # if generation fails, select the 9th image
    ids_all = _list_ids(kind)
    remaining = [i for i in ids_all if i not in shown_ids[kind]]
    if not remaining:
        return -1
    if not ENABLE_GENERATION:
        return random.choice(remaining)
    return max(remaining)

async def _cooperative_sleep(total_sec: float, local_session: int, step: float = 0.1):
    end = time.time() + total_sec
    while time.time() < end:
        if not _is_session_valid(local_session):
            return
        await asyncio.sleep(step)

async def _append_while_generating(kind: str, local_session: int, gen_task: asyncio.Task):
    def make_pool():
        #ids_all = _list_ids(kind)
        ids_all = list(before_sets[kind])
        #used = set(selected_base) | set(temp_ids[kind])
        used = shown_ids[kind]
        return [i for i in ids_all if i not in used]

    while True:
        # stop when...
        if gen_task.done() or (not _is_session_valid(local_session)):
            return

        pool = make_pool()

        if pool:
            img_id = random.choice(pool)
            shown_ids[kind].append(img_id)
            #rospy.loginfo(f"shown_ids = {shown_ids[kind]}")
            #temp_ids[kind].append(img_id)
            await _ws_broadcast(f"TMP_IMAGE:{kind}:{img_id}")
            # interval to next image
        await asyncio.sleep(2.0)

async def _send_hide_image(img_id):
    msg = f"HIDE_IMAGE:{img_id}"
    for ws in list(clients):
        try:
            await ws.send(msg)
        except Exception:
            try:
                clients.remove(ws)
            except KeyError:
                pass

async def hide_current(kind: str):
    snapshot = list(shown_ids[kind])
    for img_id in snapshot:
        await _send_hide_image(img_id)
        await asyncio.sleep(0.4)
    leftovers = [i for i in shown_ids[kind] if i not in snapshot]
    for img_id in leftovers:
        await _send_hide_image(img_id)
        await asyncio.sleep(0.2)
    #displaying_states[kind] = False
    # for img_id in temp_ids[kind]:
    #     await _send_hide_image(img_id)
    #     await asyncio.sleep(0.2)
    # if appended_id[kind] is not None:
    #     await _send_hide_image(appended_id[kind])
    shown_ids[kind] = []
    appended_id[kind] = None

def _is_session_valid(local_session: int) -> bool:
    return gen_session == local_session

async def _append_final_no_generation(kind: str, local_session: int):
    if not _is_session_valid(local_session):
        rospy.loginfo(f"{kind} fallback skipped, hide_image")
        await hide_current(kind)
        pending_hide[kind] = False
        return
    fallback_id = _pick_fallback_id(kind)
    if fallback_id >= 0:
        appended_id[kind] = fallback_id
        if fallback_id not in shown_ids[kind]:  
            shown_ids[kind].append(fallback_id)
        await _ws_broadcast(f"APPEND_IMAGE:{kind}:{fallback_id}")
        rospy.loginfo(f"Appended FALLBACK existing image ({kind}): id={fallback_id}")
    else:
        rospy.loginfo(f"No fallback image available for {kind}; keep only the selected grid")

async def publish_to_web(kind: str):

    local_session = gen_session

    displaying_states[kind] = True
    shown_ids[kind] = []
    appended_id[kind] = None
    pending_hide[kind] = False

    try:
        ids_now = _list_ids(kind)
        if not ids_now:
            rospy.loginfo(f"No {kind} images available.")
            return

        # select 8 images
        k = min(8, len(ids_now))
        selected = ids_now[:] if len(ids_now) <= 8 else random.sample(ids_now, 8)
        shown_ids[kind] = selected[:]

        # check if session is valid
        if not _is_session_valid(local_session):
            rospy.loginfo(f"{kind} session invalidated before SHOW_IMAGE; abort showing")
            #displaying_states[kind] = False
            return
        
        # capture
        before_sets[kind] = set(selected) | _idset(kind)

        # generate drawing（draw_on_touch を呼ぶ）
        if ENABLE_GENERATION:
            gen_task = asyncio.create_task(_generate_one_in_executor(kind))
            
        
        # send selected image_ids and show image
        id_string = ",".join(str(i) for i in selected)
        await _ws_broadcast(f"SHOW_IMAGE:{kind}:{id_string}")
        rospy.loginfo(f"Sent {kind} image list (n={len(selected)})")
        #await asyncio.sleep(k * 2.0 + 0.5)
        await _cooperative_sleep(k * 2.0 + 0.5, local_session)

        if not ENABLE_GENERATION:
            await _append_final_no_generation(kind, local_session)
                        
        else:
            temp_loop = asyncio.create_task(_append_while_generating(kind, local_session, gen_task))

            gen_id, new_id = None, None
            try:
                gen_id, new_id = await gen_task
            except asyncio.CancelledError:
                rospy.loginfo(f"{kind} generation task cancelled")
            finally:
                if not temp_loop.done():
                    temp_loop.cancel()
                    try:
                        await temp_loop
                    except asyncio.CancelledError:
                        pass

            # if session is invalid after generation, delete raw image  
            if not _is_session_valid(local_session):
                rospy.loginfo(f"{kind} session invalid -> skip APPEND and cleanup")
                #new_id = _detect_new_id(kind, before_sets[kind], timeout_sec=0.1, poll_interval=0.1)
                if new_id: #>= 0:
                    _delete_raw_image(new_id)
                await hide_current(kind)
                return

            # detect new id and append image
            #new_id = _detect_new_id(kind, before_sets[kind], timeout_sec=2.0, poll_interval=0.5)
            #if new_id >= 0:
            if gen_id:
                appended_id[kind] = new_id
                if new_id not in shown_ids[kind]:   
                    shown_ids[kind].append(new_id)
                #rospy.loginfo(f"shown_ids = {shown_ids[kind]}")
                await _ws_broadcast(f"APPEND_IMAGE:{kind}:{new_id}")
                rospy.loginfo(f"Appended new image ({kind}): id={new_id}")                
                #_delete_raw_image(new_id)
            else:
                rospy.logwarn(f"Failed to detect new image id for kind={kind}")
                #generation failed
                await _append_final_no_generation(kind, local_session)
            if new_id:
                _delete_raw_image(new_id)
                
        await asyncio.sleep(4.0)
        if not trigger_states.get(kind, False) or pending_hide.get(kind, False) or not _is_session_valid(local_session):
            # hide images if trigger is off or if pending_flag is on
            await hide_current(kind)
            pending_hide[kind] = False
            rospy.loginfo(f"Post-show auto hide (one by one) for {kind}")
    finally:
        displaying_states[kind] = False


def make_callback(kind: str):
    def cb(data: Bool):
        global gen_session
        trigger_states[kind] = data.data
        #rospy.loginfo(f"{kind} trigger_states is {data.data}")
        if data.data:
            # for other in PARTS:
            #         if other != kind and (displaying_states[other] or shown_ids[other] or appended_id[other] is not None):
            #             asyncio.run_coroutine_threadsafe(hide_current(other), loop)
            if displaying_states[kind]:
                rospy.loginfo(f"Already displaying {kind}, skipping")
                return
            
            displaying_states[kind] = True
            gen_session += 1
            asyncio.run_coroutine_threadsafe(publish_to_web(kind), loop)

        else:
            if displaying_states[kind]:
                pending_hide[kind] = True
                rospy.loginfo(f"{kind} pending_hide set True")
            elif shown_ids[kind] or appended_id[kind] is not None:
                rospy.loginfo(f"{kind} false after show -> hide_current({kind})")
                asyncio.run_coroutine_threadsafe(hide_current(kind), loop)
    return cb

async def handler(websocket, path):
    peer = websocket.remote_address
    rospy.loginfo(f"WS connected: {peer}")
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)
        rospy.loginfo(f"WS disconnected: {peer}")

def shutdown_handler(signum, frame):
    rospy.loginfo("Shutting down...")
    loop.stop()
    sys.exit(0)

async def main():
    rospy.init_node('touch_image_camera', anonymous=True)

    global ENABLE_GENERATION
    
    ENABLE_GENERATION = rospy.get_param("~enable_generation", True)
    rospy.loginfo(f"enable_generation = {ENABLE_GENERATION}")
    
    for p in PARTS:                                                                                 
        rospy.Subscriber(f"/{p}_touch_trigger", Bool, make_callback(p))

    server = await websockets.serve(handler, "0.0.0.0", 8765)
    rospy.loginfo("WebSocket server started at ws://0.0.0.0:8765/")

        
    while not rospy.is_shutdown():
        await asyncio.sleep(0.1)

    server.close()
    await server.wait_closed()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, shutdown_handler)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
