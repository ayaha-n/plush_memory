import os
import re
import requests
import base64
from PIL import Image

# === 設定 ===
path_to_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images"
path_to_raw_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images/raw_picture"
bear_image_path = os.path.join(path_to_dir, "yellow_bear.png")
edit_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT_EDIT")
api_key = os.getenv("AZURE_API_KEY")

headers = {
    "Authorization": f"Bearer {api_key}",
}

# === プロンプト定義 ===
prompts = {
    "shake": "Please draw this yellow bear shaking hands with this human character. Please draw in the same color tone as the bear.",
    "hug": "Please draw this yellow bear hugging with this human character. Please draw in the same color tone as the bear.",
    "head": "Please draw this human character touching the head of this yellow bear. Please draw in the same color tone as the bear. The human character should be smiling."
}

# === ユーティリティ関数 ===

def get_participant_ids():
    """sample_image_<participant_id>.jpg にマッチする participant_id を抽出"""
    ids = []
    for fname in os.listdir(path_to_raw_dir):
        m = re.match(r"sample_image_(\d+)\.jpg", fname)
        if m:
            ids.append(m.group(1))
    return sorted(ids)

def save_image_from_api(prompt, image_path, output_path):
    body = {
        "prompt": prompt,
        "n": 1,
        "size": "1024x1024",
        "quality": "medium",
    }
    files = {
        "image": (os.path.basename(image_path), open(image_path, "rb"), "image/jpeg")
    }

    response = requests.post(edit_endpoint, headers=headers, data=body, files=files)

    if response.status_code == 200:
        result = response.json()
        if "b64_json" in result["data"][0]:
            b64_img = result["data"][0]["b64_json"]
            with open(output_path, "wb") as f:
                f.write(base64.b64decode(b64_img))
        elif "url" in result["data"][0]:
            image_url = result["data"][0]["url"]
            image_data = requests.get(image_url).content
            with open(output_path, "wb") as f:
                f.write(image_data)
        print(f"Saved image: {output_path}")
        return True
    else:
        print(f"error: {response.status_code}")
        print(response.text)
        return False

# === メイン処理 ===

participant_ids = get_participant_ids()

for pid in participant_ids:
    print(f"\n--- Processing participant {pid} ---")
    
    sample_image = os.path.join(path_to_raw_dir, f"sample_image_{pid}.jpg")
    person_image = os.path.join(path_to_dir, f"person_image_{pid}.png")
    combined_image = os.path.join(path_to_dir, f"combined_image_{pid}.png")
    
    # ステップ1: person_image の生成（なければ）
    if not os.path.exists(person_image):
        success = save_image_from_api(
            "Please turn this person into a cartoon-style illustration.",
            sample_image,
            person_image
        )
        if not success:
            continue
    
    # ステップ2: combined_image の生成（なければ）
    if not os.path.exists(combined_image):
        img1 = Image.open(bear_image_path)
        img2 = Image.open(person_image)
        combined_width = img1.width + img2.width
        combined_height = max(img1.height, img2.height)
        combined_img = Image.new('RGBA', (combined_width, combined_height))
        combined_img.paste(img1, (0, 0))
        combined_img.paste(img2, (img1.width, 0))
        combined_img.save(combined_image)
        print(f"Saved combined image: {combined_image}")

    # ステップ3: 各プロンプトごとに出力ファイルを生成
    for label, prompt in prompts.items():
        output_path = os.path.join(path_to_dir, f"generated_image_{pid}_{label}.png")
        if not os.path.exists(output_path):
            save_image_from_api(prompt, combined_image, output_path)
        else:
            print(f"Already exists: {output_path}")
