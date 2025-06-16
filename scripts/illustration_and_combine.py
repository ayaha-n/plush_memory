import os
import requests
import base64
from PIL import Image

path_to_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images"
input_path = os.path.join(path_to_dir, "sample_picture.jpg")  # 入力画像
output_image_file = os.path.join(path_to_dir, "generated_image.png")  # 出力画像

# 環境変数からAPI情報取得（または直接記入してもOK）
edit_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT_EDIT")  # 例: "https://xxx.openai.azure.com"
api_key = os.getenv("AZURE_API_KEY")

# ヘッダー設定
headers = {
    "Authorization": f"Bearer {api_key}",
}

# リクエストボディ（フォームフィールド）
edit_body1 = {
    "prompt": "Please turn this person into a cartoon-style illustration.",
    "n": 1,
    "size": "1024x1024",
    "quality": "medium",
}

# ファイル読み込み（input_pathから）
files = {
    "image": ("sample_picture.jpg", open(input_path, "rb"), "image/jpeg")
}

# POSTリクエスト送信
response = requests.post(edit_endpoint, headers=headers, data=edit_body1, files=files)

# 応答の処理
if response.status_code == 200:
    result = response.json()
    if "b64_json" in result["data"][0]:
        b64_img = result["data"][0]["b64_json"]
        with open(output_image_file, "wb") as f:
            f.write(base64.b64decode(b64_img))
    elif "url" in result["data"][0]:
        image_url = result["data"][0]["url"]
        image_data = requests.get(image_url).content
        with open(output_image_file, "wb") as f:
            f.write(image_data)
    print(f"Saved image: {output_image_file}")
else:
    print(f"error: {response.status_code}")
    print(response.text)

img1 = Image.open(os.path.join(path_to_dir, "yellow_bear.png"))
img2 = Image.open(os.path.join(path_to_dir, "generated_image.png"))

def get_next_filename(base_name, ext, directory):
    i = 1
    while True:
        filename = f"{base_name}_{i}.{ext}"
        if not os.path.exists(os.path.join(directory, filename)):
            return os.path.join(directory, filename)
        i += 1

output_image_file = get_next_filename("generated_combined_image", "png", path_to_dir)

combined_width = img1.width + img2.width
combined_height = max(img1.height, img2.height)
combined_img = Image.new('RGBA', (combined_width, combined_height))

combined_img.paste(img1, (0, 0))
combined_img.paste(img2, (img1.width, 0))

combined_img_path = os.path.join(path_to_dir, "combined_image.png")
combined_img.save(combined_img_path)
print(f"Saved combined image: {combined_img_path}")

edit_body2 = {
    "prompt": "Please draw this human character touching the head of this uellow bear. Please draw in the same color tone as the bear. The human character should be smiling.",
    "n": 1,
    "size": "1024x1024",
    "quality": "medium",
}

files = {
    "image": ("combined_image.png", open(combined_img_path, "rb"), "image/jpeg")
}

# POSTリクエスト送信
response = requests.post(edit_endpoint, headers=headers, data=edit_body2, files=files)

# 応答の処理
if response.status_code == 200:
    result = response.json()
    if "b64_json" in result["data"][0]:
        b64_img = result["data"][0]["b64_json"]
        with open(output_image_file, "wb") as f:
            f.write(base64.b64decode(b64_img))
    elif "url" in result["data"][0]:
        image_url = result["data"][0]["url"]
        image_data = requests.get(image_url).content
        with open(output_image_file, "wb") as f:
            f.write(image_data)
    print(f"Saved image: {output_image_file}")
else:
    print(f"error: {response.status_code}")
    print(response.text)
