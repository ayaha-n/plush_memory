import os
import requests
import base64
from PIL import Image


# 画像のパス設定
path_to_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images"
img1 = Image.open(os.path.join(path_to_dir, "yellow_bear.png"))
img2 = Image.open(os.path.join(path_to_dir, "generated_image.png"))
output_image_file = os.path.join(path_to_dir, "generated_combined_image.png")  # 出力画像

combined_width = img1.width + img2.width
combined_height = max(img1.height, img2.height)
combined_img = Image.new('RGBA', (combined_width, combined_height))

# 左右に貼り付け
combined_img.paste(img1, (0, 0))
combined_img.paste(img2, (img1.width, 0))

# 保存
combined_img_path = os.path.join(path_to_dir, "combined_image.png")
combined_img.save(combined_img_path)
print(f"Saved combined image: {combined_img_path}")

# 環境変数からAPI情報取得（または直接記入してもOK）
edit_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT_EDIT")  # 例: "https://xxx.openai.azure.com"
api_key = os.getenv("AZURE_API_KEY")


# ヘッダー設定
headers = {
    "Authorization": f"Bearer {api_key}",
}

# リクエストボディ（フォームフィールド）
edit_body = {
    "prompt": "Please draw this yellow bear shaking hands with this human character. Please draw in the same color tone as the bear.",
    "n": 1,
    "size": "1024x1024",
    "quality": "medium",
}

# ファイル読み込み（input_pathから）
files = {
    "image": ("combined_image.png", open(combined_img_path, "rb"), "image/jpeg")
}

# POSTリクエスト送信
response = requests.post(edit_endpoint, headers=headers, data=edit_body, files=files)

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
