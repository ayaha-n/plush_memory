import os
import requests
import base64

# 画像のパス設定
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
edit_body = {
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
