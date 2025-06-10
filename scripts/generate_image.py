import os
import requests
import base64
from PIL import Image
from io import BytesIO

endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")  
deployment = os.getenv("gpt-image-1")
api_version = os.getenv("2025-04-01-preview")
subscription_key = os.getenv("AZURE_OPENAI_API_KEY")

headers = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {subscription_key}"
}

data = {
    "prompt": "An illustration of a yellow teddy bear",
    "size": "1024x1024",
    "quality": "medium",
    "output_compression": 100,
    "output_format": "png",
    "n": 1
}

response = requests.post(endpoint, headers=headers, json=data)
response.raise_for_status()

result = response.json()

# 画像データのBase64文字列を取得
b64_image = result['data'][0]['b64_json']

output_dir = os.path.join(os.path.dirname(__file__), "/home/leus/ros/catkin_ws/src/plush_memory/data/images")
output_path = os.path.join(output_dir, "generated_image.png")

# デコードしてファイルに保存
with open(output_path, "wb") as f:
    f.write(base64.b64decode(b64_image))

print(f"画像を {output_path} に保存しました。")

