import os
import re
import requests
import base64
from PIL import Image

# === setting ===
path_to_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images"
path_to_raw_dir = "/home/leus/ros/catkin_ws/src/plush_memory/data/images/raw_picture"
bear_image_path = os.path.join(path_to_dir, "yellow_bear.png")
bear_flipped_image_path = os.path.join(path_to_dir, "yellow_bear_flipped.png")
edit_endpoint = "https://api.openai.com/v1/images/edits"
api_key = os.getenv("OPENAI_API_KEY")
image_model = "gpt-image-1"

headers = {
    "Authorization": f"Bearer {api_key}",
}

# === プロンプト定義 ===
# Action prompts, style-agnostic — pass through prompt_for() to pick a style.
hand_prompt = "Please draw this yellow bear shaking hands with this human character. The bear should be sitting and the human character should be smiling."

# The bear faces the viewer, so its own left leg is on the picture's right
# side and vice versa — spell out both so the model doesn't mirror it.
def leg_prompt(own_side, picture_side):
    return (f"Please draw this human character softly touching the {own_side} leg of this yellow bear "
            f"(the bear's own {own_side} leg, which appears on the {picture_side} side of the picture since the bear faces the viewer). "
            "The bear should be sitting and the human character should be smiling.")

prompts = {
    "hand": hand_prompt,
    "rarm": hand_prompt,
    "larm": hand_prompt,
    "hug": "Please draw this yellow bear hugging with this human character.",
    "head": "Please draw this human character touching the head of this yellow bear. The bear should be sitting and the human character should be smiling.",
    "stomach": "Please draw this human character giving a soft pat to the belly of this yellow bear plush toy. The bear should be sitting and the human character should be smiling.",
    "rleg": leg_prompt("right", "left"),
    "lleg": leg_prompt("left", "right"),
}

# === スタイル定義 ===
# "classic": the original flat-cartoon-color look the HTML display
# (plush_memory_camera.html) was built around — kept as-is so it doesn't
# need a transparent cutout.
# "shepard": classic children's-book pen-and-ink look, after E. H. Shepard's
# original Winnie-the-Pooh illustrations — fine linework and cross-hatching
# with a soft watercolor wash instead of flat color fill, and a transparent
# background (it suits one naturally, since Shepard's drawings barely have a
# background to begin with). This is what the e-ink viewer displays.
STYLES = {
    "classic": " Please draw in the same color tone as the bear.",
    "shepard": (
        " Keep each character's face, proportions, and expression exactly as "
        "they already appear in the reference image — do not redesign or "
        "reinterpret the face. Only change the art medium: render it as a "
        "delicate pen-and-ink line illustration with a soft watercolor wash, "
        "in the style of the classic colorized editions of E. H. Shepard's "
        "Winnie-the-Pooh drawings — fine ink linework and light "
        "cross-hatching for shading, gentle multi-color watercolor tones "
        "(not flat cartoon color fill), no background. Two deliberate "
        "exceptions to \"keep it exactly as the reference\": (1) the human "
        "character's open eye(s) must be mostly a dark pupil with a "
        "highlight dot inside, even if the reference draws them as a flat "
        "solid shape with no highlight — keep the eye itself small and "
        "gentle, almond-shaped, NOT a wide staring eye with a lot of white "
        "sclera showing. Make the highlight dot a specific, consistent "
        "size: roughly one quarter of the pupil's diameter, not a tiny "
        "speck and not covering most of the pupil. The human character's "
        "gaze (pupil position) should be "
        "turned toward the bear, so they are clearly looking at it. (2) the "
        "bear's eyes must be two simple round button shapes, matching each "
        "other in size and shape, each with one highlight dot sized the same "
        "way — about one quarter of the eye's diameter. (3) if "
        "the human character's mouth is a closed smile, draw it as one "
        "simple gently-curved line — not an open mouth, not a jagged or "
        "split-looking shape. For both characters, never a flat line for an "
        "open eye, a solid color fill with no highlight, a crosshatched eye, "
        "or a collapsed sliver."
    ),
}


def prompt_for(kind: str, style: str = "shepard") -> str:
    return prompts[kind] + STYLES[style]

# === ユーティリティ関数 ===

def get_participant_ids():
    """sample_image_<participant_id>.jpg にマッチする participant_id を抽出"""
    ids = []
    for fname in os.listdir(path_to_raw_dir):
        m = re.match(r"sample_image_(\d+)\.jpg", fname)
        if m:
            ids.append(m.group(1))
    return sorted(ids, key=lambda x: int(x))

def save_image_from_api(prompt, image_path, output_path, transparent=True):
    body = {
        "model": image_model,
        "prompt": prompt,
        "n": 1,
        "size": "1024x1024",
        "quality": "medium",
        "output_format": "png",
    }
    if transparent:
        # Real alpha-channel cutout instead of an opaque square, so the
        # e-ink viewer can blend soft edges straight from the source image
        # rather than faking it with a blurred mask. Left off for the
        # "classic" style, which the HTML display was built around as a
        # plain opaque square.
        body["background"] = "transparent"
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

if __name__ == "__main__":

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
                person_image,
                transparent=False,
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
