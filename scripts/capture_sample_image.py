# scripts/capture_sample_image.py
import cv2
import os
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default=0, help="カメラのデバイス番号 (/dev/videoX の X)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--participant-id", required=True, help="保存先ファイル名のID (例: 15)")
    parser.add_argument(
        "--raw-dir",
        default="/home/leus/ros/catkin_ws/src/plush_memory/data/images/raw_picture",
        help="sample_image_{id}.jpg を置くディレクトリ")
    args = parser.parse_args()

    os.makedirs(args.raw_dir, exist_ok=True)
    out_path = os.path.join(args.raw_dir, f"sample_image_{args.participant_id}.jpg")

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"カメラが開けませんでした: /dev/video{args.device}")

    # 解像度指定（対応しないカメラもあります）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # 必要に応じて

    # 露光・ホワイトバランスの安定待ちに数フレーム捨てる
    for _ in range(5):
        cap.read()

    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("フレームの取得に失敗しました")

    # OpenCVはBGR→JPEGそのままでOK
    if not cv2.imwrite(out_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
        raise RuntimeError(f"保存に失敗しました: {out_path}")

    print(f"Saved: {out_path}")

if __name__ == "__main__":
    main()
