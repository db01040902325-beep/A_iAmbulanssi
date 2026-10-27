"""노트북/라즈베리파이 카메라 원본 저장. 노면 충격을 예측하는 AI는 아직 아님.

실행: pip install opencv-python; python tools/camera_record.py --seconds 10
"""
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import time


def main():
    p = argparse.ArgumentParser(description='노면 영상 데이터 수집용')
    p.add_argument('--device', type=int, default=0)
    p.add_argument('--seconds', type=float, default=10)
    p.add_argument('--output', default='recordings')
    args = p.parse_args()
    if args.seconds <= 0:
        raise SystemExit('--seconds should be positive')
    try:
        import cv2
    except ImportError:
        raise SystemExit('OpenCV 설치가 필요합니다: pip install opencv-python')
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise SystemExit('카메라를 열 수 없습니다. --device 번호와 접근권한을 확인하세요.')
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    fps = 20
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(str(out / 'road_video.mp4'), fourcc, fps, (width, height))
    start = time.monotonic()
    count = 0
    try:
        with (out / 'frames.csv').open('w', newline='', encoding='utf-8') as stream:
            rows = csv.writer(stream)
            rows.writerow(['frame_index', 'monotonic_elapsed_s', 'wall_time_utc'])
            while time.monotonic() - start < args.seconds:
                ok, frame = cap.read()
                if not ok:
                    break
                writer.write(frame)
                rows.writerow([count, f'{time.monotonic() - start:.6f}', datetime.now(timezone.utc).isoformat()])
                count += 1
    finally:
        writer.release()
        cap.release()
    print(f'{count} 프레임 기록: {out.resolve()}')
    print('주의: frame timestamps are approximate; accurate AI training needs synchronized IMU/actuator labels.')


if __name__ == '__main__':
    main()
