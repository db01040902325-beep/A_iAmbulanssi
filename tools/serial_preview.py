"""ESP32 C++ 출력 없는 프로토콜 수신 시험 (실제 구동 없음).

사용법: pip install pyserial
python tools/serial_preview.py --port COM5 (Windows)
python tools/serial_preview.py --port /dev/ttyUSB0 (Ubuntu)
"""
import argparse
import time


def main():
    p = argparse.ArgumentParser(description='ESP32 PREVIEW_ONLY 시리얼 명령 점검')
    p.add_argument('--port', required=True)
    p.add_argument('--baud', type=int, default=115200)
    args = p.parse_args()
    try:
        import serial
    except ImportError:
        raise SystemExit('pyserial 설치: python -m pip install pyserial')
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        time.sleep(2)  # ESP32 reset 가능
        ser.reset_input_buffer()
        # 실제 모터 명령이 아니라 펌웨어의 파싱 및 스위치 응답만 검증
        command = b'CMD,1,0.0,0.0\n'
        ser.write(command)
        reply = ser.readline().decode('utf-8', errors='replace').strip()
        print('전송:', command.decode().strip())
        print('응답:', reply or '(timeout)')
        print('실제 모터는 이 펌웨어에서 구동되지 않습니다.')


if __name__ == '__main__':
    main()
