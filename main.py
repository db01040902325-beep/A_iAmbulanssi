"""실행: python main.py --plot"""
import argparse
import csv
from pathlib import Path
from bedcontrol.simulation import simulate, summary


def main():
    parser = argparse.ArgumentParser(description='구급차 침대 2자유도 실험용 시뮬레이터')
    parser.add_argument('--out', default='results', help='CSV/PNG 저장 폴더')
    parser.add_argument('--plot', action='store_true', help='matplotlib 설치 시 그래프 출력 파일 생성')
    args = parser.parse_args()
    output = Path(args.out)
    output.mkdir(parents=True, exist_ok=True)
    modes = ('passive', 'feedback', 'preview')
    datasets = {mode: simulate(mode) for mode in modes}
    for mode, rows in datasets.items():
        with (output / f'{mode}.csv').open('w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        score = summary(rows)
        print(f'{mode:8s} | heave RMS={score["heave_rms_mm"]:.3f}mm | pitch RMS={score["pitch_rms_deg"]:.3f}°')
    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print('matplotlib 없음: 그래프는 생략합니다. 설치: pip install matplotlib')
        else:
            # 각각 개별 차트로 출력
            for key, name, ylabel in (('bed_z_mm', 'heave', 'Heave displacement (mm)'),
                                       ('bed_pitch_deg', 'pitch', 'Pitch angle (deg)')):
                fig, ax = plt.subplots(figsize=(10, 4))
                for mode in modes:
                    rows = datasets[mode]
                    ax.plot([r['time_s'] for r in rows], [r[key] for r in rows], label=mode)
                ax.set_xlabel('Time (s)')
                ax.set_ylabel(ylabel)
                ax.legend()
                ax.grid(True, alpha=0.3)
                fig.tight_layout()
                fig.savefig(output / f'{name}.png', dpi=150)
                plt.close(fig)
            print(f'그래프 저장: {output.resolve()}')
    print('주의: preview는 이상적 미래 노면 데이터로, 실제 학습된 AI가 아닙니다.')


if __name__ == '__main__':
    main()
