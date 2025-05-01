# test_sampling.py
import time
import argparse
import kinco_backend

def main():
    parser = argparse.ArgumentParser(description="Test Kinco position sampling rate")
    parser.add_argument('--port', required=True,
                        help='串口名称，例如 Windows 下的 COM11，或 Linux 下的 /dev/ttyUSB0')
    parser.add_argument('--count', type=int, default=1000,
                        help='循环读取次数 (default: 1000)')
    args = parser.parse_args()

    # 打开指定串口
    try:
        kinco_backend.open_port(args.port)
    except Exception as e:
        print(f"无法打开串口 {args.port}: {e}")
        return

    N = args.count
    # 测试开始
    start = time.time()
    for i in range(N):
        # 只读取，不打印，保持最快速度
        kinco_backend.read_pos()
    elapsed = time.time() - start

    avg_ms = elapsed / N * 1000
    freq_hz = 1000 / avg_ms
    print(f"Total {N} reads in {elapsed:.3f}s → avg {avg_ms:.3f} ms/read ({freq_hz:.1f} Hz)")

if __name__ == '__main__':
    main()
