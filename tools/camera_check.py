#!/usr/bin/env python3
"""Own a camera-only launch, request one diagnostic detection, then shut it down."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def confidence(value):
    number = float(value)
    if not math.isfinite(number) or not 0 <= number <= 1:
        raise argparse.ArgumentTypeError('confidence must be within 0..1')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--class-id', type=int, default=0)
    parser.add_argument('--confidence', type=confidence, default=0.3)
    args = parser.parse_args()
    if args.class_id < 0:
        parser.error('class-id must be nonnegative')
    import rclpy
    from custom_interface.srv import CameraSrv

    # Never attach to or stop a separately owned camera process.
    names = subprocess.check_output(['ros2', 'node', 'list'], timeout=10, text=True).splitlines()
    if any(name.strip() in ('/camera_server', '/camera/camera', '/camera') for name in names):
        raise RuntimeError('已有相机节点；先在原 camera 窗口 Ctrl+C，再运行 camera-check')
    logs = ROOT / 'log-project' / ('camera-check-' + time.strftime('%Y%m%d-%H%M%S') + '-' + str(os.getpid()))
    logs.mkdir(parents=True)
    print('本次日志与图片:', logs, flush=True)
    env = dict(os.environ, HARVEST_DIAGNOSTIC_DIR=str(logs))
    process = node = client = None
    rclpy.init()
    try:
        with (logs / 'camera.log').open('w') as stream:
            process = subprocess.Popen(['bash', 'scripts/project', 'camera'], cwd=str(ROOT),
                                       env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            node = rclpy.create_node('camera_check_client')
            client = node.create_client(CameraSrv, '/camera_srv')
            deadline = time.monotonic() + 35
            while not client.wait_for_service(timeout_sec=0.5):
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('相机服务未就绪；查看 camera.log')
            frame_deadline = time.monotonic() + 15
            while True:
                request = CameraSrv.Request()
                request.command = 'detect_debug'
                request.identifier = args.class_id
                request.conf = args.confidence
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=75)
                if not future.done():
                    future.cancel()
                    raise RuntimeError('检测响应超过 75 秒；停止本次相机节点')
                response = future.result()
                if response is None:
                    raise RuntimeError('检测服务没有返回结果')
                if response.message != 'No frame available' or time.monotonic() > frame_deadline:
                    break
                print('等待首组 RGB/对齐深度图像...', flush=True)
                time.sleep(0.5)
            result = {'success': response.success, 'message': response.message,
                      'class_id': args.class_id, 'confidence': args.confidence,
                      'coordinates_base_link_m': [dict(x=p.x, y=p.y, z=p.z) for p in response.coordinates]}
            (logs / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
            print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
            if not response.success:
                raise RuntimeError(response.message)
            print('相机检测请求完成；未启动机械臂或夹爪。', flush=True)
    finally:
        if process is not None:
            # Include descendants even if the launch parent has already exited.
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        if client is not None:
            node.destroy_client(client)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('本次相机入口已结束；日志:', logs, flush=True)
        path = logs / 'camera.log'
        if path.exists():
            issues = [line for line in path.read_text(errors='replace').splitlines()
                      if 'Traceback' in line or 'Error' in line or '[ERROR]' in line]
            if issues:
                print('camera.log 提示:\n' + '\n'.join(issues[-8:]), flush=True)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        main()
    except KeyboardInterrupt:
        print('检测已中止', flush=True)
        raise SystemExit(130)
    except Exception as error:
        print('停止:', error, flush=True)
        raise SystemExit(1)
