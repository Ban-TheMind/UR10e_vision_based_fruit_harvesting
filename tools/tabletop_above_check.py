#!/usr/bin/env python3
"""One compensated above-target trial against an existing project all launch."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/demo_package'))
from demo_package.tabletop_geometry import tabletop_poses, rotate_offset


def quaternion_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Move to Home, detect once, then move above; never operate the gripper')
    parser.add_argument('--height', type=float, required=True, help='Taught tool0 Z in base_link, meters (not fingertip height)')
    parser.add_argument('--rpy', nargs=3, type=float, required=True, help='Taught tool0 RPY in base_link, radians')
    parser.add_argument('--xy-offset', nargs=2, type=float, default=[0.0, 0.0], help='Residual gripper-center XY correction, meters')
    parser.add_argument('--profile', default=os.environ.get('HARVEST_PROFILE', str(ROOT / 'src/harvesting_bringup/config/site.json')))
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('deployment', ROOT / 'src/harvesting_bringup/launch/deployment.py')
    deployment = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(deployment)
    profile = deployment.load_profile(args.profile)
    geometry = deployment.tabletop_parameters(profile)
    offset = geometry.get('tool_offset_m', [0.0, 0.0, 0.1628])
    clearance = geometry.get('clearance_m', 0.12)
    from types import SimpleNamespace
    tabletop_poses(SimpleNamespace(x=0, y=0, z=0), args.height, args.rpy, args.xy_offset, clearance, offset)

    import rclpy
    from custom_interface.srv import CameraSrv, MovementRequest
    from moveit_msgs.srv import GetPositionFK
    from demo_package.routine_base import configure_execution_timeout
    from demo_package.tabletop_geometry import rpy_rotation

    logs = ROOT / 'log-project' / ('above-check-' + time.strftime('%Y%m%d-%H%M%S') + '-' + str(os.getpid()))
    logs.mkdir(parents=True)
    report = {'execute': args.execute, 'height_reference': 'tool0/base_link', 'tool_offset_m': offset}
    print('本次记录:', logs, flush=True)
    rclpy.init()
    node = rclpy.create_node('tabletop_above_check')
    try:
        movement = node.create_client(MovementRequest, '/moveit_path_plan')
        camera = node.create_client(CameraSrv, '/camera_srv')
        fk = node.create_client(GetPositionFK, '/compute_fk')
        for client in (movement, camera, fk):
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('服务未就绪: ' + client.srv_name + '；先启动 project all')

        def exchange(client, request, seconds):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
            if not future.done():
                future.cancel()
                raise RuntimeError('请求超时: ' + client.srv_name + '；如运动已开始请现场停止检查')
            result = future.result()
            if result is None:
                raise RuntimeError('服务无响应: ' + client.srv_name)
            return result

        def move(target, command):
            req = MovementRequest.Request()
            req.command, req.positions, req.constraints_identifier = command, list(target), 'NONE'
            response = exchange(movement, req, deployment.execution_timeout(profile)+60)
            if not response.success:
                raise RuntimeError('规划/运动失败: ' + command)

        if args.execute:
            # Receive-only interface: never instantiate RTDEControlInterface.
            from rtde_receive import RTDEReceiveInterface
            configure_execution_timeout(node, deployment.execution_timeout(profile))
            print('先移动到 Home，再检测一次；之后只到上方，不下降、不操作夹爪', flush=True)
            move(profile['task']['birds_eye_joint_pos'], 'joint')
            time.sleep(1.5)

        req = CameraSrv.Request()
        req.command, req.identifier, req.conf = 'detect_debug', 0, 0.3
        deadline = time.monotonic()+20
        while True:
            detected = exchange(camera, req, 75)
            if detected.message != 'No frame available' or time.monotonic() >= deadline:
                break
            time.sleep(0.5)
        report['camera_message'] = detected.message
        if not detected.success or not detected.coordinates:
            raise RuntimeError('没有有效苹果，未发送上方运动: ' + detected.message)
        apple = detected.coordinates[0]
        target, pick = tabletop_poses(apple, args.height, args.rpy, args.xy_offset, clearance, offset)
        report.update(apple_base_link_m=[apple.x, apple.y, apple.z], target_tool0=target, pick_tool0=pick)
        print('苹果 base_link XYZ:', report['apple_base_link_m'], flush=True)
        print('补偿后 tool0 上方目标:', target, flush=True)
        move(target, 'plan_cartesian')
        if not args.execute:
            print('规划成功；未执行运动。加 --execute 后会先 Home 再检测并执行上方接近。', flush=True)
            return
        move(target, 'cartesian')

        robot = RTDEReceiveInterface(profile['robot']['robot_ip'])
        try:
            joints = robot.getActualQ()
            report['controller_tcp_base_xyz_rotvec'] = robot.getActualTCPPose()
        finally:
            robot.disconnect()
        report['actual_joints_deg'] = [math.degrees(v) for v in joints]
        req = GetPositionFK.Request()
        req.header.frame_id, req.fk_link_names = 'base_link', ['tool0']
        req.robot_state.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        req.robot_state.joint_state.position = joints
        response = exchange(fk, req, 15)
        if response.error_code.val != 1 or len(response.pose_stamped) != 1:
            raise RuntimeError('运动结束，但 FK 查询失败；不要继续下降')
        pose = response.pose_stamped[0].pose
        actual = [pose.position.x, pose.position.y, pose.position.z]
        q = pose.orientation
        rotation = quaternion_rotation(q)
        wanted = rpy_rotation(args.rpy)
        # trace(R_goal.T * R_actual) == sum of corresponding matrix entries.
        cosine = (sum(wanted[i][j]*rotation[i][j] for i in range(3) for j in range(3))-1)/2
        angle = math.degrees(math.acos(max(-1, min(1, cosine))))
        position_error = math.sqrt(sum((actual[i]-target[i])**2 for i in range(3)))
        delta = rotate_offset(rotation, offset)
        report.update(actual_tool0_base_link_xyz_m=actual,
                      actual_tool0_quaternion_xyzw=[q.x, q.y, q.z, q.w],
                      modeled_gripper_center_base_link_m=[actual[i]+delta[i] for i in range(3)],
                      position_error_mm=position_error*1000, orientation_error_deg=angle)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if position_error > 0.005 or angle > 1.0:
            raise RuntimeError('实际 tool0 未达到目标；检查上述记录，不继续下降')
        print('上方接近完成并立即核对位姿；请保持当前位置观察 XY 对齐。', flush=True)
    except BaseException as exc:
        report['error'] = str(exc)
        raise
    finally:
        (logs / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('客户端已中断；如机械臂仍运动请现场停止。', flush=True)
        raise SystemExit(130)
    except Exception as exc:
        print('停止:', exc, flush=True)
        raise SystemExit(1)
