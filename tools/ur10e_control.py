#!/usr/bin/env python3
"""One foreground session owns driver/MoveIt and optionally executes approved Home."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
IP = "192.168.11.60"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = [-180., -90., 127., -123., 270., 0.]
LIMITS = [(-363., -120.), (-125., -40.), (40., 180.),
          (-300., 0.), (180., 363.), (-363., 363.)]


def check_angles(radians):
    if len(radians) != 6:
        raise RuntimeError("关节数据必须为六项")
    angles = [math.degrees(value) for value in radians]
    for name, angle, (low, high) in zip(JOINTS, angles, LIMITS):
        if not math.isfinite(angle) or not low - 0.01 <= angle <= high + 0.01:
            raise RuntimeError("关节超出现场限位: {}={:.3f} 度".format(name, angle))
    return angles


def check_trajectory(trajectory, actual):
    joint = trajectory.joint_trajectory
    if joint.joint_names != JOINTS or not joint.points:
        raise RuntimeError("规划轨迹关节顺序错误或为空")
    if trajectory.multi_dof_joint_trajectory.points:
        raise RuntimeError("Home 不接受多自由度轨迹")
    last_time = -1.
    for point in joint.points:
        check_angles(point.positions)
        seconds = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        if not math.isfinite(seconds) or seconds <= last_time:
            raise RuntimeError("轨迹时间无效")
        last_time = seconds
        for values in (point.velocities, point.accelerations):
            if values and (len(values) != 6 or not all(math.isfinite(v) for v in values)):
                raise RuntimeError("轨迹速度或加速度无效")
    if last_time <= 0 or last_time > 600:
        raise RuntimeError("轨迹时长无效或超过执行上限")
    if max(abs(a - b) for a, b in zip(joint.points[0].positions, actual)) > 0.01:
        raise RuntimeError("规划起点与真机当前角度不一致")
    end = check_angles(joint.points[-1].positions)
    if max(abs(a - b) for a, b in zip(end, HOME)) > 0.1:
        raise RuntimeError("规划终点不是已授权的 Home")
    return last_time


def dashboard():
    with socket.create_connection((IP, 29999), 3) as sock:
        sock.settimeout(3)
        with sock.makefile("r") as stream:
            stream.readline()
            answers = {}
            for command in ("is in remote control", "robotmode", "safetystatus"):
                sock.sendall((command + "\n").encode())
                answers[command] = stream.readline().strip()
    if answers["is in remote control"].lower() != "true":
        raise RuntimeError("示教器必须处于远程控制模式")
    if answers["robotmode"] != "Robotmode: RUNNING":
        raise RuntimeError("请在示教器检查上电与制动: " + answers["robotmode"])
    if answers["safetystatus"] not in ("Safetystatus: NORMAL", "Safetystatus: REDUCED"):
        raise RuntimeError("安全状态不允许运动: " + answers["safetystatus"])
    return answers


def robot_state(receiver):
    return {"runtime_state": receiver.getRuntimeState(),
            "speed_scaling": receiver.getSpeedScaling(),
            "target_fraction": receiver.getTargetSpeedFraction(),
            "protective": receiver.isProtectiveStopped(),
            "emergency": receiver.isEmergencyStopped(),
            "q": list(receiver.getActualQ())}


def check_running(state, low_speed=False):
    if state["protective"] or state["emergency"]:
        raise RuntimeError("保护停止或急停；程序不会自动解锁")
    if state["runtime_state"] != 2:
        raise RuntimeError("机器人控制程序未运行，runtime_state=" + str(state["runtime_state"]))
    scaling = state["speed_scaling"] * state["target_fraction"]
    if not math.isfinite(scaling) or scaling <= 0:
        raise RuntimeError("机器人速度缩放为零或无效")
    if low_speed and scaling > 0.10 + 1e-6:
        raise RuntimeError("请将示教器速度滑块设为 5%，当前组合速度缩放={:.3f}".format(scaling))
    check_angles(state["q"])


def existing_control():
    found = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            executable = path.read_bytes().split(b"\0", 1)[0].decode(errors="replace")
            if Path(executable).name in ("ros2_control_node", "move_group"):
                found.append((path.parent.name, executable))
        except (OSError, ValueError):
            pass
    return found


def wait_future(node, future, seconds):
    import rclpy
    end = time.monotonic() + seconds
    while not future.done() and time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise RuntimeError("ROS 请求等待超时")
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--home", action="store_true", help="执行已授权 Home；默认只检查")
    args = parser.parse_args()
    import rclpy
    from rclpy.action import ActionClient
    from rtde_receive import RTDEReceiveInterface
    from controller_manager_msgs.srv import ListControllers
    from moveit_msgs.action import MoveGroup, ExecuteTrajectory
    from moveit_msgs.msg import Constraints, JointConstraint
    from action_msgs.msg import GoalStatus

    logs = ROOT / "log-project" / ("control-" + time.strftime("%Y%m%d-%H%M%S") + "-" + str(os.getpid()))
    logs.mkdir(parents=True)
    print("本次日志:", logs, flush=True)
    children = []
    files = []
    receiver = node = active = None
    clients = []
    rclpy.init()
    try:
        # A previous launch may need a few seconds to finish its normal shutdown.
        deadline = time.monotonic() + 8
        while existing_control() and time.monotonic() < deadline:
            time.sleep(0.5)
        old = existing_control()
        if old:
            raise RuntimeError("仍有其他控制进程；不重复启动、不终止它们: " + str(old))
        print("示教器状态:", dashboard(), flush=True)

        def launch(label, filename, extra):
            stream = (logs / (label + ".log")).open("w")
            files.append(stream)
            command = ["ros2", "launch", "end_effector_description", filename,
                       "ur_type:=ur10e", "robot_ip:=" + IP, "use_fake_hardware:=false",
                       "launch_rviz:=false", "robot_controller:=scaled_joint_trajectory_controller"] + extra
            process = subprocess.Popen(command, cwd=str(ROOT), stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            children.append(process)
            return process

        driver = launch("driver", "foxy_ur_control.launch.py", ["headless_mode:=true"])
        node = rclpy.create_node("ur10e_home_entry")
        service = node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not service.wait_for_service(timeout_sec=25):
            raise RuntimeError("驱动服务未就绪，请看本次 driver.log")
        text = subprocess.check_output(["ros2", "param", "get", "/controller_manager", "robot_description"],
                                       timeout=8, text=True)
        description = ET.fromstring(text[text.index("<"):])
        parameters = {p.get("name"): p.text for p in description.findall("./ros2_control/hardware/param")}
        if parameters.get("headless_mode") != "1":
            raise RuntimeError("驱动实际 headless_mode 不是 1；停止执行")
        receiver = RTDEReceiveInterface(IP)
        deadline = time.monotonic() + 20
        last_error = ""
        while time.monotonic() < deadline:
            if driver.poll() is not None:
                raise RuntimeError("驱动退出，请看本次 driver.log")
            try:
                state = robot_state(receiver)
                check_running(state, low_speed=args.home)
                response = wait_future(node, service.call_async(ListControllers.Request()), 3)
                if not any(c.name == "scaled_joint_trajectory_controller" and c.state == "active"
                           for c in response.controller):
                    raise RuntimeError("scaled 控制器未激活")
                connections = subprocess.check_output(["ss", "-tnH"], timeout=3, text=True)
                port = ":" + str(int(parameters["reverse_port"]))
                if not any(len(parts) >= 5 and parts[0] == "ESTAB" and parts[3].endswith(port)
                           and parts[4].startswith(IP + ":")
                           for parts in (line.split() for line in connections.splitlines())):
                    raise RuntimeError("机器人 reverse 控制连接未建立")
                break
            except RuntimeError as error:
                last_error = str(error)
                time.sleep(0.5)
        else:
            raise RuntimeError(last_error)
        print("驱动就绪: headless=1，程序运行，reverse 已连接，scaled 控制器 active", flush=True)
        print("当前关节角(度):", [round(v, 2) for v in check_angles(state["q"])], flush=True)
        if not args.home:
            print("检查完成，未发送运动", flush=True)
            return

        moveit = launch("moveit", "foxy_moveit.launch.py", [])
        planning = ActionClient(node, MoveGroup, "/move_action")
        execution = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        clients.extend([planning, execution])
        if not planning.wait_for_server(timeout_sec=25) or not execution.wait_for_server(timeout_sec=10):
            raise RuntimeError("MoveIt action 未就绪，请看本次 moveit.log")
        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 15.
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        for name, angle in zip(JOINTS, HOME):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = math.radians(angle)
            joint.tolerance_above = joint.tolerance_below = 0.001
            joint.weight = 1.
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        print("规划 Home:", HOME, "度；尚未发送运动", flush=True)
        active = wait_future(node, planning.send_goal_async(goal), 8)
        if not active.accepted:
            raise RuntimeError("规划请求被拒绝")
        planned = wait_future(node, active.get_result_async(), 25)
        active = None
        if planned.status != GoalStatus.STATUS_SUCCEEDED or planned.result.error_code.val != 1:
            raise RuntimeError("Home 规划失败，MoveIt code=" + str(planned.result.error_code.val))
        dashboard()
        state = robot_state(receiver)
        check_running(state, low_speed=True)
        duration = check_trajectory(planned.result.planned_trajectory, state["q"])
        print("规划已核对全部轨迹点现场限位，名义时长 {:.2f} 秒；开始执行".format(duration), flush=True)
        execute = ExecuteTrajectory.Goal()
        execute.trajectory = planned.result.planned_trajectory
        if hasattr(execute, "controller_names"):
            execute.controller_names = ["scaled_joint_trajectory_controller"]
        active = wait_future(node, execution.send_goal_async(execute), 8)
        if not active.accepted:
            raise RuntimeError("执行请求被拒绝")
        result_future = active.get_result_async()
        deadline = time.monotonic() + 600
        progress_time = time.monotonic()
        progress_q = state["q"]
        report = 0.
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError("Home 执行超过 600 秒，取消")
            if driver.poll() is not None or moveit.poll() is not None:
                raise RuntimeError("驱动或 MoveIt 退出，取消")
            state = robot_state(receiver)
            check_running(state, low_speed=True)
            if max(abs(a - b) for a, b in zip(state["q"], progress_q)) >= math.radians(0.05):
                progress_q, progress_time = state["q"], now
            if now - progress_time > 15:
                raise RuntimeError("15 秒无关节进展，取消 Home")
            if now - report > 10:
                angles = check_angles(state["q"])
                print("执行中，距 Home 最大误差 {:.2f} 度".format(
                    max(abs(a - b) for a, b in zip(angles, HOME))), flush=True)
                report = now
        finished = result_future.result()
        active = None
        if finished.status != GoalStatus.STATUS_SUCCEEDED or finished.result.error_code.val != 1:
            raise RuntimeError("Home 未成功，MoveIt code=" + str(finished.result.error_code.val))
        final = check_angles(receiver.getActualQ())
        if max(abs(a - b) for a, b in zip(final, HOME)) > 0.5:
            raise RuntimeError("action 报成功，但实测关节未到 Home: " + str(final))
        (logs / "home-result.json").write_text(json.dumps(
            {"success": True, "target_deg": HOME, "actual_deg": final}, indent=2))
        print("Home 完成，实测关节角(度):", [round(v, 2) for v in final], flush=True)
    finally:
        if active is not None and active.accepted and node is not None and rclpy.ok():
            try:
                wait_future(node, active.cancel_goal_async(), 5)
                print("已请求取消当前 action", flush=True)
            except Exception as error:
                print("取消请求异常:", error, flush=True)
        # Signal only the process groups created and owned by this invocation.
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(children):
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        if receiver is not None:
            receiver.disconnect()
        for client in clients:
            client.destroy()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for stream in files:
            stream.close()
        print("本入口创建的控制进程已停止；本次日志:", logs, flush=True)
        for filename in ("driver.log", "moveit.log"):
            path = logs / filename
            if path.exists():
                errors = [line for line in path.read_text(errors="replace").splitlines()
                          if "[ERROR]" in line or "[FATAL]" in line]
                if errors:
                    print(filename + " 错误:\n" + "\n".join(errors[-8:]), flush=True)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        main()
    except KeyboardInterrupt:
        print("已中止，未宣告 Home 完成", flush=True)
        raise SystemExit(130)
    except Exception as error:
        print("停止:", error, flush=True)
        raise SystemExit(1)
