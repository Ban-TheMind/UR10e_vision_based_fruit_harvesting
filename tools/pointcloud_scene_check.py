#!/usr/bin/env python3
"""Inspect the guarded map and optionally send an existing plan-only command."""
import argparse
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default=str(ROOT / 'src/harvesting_bringup/config/site.json'))
    parser.add_argument('--plan-joint', nargs=6, type=float, metavar='RAD')
    parser.add_argument('--plan-cartesian', nargs=6, type=float, metavar='XYZ_RPY')
    parser.add_argument('--clear-map', action='store_true', help='Clear OctoMap before reacquiring observations; never moves hardware')
    parser.add_argument('--timeout', type=float, default=35.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    if args.plan_joint is not None and args.plan_cartesian is not None:
        parser.error('Choose one plan target')
    positions = args.plan_joint if args.plan_joint is not None else args.plan_cartesian
    if positions is not None and not all(math.isfinite(v) for v in positions):
        parser.error('Plan target must contain six finite numbers')
    from harvesting_config import deployment
    settings = deployment.resolve_deployment(args.profile, 'mapping').perception

    import rclpy
    from std_srvs.srv import Trigger, Empty
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    from custom_interface.srv import MovementRequest
    rclpy.init()
    node = rclpy.create_node('pointcloud_scene_check')

    def call(service_type, name, request, timeout):
        client = node.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=min(timeout, 5.)):
                raise RuntimeError('Service unavailable: ' + name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.exception() is not None:
                raise RuntimeError('Service timed out or failed: ' + name)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        if args.clear_map:
            call(Empty, '/clear_octomap', Empty.Request(), args.timeout)
            # The updater will repopulate the scene; readiness retries below.
            print('OctoMap cleared; waiting for fresh processing and a populated scene.')
        deadline = time.monotonic() + args.timeout
        ready = None
        while time.monotonic() < deadline:
            ready = call(Trigger, '/pointcloud_scene_ready', Trigger.Request(), min(5., args.timeout))
            print(ready.message, flush=True)
            if ready.success:
                break
            time.sleep(0.5)
        if ready is None or not ready.success:
            raise RuntimeError('Point-cloud scene not ready; no planning request sent')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.WORLD_OBJECT_NAMES
        scene = call(GetPlanningScene, '/get_planning_scene', request, args.timeout).scene
        world_map = scene.world.octomap
        old_ids = {'backWall', 'sideWall', 'table', 'ceiling'}
        leftovers = old_ids.intersection(obj.id for obj in scene.world.collision_objects)
        if leftovers:
            raise RuntimeError('Legacy hard-coded obstacles remain in running scene: ' + ', '.join(sorted(leftovers)) + '; restart move_group')
        if world_map.header.frame_id != settings['octomap_frame'] or not world_map.octomap.data:
            raise RuntimeError('Actual planning scene has an empty map or unexpected frame')
        print('Scene map: frame={}, resolution={}m, bytes={}'.format(
            world_map.header.frame_id, world_map.octomap.resolution, len(world_map.octomap.data)))
        if positions is not None:
            command = 'plan_joint' if args.plan_joint is not None else 'plan_cartesian'
            target = MovementRequest.Request()
            target.command, target.positions, target.constraints_identifier = command, positions, 'NONE'
            response = call(MovementRequest, '/moveit_path_plan', target, args.timeout)
            print('Plan-only result: {}'.format(response.success))
            if not response.success:
                raise RuntimeError('Plan rejected or failed; inspect planning-server reason and scene collision state')
        print('PASS map readiness/scene inspection; no execution command was sent.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError, OSError, ImportError) as error:
        sys.exit('FAIL: ' + str(error))
