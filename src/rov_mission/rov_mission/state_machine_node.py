#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros

from rov_mission.behaviors import (
    CaptureHomePose, ArmThrusters, DiveToDepth, StartMapping, PublishGoal, MonitorMission,
)

STAGE_DESCRIPTIONS = {
    'CaptureHomePose': 'Zapamiętywanie pozycji startowej (home)',
    'ArmThrusters': 'Uzbrajanie pędników',
    'DiveToTargetDepth': 'Nurkowanie na głębokość docelową',
    'StartMapping': 'Oczekiwanie na dane z mapowania 3D (OctoMap)',
    'PublishGoal': 'Publikowanie celu misji',
    'MonitorGoalApproach': 'Dojazd do celu misji',
    'PublishHomeGoal': 'Publikowanie celu powrotu do bazy',
    'MonitorReturnHome': 'Powrót do pozycji startowej',
}


def make_stage_logger(node):
    """Logs the currently active leaf behaviour whenever it changes, so the
    console shows which mission stage is running without spamming every tick."""
    last_stage = {'name': None}

    def handler(tree):
        active = next(
            (b for b in tree.root.iterate()
             if not getattr(b, 'children', None) and b.status == py_trees.common.Status.RUNNING),
            None,
        )
        stage = active.name if active else None
        if stage != last_stage['name']:
            last_stage['name'] = stage
            if stage is not None:
                node.get_logger().info(f'Etap misji: {STAGE_DESCRIPTIONS.get(stage, stage)}')

    return handler


def create_root(node) -> py_trees.behaviour.Behaviour:
    goal_x = node.get_parameter('goal.x').value
    goal_y = node.get_parameter('goal.y').value
    goal_z = node.get_parameter('goal.z').value
    goal_yaw = node.get_parameter('goal.yaw').value

    root = py_trees.composites.Sequence(name="MissionRoot", memory=True)

    go_to_goal = py_trees.composites.Sequence(name="GoToGoal", memory=True)
    go_to_goal.add_children([
        PublishGoal(name="PublishGoal", static_goal=(goal_x, goal_y, goal_z, goal_yaw)),
        MonitorMission(name="MonitorGoalApproach", static_goal=(goal_x, goal_y, goal_z, goal_yaw)),
    ])

    return_home = py_trees.composites.Sequence(name="ReturnHome", memory=True)
    return_home.add_children([
        PublishGoal(name="PublishHomeGoal", key="home_pose"),
        MonitorMission(name="MonitorReturnHome", key="home_pose"),
    ])

    root.add_children([
        CaptureHomePose(),
        ArmThrusters(),
        DiveToDepth(target_z=node.get_parameter('cruise_depth').value),
        StartMapping(),
        go_to_goal,
        return_home,
    ])
    # Without this, a Sequence that has reached SUCCESS/FAILURE restarts itself on
    # the very next tick_tock tick (memory only persists status while RUNNING) - the
    # whole mission would loop forever instead of running once as PLAN.md describes.
    return py_trees.decorators.OneShot(
        name="MissionOnce", child=root, policy=py_trees.common.OneShotPolicy.ON_COMPLETION)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('mission_params_node')
    node.declare_parameter('map_frame', 'odom')
    node.declare_parameter('base_frame', 'base_link')
    node.declare_parameter('cruise_depth', -1.5)
    node.declare_parameter('goal.x', 3.0)
    node.declare_parameter('goal.y', 3.0)
    node.declare_parameter('goal.z', -1.5)
    node.declare_parameter('goal.yaw', 0.0)

    root = create_root(node)
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    try:
        tree.setup(node=node, timeout=15.0)
    except Exception as exc:
        node.get_logger().error(f'Behavior tree initialization failed: {exc}')
        rclpy.shutdown()
        return

    tree.add_post_tick_handler(make_stage_logger(node))
    tree.tick_tock(period_ms=200.0)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
