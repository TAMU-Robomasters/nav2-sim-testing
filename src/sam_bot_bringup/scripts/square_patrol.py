#!/usr/bin/env python3
"""
Square patrol — drive the robot in a repeating square using Nav2 waypoints.

Launch the full navigation stack first (in another terminal):

    ros2 launch sam_bot_bringup real_lidar_navigate.launch.py use_rviz:=true

then run this script:

    ros2 run sam_bot_bringup square_patrol.py

It connects to the already-running Nav2 action servers via the
nav2_simple_commander API, waits until the stack is ACTIVE, then loops the
four corners forever (Ctrl-C to stop).

This robot is holonomic and its yaw is owned by the embedded MCU, so the
waypoint orientations are irrelevant (the goal checker uses
yaw_goal_tolerance > pi). We leave orientation at identity.
"""

from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Square corners in the map frame (metres), visited in order. The robot starts
# at (0, 0) — matching amcl's set_initial_pose in nav2_params_real_lidar.yaml.
SQUARE_CORNERS = [
    (0.601, 1.63),
    (0.603, -1.79),
    (3.98, -1.29),
    (3.85, 1.6),
]


def make_pose(nav, x, y):
    """Build a map-frame PoseStamped at (x, y) with identity orientation."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0  # yaw is ignored on this robot
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    # amcl auto-initializes at (0, 0, 0) via set_initial_pose:true in the YAML.
    # We publish the SAME pose here so waitUntilNav2Active()'s internal
    # _waitForInitialPose() agrees with the YAML instead of fighting it.
    nav.setInitialPose(make_pose(nav, 0.0, 0.0))

    nav.waitUntilNav2Active()  # blocks until amcl + bt_navigator are ACTIVE
    nav.get_logger().info("Nav2 active — starting square patrol")

    waypoints = [make_pose(nav, x, y) for x, y in SQUARE_CORNERS]

    lap = 0
    try:
        while rclpy.ok():
            lap += 1
            nav.get_logger().info(f"Starting lap {lap}")
            # Re-stamp each lap so the poses carry a current timestamp.
            stamped = [deepcopy(make_pose(nav, p.pose.position.x, p.pose.position.y))
                       for p in waypoints]
            nav.followWaypoints(stamped)

            while not nav.isTaskComplete():
                feedback = nav.getFeedback()
                if feedback:
                    nav.get_logger().info(
                        f"  heading to waypoint {feedback.current_waypoint + 1}"
                        f"/{len(stamped)}",
                        throttle_duration_sec=2.0,
                    )

            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                nav.get_logger().info(f"Lap {lap} complete")
            else:
                nav.get_logger().warn(f"Lap {lap} ended early: {result}")
    except KeyboardInterrupt:
        nav.get_logger().info("Interrupted — cancelling current task")
        nav.cancelTask()
    finally:
        # Note: we deliberately do NOT call lifecycleShutdown() — that would
        # tear down the Nav2 nodes this script doesn't own. Just shut down rclpy.
        rclpy.shutdown()


if __name__ == "__main__":
    main()
