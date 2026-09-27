#!/usr/bin/env python3
"""
Waypoint patrol — drive the robot through a set of Nav2 waypoints continuously.

Launch the full navigation stack first (in another terminal):

    ros2 launch sam_bot_bringup real_lidar_navigate.launch.py use_rviz:=true

then run this script:

    ros2 run sam_bot_bringup square_patrol.py

It connects to the already-running Nav2 action servers via the
nav2_simple_commander API, waits until the stack is ACTIVE, and loops through
the SQUARE_CORNERS waypoints infinitely (Ctrl-C to abort). 

If Nav2 fails a task, the script will force a restart and keep trying.

This robot is holonomic and its yaw is owned by the embedded MCU, so the
waypoint orientations are irrelevant (the goal checker uses
yaw_goal_tolerance > pi). We leave orientation at identity.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Square corners in the map frame (metres), visited in order. The robot starts
# at (0, 0) — matching amcl's set_initial_pose in nav2_params_real_lidar.yaml.
SQUARE_CORNERS = [
    (6.33, -1.08),
    (5.14, -1.09),
    (5.03, 0.781),
    (6.44, 0.853),
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

    # amcl owns the pose estimate: it auto-initializes via set_initial_pose:true
    # in nav2_params_real_lidar.yaml (and can be re-localized from RViz). We must
    # NOT publish an initial pose here — that would reset amcl's estimate.
    #
    # waitUntilNav2Active(localizer='amcl') internally runs _waitForInitialPose(),
    # which republishes BasicNavigator's initial pose *only while it hasn't yet
    # seen /amcl_pose*. So we first spin until amcl's own pose arrives; that flips
    # initial_pose_received=True, and the wait then skips the republish entirely.
    nav.info("Waiting for /amcl_pose (set by the launch, not by this script)...")
    while rclpy.ok() and not nav.initial_pose_received:
        rclpy.spin_once(nav, timeout_sec=1.0)

    nav.waitUntilNav2Active()  # blocks until amcl + bt_navigator are ACTIVE
    nav.get_logger().info("Nav2 active — starting infinite square patrol")

    waypoints = [make_pose(nav, x, y) for x, y in SQUARE_CORNERS]
    loop_count = 1

    try:
        while rclpy.ok():
            nav.get_logger().info(
                f"--- Starting patrol loop {loop_count} with {len(waypoints)} waypoints ---"
            )
            
            # Refresh the timestamps for the new loop iteration to avoid TF extrapolation errors
            for pt in waypoints:
                pt.header.stamp = nav.get_clock().now().to_msg()

            nav.followWaypoints(waypoints)

            while not nav.isTaskComplete():
                feedback = nav.getFeedback()
                if feedback:
                    nav.get_logger().info(
                        f"  heading to waypoint {feedback.current_waypoint + 1}"
                        f"/{len(waypoints)}",
                        throttle_duration_sec=2.0,
                    )

            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                nav.get_logger().info(f"Completed patrol loop {loop_count}. Restarting...")
            else:
                # Nav2 gave up, but we are going to force it to try the loop again anyway
                nav.get_logger().warn(f"Patrol ended early with result: {result}. Forcing restart...")
            
            loop_count += 1

    except KeyboardInterrupt:
        nav.get_logger().info("Interrupted — cancelling current task")
        nav.cancelTask()
    finally:
        # Note: we deliberately do NOT call lifecycleShutdown() — that would
        # tear down the Nav2 nodes this script doesn't own. Just shut down rclpy.
        rclpy.shutdown()


if __name__ == "__main__":
    main()
