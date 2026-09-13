#!/usr/bin/env python3
"""Waypoint mission executor for the TurtleBot3 warehouse navigation task.

Drives the robot through an ordered sequence of Nav2 goals:
    Home -> Loading (30s hold) -> Storage -> Shipping -> Home

Publishes a MarkerArray on /waypoint_markers so each station renders as
blue (inactive) or green (active) in RViz.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray


class MissionNode(Node):
    """Sends the ordered waypoint mission and tracks marker state."""

    def __init__(self):
        super().__init__('mission_node')

        # Action client for the /navigate_to_pose action
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Marker publisher: one sphere + label per station, blue until
        # active, green while it is the current Nav2 goal.
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

        # station_location = (x, y, yaw), all in the map frame
        self.home = (0.0, 0.0, 0.0)
        loading = (12.0, 0.0, 0.0)
        storage = (12.0, 5.0, 0.0)
        shipping = (3.0, 5.0, 0.0)

        # The required mission route, in order.
        self.stations = [
            {'name': 'loading', 'pose': loading, 'id': 0, 'wait': 30.0},
            {'name': 'storage', 'pose': storage, 'id': 1, 'wait': 0.0},
            {'name': 'shipping', 'pose': shipping, 'id': 2, 'wait': 0.0},
            {'name': 'charging', 'pose': self.home, 'id': 3, 'wait': 0.0},
        ]

        # All markers start blue; each turns green once it becomes the
        # active goal, then reverts to blue once the next goal is set.
        self.marker_color = {s['name']: (0.0, 0.0, 1.0) for s in self.stations}

        self.current_index = 0

    # Markers
    def publish_markers(self):
        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for s in self.stations:
            x, y, yaw = s['pose']
            r, g, b = self.marker_color[s['name']]
            is_active = (g == 1.0)  # green means "active navigation goal"

            # Sphere marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'stations'
            marker.id = s['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.5
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0).to_msg()  # 0 = forever
            array.markers.append(marker)

            # Text label: "Active" (green) / "Inactive" (blue)
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = now
            text_marker.ns = 'station_labels'
            text_marker.id = s['id'] + 100  # offset so ids don't collide with spheres
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.6  # float above the sphere
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # text height
            text_marker.color.r = r
            text_marker.color.g = g
            text_marker.color.b = b
            text_marker.color.a = 1.0
            text_marker.text = s['name'].capitalize() + (' (Active)' if is_active else '(Inactive)')
            text_marker.lifetime = Duration(seconds=0).to_msg()
            array.markers.append(text_marker)

        self.marker_pub.publish(array)

    def one_shot(self, seconds, callback):
        timer_box = {}

        def _fire():
            timer_box['timer'].cancel()
            self.destroy_timer(timer_box['timer'])
            callback()

        timer_box['timer'] = self.create_timer(seconds, _fire)

    # Navigation sequence
    def start(self):
        self.get_logger().info('Mission started at Charging Station.')
        self.send_current_goal()

    def send_current_goal(self):
        if self.current_index >= len(self.stations):
            self.get_logger().info('Mission complete. Robot returned to Charging Station.')
            rclpy.shutdown()
            return

        # Revert the previous station back to inactive/blue.
        if self.current_index > 0:
            prev_name = self.stations[self.current_index - 1]['name']
            self.marker_color[prev_name] = (0.0, 0.0, 1.0)

        # Set this station as the active goal (green) BEFORE navigating to it.
        station = self.stations[self.current_index]
        self.marker_color[station['name']] = (0.0, 1.0, 0.0)
        self.publish_markers()

        x, y, yaw = station['pose']
        self.get_logger().info(f'Navigating to {station["name"].capitalize()} Station...')
        self.send_goal(x, y, yaw, station['name'])

    def send_goal(self, x, y, yaw, station_name):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending goal to {station_name}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            station = self.stations[self.current_index]
            self.get_logger().error(
                f'Goal to {station["name"].capitalize()} Station was rejected. Mission aborted.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f} m')

    def result_callback(self, future):
        station = self.stations[self.current_index]
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Navigation to {station["name"].capitalize()} Station failed '
                f'(status={status}). Mission aborted.')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Reached {station["name"].capitalize()} Station.')

        wait_time = station.get('wait', 0.0)
        self.current_index += 1

        if wait_time > 0:
            self.get_logger().info(
                f'Waiting {wait_time:.0f}s at {station["name"].capitalize()} Station...')
            self.one_shot(wait_time, self.send_current_goal)
        else:
            self.send_current_goal()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    # Kicks off: Loading (wait 30s) -> Storage -> Shipping -> Home
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
