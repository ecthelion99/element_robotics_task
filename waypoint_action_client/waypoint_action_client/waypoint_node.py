#!/usr/bin/python3

import re

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from tf_transformations import quaternion_from_euler


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('waypoint_action_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

    def get_waypoint(self):
        while True:
            point_input = input("Enter the point coordinate (x, y, z, yaw comma/space seperated) or enter to complete input: ")
            if point_input == "":
                return None
            try:
                point_vals = list(map(float, re.split(r'[,\s]+', point_input)))
                if len(point_vals) != 4:
                    print("Incorrect number of inputs")
                    continue
                break
            except Exception as e:
                print(e)
                print("Invalid input value")
        pose = Pose()
        pose.position = Point(x=point_vals[0], y=point_vals[1], z=point_vals[2])
        quat_arr = quaternion_from_euler(0, 0, point_vals[3])
        pose.orientation = Quaternion(x=quat_arr[0], y=quat_arr[1], z = quat_arr[2], w = quat_arr[3])
        return pose
    
    def get_waypoints(self):
        waypoints = []
        while True:
            waypoint = self.get_waypoint()
            if waypoint is None:
                if len(waypoints) == 0:
                    print("No waypoints entered")
                    continue
                break
            print(f"Adding {waypoint} to waypoints")
            waypoints.append(waypoint)

        print("Sending Goal")
        self.send_goal(waypoints)


        
    def send_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        stamped_poses = []
        for pose in poses:
            stamped_pose = PoseStamped()
            stamped_pose.header.frame_id = "map"
            stamped_pose.header.stamp = self.get_clock().now().to_msg()
            stamped_pose.pose = pose

            stamped_poses.append(stamped_pose)
        
        goal_msg.poses = stamped_poses

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(f"Completed with error code {result.error_code}")
        self.get_waypoints()
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"Waypoints remaning: {feedback.number_of_poses_remaining}, distance remaining: {feedback.distance_remaining}")


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.get_waypoints()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()