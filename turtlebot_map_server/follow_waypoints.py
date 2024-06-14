#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class Waypoints(Node):
    def __init__(self):
        super().__init__("follow_waypoint_node")

        self.timer = self.create_timer(.5, self.nav_callback)

        self.navigator = BasicNavigator()

        self.inspection_route = [
                            [3.84, 5.94],
                            [2.14, 8.81],
                            [3.39, 4.64],
                            [0.05, 2.71],
                            [0.28, -0.30, 0.769]]

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # self.initial_pose.pose.position.x = 2.60
        # self.initial_pose.pose.position.y = 6.30
        self.initial_pose.pose.position.x = 0.28
        self.initial_pose.pose.position.y = -0.30
        self.initial_pose.pose.orientation.z = 1.0
        self.initial_pose.pose.orientation.w = 0.0

        self.inspection_points = []
        self.inspection_pose = PoseStamped()
        self.inspection_pose.header.frame_id = 'map'
        self.inspection_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.inspection_pose.pose.orientation.z = 1.0
        self.inspection_pose.pose.orientation.w = 0.0
        for pt in self.inspection_route:
            self.inspection_pose.pose.position.x = pt[0]
            self.inspection_pose.pose.position.y = pt[1]
            self.inspection_points.append(deepcopy(self.inspection_pose))
        nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(self.inspection_points)

    def nav_callback(self):
        if not self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Inspection of shelves complete! Returning to start...')
            elif result == TaskResult.CANCELED:
                print('Inspection of shelving was canceled. Returning to start...')
                exit(1)
            elif result == TaskResult.FAILED:
                print('Inspection of shelving failed! Returning to start...')


def main():
    rclpy.init()
    waypoint = Waypoints()
    rclpy.spin(waypoint)
    waypoint.destroy_node()
    rclpy.shutdown()    


if __name__ == '__main__':
    main()
