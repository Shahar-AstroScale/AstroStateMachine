#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from astro_action_interfaces.action import WaitForOpCmd
from astro_actions import ACTION_NAMES


class AstroInitActionServer(Node):

    def __init__(self):
        super().__init__(f"{ACTION_NAMES.INIT}_action_server")
        self._action_server = ActionServer(
            self, WaitForOpCmd, ACTION_NAMES.INIT, self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        result = WaitForOpCmd.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    astro_init_action_server = AstroInitActionServer()

    rclpy.spin(astro_init_action_server)


if __name__ == "__main__":
    main()

