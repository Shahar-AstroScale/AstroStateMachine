#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from astro_action_interfaces.action import WaitForOpCmd
from astro_action_interfaces.msg import UserCmd
from astro_actions import ACTION_NAMES



class AstroWaitCmdActionServer(Node):

    def __init__(self):

        super().__init__(f"{ACTION_NAMES.WAIT_FRO_OP_CMD}_action_server")
        self.subscription = self.create_subscription(
            UserCmd,
            '/user_commands',
            self.listener_callback,
            10)

        self._action_server = ActionServer(
            self, WaitForOpCmd, ACTION_NAMES.WAIT_FRO_OP_CMD, self.execute_callback
        )
        self.is_waiting_for_cmd = False
        self._incoming_cmd = None

    def execute_callback(self, goal_handle):
        self.is_waiting_for_cmd = True
        self.get_logger().info("Waitin for cmd...")
        while self._incoming_cmd is None:
            time.sleep(0.4)
            
        result = WaitForOpCmd.Result()
        print("result.user_cmd: ", self._incoming_cmd)
        print("result.user_cmd type : ", type(self._incoming_cmd))

        result.user_cmd = self._incoming_cmd
        goal_handle.succeed()
        return result

    def listener_callback(self, msg):
        if self.is_waiting_for_cmd:
            self.get_logger().info("Received command: %s" % msg)
            self._incoming_cmd = msg
            self.is_waiting_for_cmd = False


def main(args=None):


    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=5)

    astro_wait_cmd_action_server = AstroWaitCmdActionServer()
    executor.add_node(astro_wait_cmd_action_server)
    executor.spin()
    


if __name__ == "__main__":
    main()
