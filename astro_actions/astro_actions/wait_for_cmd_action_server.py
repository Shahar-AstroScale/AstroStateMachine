import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from astro_action_interfaces.action import WaitForOpCmd
from astro_actions import ACTION_NAMES


class AstroWaitCmdActionServer(Node):

    def __init__(self):
        super().__init__(f"{ACTION_NAMES.WAIT_FRO_OP_CMD}_action_server")
        self._action_server = ActionServer(
            self, WaitForOpCmd, ACTION_NAMES.WAIT_FRO_OP_CMD, self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        result = WaitForOpCmd.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    astro_wait_cmd_action_server = AstroWaitCmdActionServer()

    rclpy.spin(astro_wait_cmd_action_server)


if __name__ == "__main__":
    main()
