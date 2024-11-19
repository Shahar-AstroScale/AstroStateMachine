from yasmin import Blackboard

from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from astro_actions import ACTION_NAMES

from astro_action_interfaces.action import WaitForOpCmd
from astro_action_interfaces.msg import UserCmd
import enum 

class WaitForCmdState(ActionState):
    """
    Class representing the state of the Fibonacci action.

    Inherits from ActionState and implements methods to handle the
    Fibonacci action in a finite state machine.

    Attributes:
        None
    """
    class Outcomes(enum.Enum):
        SUCCEED = "succeed"
        ABORT = "abort"
        CANCEL = "cancel"
        SELF_TEST = "self_test"
        MANUAL_CONTROL = "manual_control"
        SHUTDOWN = "shutdown"
        CONNECT = "connect"
        SEARCH_CONNECTOR = "search_connector"
        GO_TO_POSITION = "go_to_position"

    def __init__(self) -> None:
        """
        Initializes the Connect State.


        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            WaitForOpCmd,  # action type
            ACTION_NAMES.WAIT_FRO_OP_CMD,  # action name
            self.create_goal_handler,  # callback to create the goal
            [outcome.value for outcome in self.Outcomes],  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> WaitForOpCmd.Goal:
        
        goal = WaitForOpCmd.Goal()
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: WaitForOpCmd.Result
    ) -> str:
        if response.user_cmd.cmd_type == UserCmd.GO_TO_POSITION:
            blackboard["next_position"] = {
                "x": response.user_cmd.x,
                "y": response.user_cmd.y,
                "z": response.user_cmd.z,
                "ax": response.user_cmd.ax,
                "ay": response.user_cmd.ay,
                "az": response.user_cmd.az,
            }
            return self.Outcomes.GO_TO_POSITION.value
        return ABORT

    def print_feedback(
        self, blackboard: Blackboard, feedback: WaitForOpCmd.Feedback
    ) -> None:
        ...