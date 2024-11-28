from yasmin import Blackboard

from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from astro_actions import ACTION_NAMES
from astro_action_interfaces.action import MoveToPosition
import enum


class ApproachConnectorState(ActionState):
    """
    Class representing the state of the Fibonacci action.

    Inherits from ActionState and implements methods to handle the
    Fibonacci action in a finite state machine.

    Attributes:
        None
    """
    # class Outcomes(enum.Enum):
        
    #     # CONNECT = "connect"
    #     WAIT_FOR_OP_CMD = "wait_for_op_cmd"

    def __init__(self) -> None:
        """
        Initializes the Connect State.


        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            MoveToPosition,  # action type
            ACTION_NAMES.APPROACH_CONNECTOR,  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> MoveToPosition.Goal:
        
        print(f"blackboard: {blackboard}")
        goal = MoveToPosition.Goal()
        goal.x = blackboard["connector_position"]["x"]
        goal.y = blackboard["connector_position"]["y"]
        goal.z = blackboard["connector_position"]["z"] - blackboard['connector_tolerance']
        goal.ax = blackboard["connector_position"]["ax"] 
        goal.ay = blackboard["connector_position"]["ay"]
        goal.az = blackboard["connector_position"]["az"]        
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: MoveToPosition.Result
    ) -> str:
        """
        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Connect.Result): The result object from the Connect action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        blackboard["connection_successful"] = (
            response.success
        )  # Store the result sequence in the blackboard
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: MoveToPosition.Feedback
    ) -> None:
        ...