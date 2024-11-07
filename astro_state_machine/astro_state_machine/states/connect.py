from yasmin import Blackboard

from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from astro_actions import ACTION_NAMES
from astro_action_interfaces.action import Connect
class ConnectState(ActionState):
    """
    Class representing the state of the Fibonacci action.

    Inherits from ActionState and implements methods to handle the
    Fibonacci action in a finite state machine.

    Attributes:
        None
    """

    def __init__(self) -> None:
        """
        Initializes the Connect State.


        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            Connect,  # action type
            ACTION_NAMES.CONNECT,  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Connect.Goal:
        
        goal = Connect.Goal()
        return goal

    def response_handler(
        self, blackboard: Blackboard, response: Connect.Result
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
        self, blackboard: Blackboard, feedback: Connect.Feedback
    ) -> None:
        """
        Prints feedback from the Fibonacci action.

        This method logs the partial sequence received during the action.

        Parameters:
            blackboard (Blackboard): The blackboard (not used in this method).
            feedback (Fibonacci.Feedback): The feedback object from the Fibonacci action.

        Returns:
            None

        Raises:
            None
        """
