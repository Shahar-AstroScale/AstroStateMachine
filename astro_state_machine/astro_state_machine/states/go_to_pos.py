from yasmin import Blackboard

from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from astro_actions import ACTION_NAMES

from astro_action_interfaces.action import MoveToPosition
import enum



class GoToPosState(ActionState):
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
            MoveToPosition,  # action type
            ACTION_NAMES.GO_TO_POSITION,  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> MoveToPosition.Goal:

        print(f"blackboard: {blackboard}")
        
        goal = MoveToPosition.Goal()
        goal.ax = blackboard["next_position"]["ax"] 
        goal.ay = blackboard["next_position"]["ay"]
        goal.az = blackboard["next_position"]["az"]
        goal.x = blackboard["next_position"]["x"]
        goal.y = blackboard["next_position"]["y"]
        goal.z = blackboard["next_position"]["z"]

        return goal

    def response_handler(
        self, blackboard: Blackboard, response: MoveToPosition.Result
    ) -> str:
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: MoveToPosition.Feedback
    ) -> None:
        ...