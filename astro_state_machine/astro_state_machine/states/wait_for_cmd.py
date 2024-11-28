from yasmin import Blackboard

from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from astro_actions import ACTION_NAMES

from astro_action_interfaces.action import WaitForOpCmd
from astro_action_interfaces.msg import UserCmd,KeyValue,GoToPosData,AppConnData
from strenum  import StrEnum

def cmd_key_value_list_to_dict(cmd_key_value_list:list[KeyValue]):
    cmd_dict = {}
    for cmd in cmd_key_value_list:
        cmd_dict[cmd.key] = cmd.float_value
        if cmd.text_value is not None:
            cmd_dict[cmd.key] = cmd.text_value
    return cmd_dict

class WaitForCmdState(ActionState):
    """
    Attributes:
        None
    """
    class Outcomes(StrEnum):
        SELF_TEST = "self_test"
        MANUAL_CONTROL = "manual_control"
        SHUTDOWN = "shutdown"
        CONNECT = "connect"
        SEARCH_CONNECTOR = "search_connector"
        GO_TO_POSITION = "go_to_position",
        APPROACH_CONNECTOR = "approach_connector"
        WAIT_FOR_OP_CMD = "wait_for_op_cmd"
        

    def __init__(self, outcomes) -> None:
        """
        Initializes the Connect State.


        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            WaitForOpCmd,  # action type
            ACTION_NAMES.WAIT_FOR_OP_CMD,  # action name
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
        cmd_data = cmd_key_value_list_to_dict(response.user_cmd.values_array)
        if response.user_cmd.cmd_type == UserCmd.GO_TO_POSITION:
            go_to_pos_data = GoToPosData(**cmd_data)


            blackboard["next_position"] = {
                "x":go_to_pos_data.x,
                "y": go_to_pos_data.y,
                "z": go_to_pos_data.z,
                "ax": go_to_pos_data.ax,
                "ay": go_to_pos_data.ay,
                "az": go_to_pos_data.az,
            }
            return self.Outcomes.GO_TO_POSITION.value
        if response.user_cmd.cmd_type == UserCmd.APPROACH_CONNECTOR:
            app_conn_data = AppConnData(**cmd_data)
            blackboard["connector_tolerance"] = app_conn_data.tolerance
                    
            if ( "connector_position" not in blackboard):
                print("connector_position not set, please first detect connector")
                blackboard['error'] = "connector_position not set, please first detect connector"
                return self.Outcomes.WAIT_FOR_OP_CMD.value
            
            if ( "connector_tolerance" not in blackboard):
                print("connector_tolerance not in blackboard")
                blackboard['error'] = "connector_tolerance not in blackboard"
                return  self.Outcomes.WAIT_FOR_OP_CMD.value
            
            return self.Outcomes.APPROACH_CONNECTOR.value
        return ABORT

    def print_feedback(
        self, blackboard: Blackboard, feedback: WaitForOpCmd.Feedback
    ) -> None:
        ...