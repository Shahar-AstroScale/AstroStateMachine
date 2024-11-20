import rclpy
import yasmin
from astro_state_machine.states import *
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL, TIMEOUT
from yasmin_viewer import YasminViewerPub





def main():
    """
    Main function to execute the ROS 2 action client demo.

    This function initializes the ROS 2 client, sets up the finite state
    machine, adds the states, and starts the action processing.

    Parameters:
        None

    Returns:
        None

    Raises:
        KeyboardInterrupt: If the user interrupts the execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(
        outcomes=["finish"]
    )

    # Add states to the FSM
    sm.add_state(
        "INIT",
        InitState(),
        transitions={
            SUCCEED: "WAIT_FOR_OP_CMD",
            ABORT: "WAIT_FOR_OP_CMD",
            CANCEL: "WAIT_FOR_OP_CMD",
        },
    )

    sm.add_state(
        "WAIT_FOR_OP_CMD",
        WaitForCmdState(
            outcomes=[WaitForCmdState.Outcomes.GO_TO_POSITION.value , WaitForCmdState.Outcomes.SHUTDOWN.value, 
                      WaitForCmdState.Outcomes.CONNECT.value, WaitForCmdState.Outcomes.SEARCH_CONNECTOR.value, 
                      WaitForCmdState.Outcomes.MANUAL_CONTROL.value, WaitForCmdState.Outcomes.SELF_TEST.value] ,
        ),
        transitions={
            SUCCEED: "finish",
            WaitForCmdState.Outcomes.SELF_TEST.value: "SELF_TEST",
            WaitForCmdState.Outcomes.SHUTDOWN .value: "SHUTDOWN",
            WaitForCmdState.Outcomes.CONNECT .value: "CONNECT",
            WaitForCmdState.Outcomes.SEARCH_CONNECTOR.value : "SEARCH_CONNECTOR",
            WaitForCmdState.Outcomes.MANUAL_CONTROL.value : "MANUAL_CONTROL",
            WaitForCmdState.Outcomes.GO_TO_POSITION.value: "GO_TO_POSITION",
            ABORT:"WAIT_FOR_OP_CMD",
            CANCEL: "finish",
        },
    )

    sm.add_state(
        "CONNECT",
        ConnectState(),
        transitions={
            SUCCEED: "WAIT_FOR_OP_CMD",
            ABORT:"WAIT_FOR_OP_CMD",
            CANCEL: "WAIT_FOR_OP_CMD",
        },
    )

    sm.add_state(
        "SELF_TEST",
        SelfTestState(),
        transitions={
            SUCCEED: "WAIT_FOR_OP_CMD",
            CANCEL: "WAIT_FOR_OP_CMD",
            ABORT: "WAIT_FOR_OP_CMD",
        },
    )

    sm.add_state(
        "GO_TO_POSITION",
        GoToPosState(),
        transitions={
            SUCCEED: "WAIT_FOR_OP_CMD",
            CANCEL: "WAIT_FOR_OP_CMD",
            ABORT: "WAIT_FOR_OP_CMD",
        },
    )

    sm.add_state(
        "SHUTDOWN",
        ShutdownState(),
        transitions={
            SUCCEED: "WAIT_FOR_OP_CMD",
            CANCEL: "WAIT_FOR_OP_CMD",
            ABORT: "WAIT_FOR_OP_CMD",
        },
    )
    
    sm.add_state(
        "SEARCH_CONNECTOR",
        SearchConnectorState(),
        transitions={
            SUCCEED: "WAIT_FOR_OP_CMD",
            CANCEL: "WAIT_FOR_OP_CMD",
            ABORT: "WAIT_FOR_OP_CMD",
        },
    )


    sm.add_state(
        "MANUAL_CONTROL",
        ManualControlState(),
        transitions={
            SUCCEED: "WAIT_FOR_OP_CMD",
            CANCEL: "WAIT_FOR_OP_CMD",
            ABORT: "WAIT_FOR_OP_CMD",
        },
    )

    # Publish FSM information

    # Create an initial blackboard with the input value
    blackboard = Blackboard()

    # Execute the FSM
    try:
        sm.set_start_state("WAIT_FOR_OP_CMD")
        YasminViewerPub("ASTRO_STATE_MACHINE", sm)

        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
