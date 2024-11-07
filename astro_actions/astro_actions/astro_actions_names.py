from dataclasses import dataclass


@dataclass(frozen=True)
class _ACTION_NAMES:
    INIT: str = "init"
    WAIT_FRO_OP_CMD: str = "wait_for_cmd"
    SHUTDOWN: str = "shutdown"
    MANUAL_CONTROL: str = "manual_control"
    SELF_TEST: str = "self_test"
    CONNECTOR_SEARCH: str = "connector_search"
    GO_TO_POS: str = "go_to_pos"
    CONNECT: str = "connect"


ACTION_NAMES = _ACTION_NAMES()
