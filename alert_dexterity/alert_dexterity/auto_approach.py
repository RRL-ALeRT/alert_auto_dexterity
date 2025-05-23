#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

import yasmin
from states.approach import Nav2Dex, SitDown, print_result, wait_period
from states.move_to_pose import MoveToPoseState
from yasmin.blackboard import Blackboard
from yasmin import StateMachine, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from yasmin_viewer import YasminViewerPub
from yasmin_ros import set_ros_loggers

class AutoApproachNode(Node):
    def __init__(self):
        super().__init__("auto_approach_node")
        yasmin.YASMIN_LOG_INFO("Starting the action client node...")
        set_ros_loggers()
        self.sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])

        # self.sm.add_state(
        #     "NAVIGATING_TO_POSE",
        #     Nav2Dex(),
        #     transitions={
        #         SUCCEED: "PRINTING_RESULT",
        #         CANCEL: "CANCELLED",
        #         ABORT: ABORT,
        #     },
        # )
        # self.sm.add_state(
        #     "PRINTING_RESULT",
        #     CbState([SUCCEED], print_result),
        #     transitions={
        #         SUCCEED: "WAIT_BEFORE_SIT",
        #     },
        # )
        # self.sm.add_state(
        #     "WAIT_BEFORE_SIT",
        #     CbState([SUCCEED], wait_period),
        #     transitions={
        #         SUCCEED: "SITTING_DOWN",
        #     },
        # )
        # self.sm.add_state(
        #     "SITTING_DOWN",
        #     SitDown(),
        #     transitions={
        #         SUCCEED: "SUCCEED",
        #         ABORT: ABORT,
        #     },
        # )
        self.sm.add_state(
            "WAITING_FOR_SEQUENCE",
            CbState([SUCCEED], wait_period ),
            transitions={
                SUCCEED: "MOVE_TO_POSE",
            },
        )
        self.sm.add_state(
            "MOVE_TO_POSE",
            MoveToPoseState(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT},
        )

        YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", self.sm)
        self.blackboard = Blackboard()

        # Start the FSM as a timer callback so the node is fully initialized
        self.create_timer(0.1, self.run_fsm, callback_group=None)
        self.fsm_started = False

    def run_fsm(self):
        if self.fsm_started:
            return
        self.fsm_started = True
        try:
            outcome = self.sm(self.blackboard)
            yasmin.YASMIN_LOG_INFO(f"FSM outcome: {outcome}")
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"FSM execution failed: {e}")
            if self.sm.is_running():
                self.sm.cancel_state()
        finally:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AutoApproachNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
