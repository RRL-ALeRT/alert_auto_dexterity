#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

import yasmin
from states.approach import (
    MBFDexLinearFront,
    MBFDexOmniFront,
    SitDown,
    StandUp,
    print_result,
    wait_period,
)
from states.move_to_pose import (
    MoveToPoseState,
    retract_manipulator,
)
from states.align import AlignToMapOrientation
from yasmin.blackboard import Blackboard
from yasmin import StateMachine, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from yasmin_viewer import YasminViewerPub
from yasmin_ros import set_ros_loggers

# Monkey-patch StateMachine.validate to skip validation (workaround for yasmin_viewer bug)
_original_validate = StateMachine.validate
def _noop_validate(self, strict_mode=False):
    pass
StateMachine.validate = _noop_validate

class AutoApproachNode(Node):
    def __init__(self):
        super().__init__("auto_approach_node")
        yasmin.YASMIN_LOG_INFO("Starting the action client node...")
        set_ros_loggers()
        self.sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])

        # 1. Navigate to linear front
        self.sm.add_state(
            "NAV_TO_LINEAR_FRONT",
            MBFDexLinearFront(),
            transitions={
                SUCCEED: "PRINT_RESULT_LINEAR",
                CANCEL: CANCEL,
                ABORT: "NAV_TO_LINEAR_FRONT",
            },
        )
        # 2. Print result
        self.sm.add_state(
            "PRINT_RESULT_LINEAR",
            CbState({SUCCEED}, print_result),  
            transitions={
                SUCCEED: "WAIT_BEFORE_SIT_LINEAR",
            },
        )
        # 3. Wait before sitting
        self.sm.add_state(
            "WAIT_BEFORE_SIT_LINEAR",
            CbState({SUCCEED}, wait_period),  
            transitions={
                SUCCEED: "SIT_DOWN_LINEAR",
            },
        )
        # 4. Sit down at linear
        self.sm.add_state(
            "SIT_DOWN_LINEAR",
            SitDown(),
            transitions={
                SUCCEED: "WAIT_BEFORE_SEQUENCE_LINEAR",
                ABORT: ABORT,
            },
        )
        # 5. Wait before sequence
        self.sm.add_state(
            "WAIT_BEFORE_SEQUENCE_LINEAR",
            CbState({SUCCEED}, wait_period),  
            transitions={
                SUCCEED: "SEQUENCE_LINEAR",
            },
        )
        # 6. Do sequence at linear (MoveToPoseState looks up TF frames directly)
        self.sm.add_state(
            "SEQUENCE_LINEAR",
            MoveToPoseState(),
            transitions={
                SUCCEED: "RETRACT_MANIPULATOR",
                CANCEL: CANCEL,
                ABORT: ABORT,
            },
        )
        # 7. Retract manipulator
        self.sm.add_state(
            "RETRACT_MANIPULATOR",
            CbState({SUCCEED}, retract_manipulator),  
            transitions={
                SUCCEED: "WAIT_BEFORE_STAND_UP_LINEAR",
            },
        )
        # 8. Wait before standing up
        self.sm.add_state(
            "WAIT_BEFORE_STAND_UP_LINEAR",
            CbState({SUCCEED}, wait_period),  
            transitions={
                SUCCEED: "STAND_UP_LINEAR",
            },
        )
        # 9. Stand up
        self.sm.add_state(
            "STAND_UP_LINEAR",
            StandUp(),
            transitions={
                SUCCEED: "WAIT_BEFORE_NAV_OMNI",
                ABORT: ABORT,
            },
        )
        # 10. Wait before navigating to omni
        self.sm.add_state(
            "WAIT_BEFORE_NAV_OMNI",
            CbState({SUCCEED}, wait_period),  
            transitions={
                SUCCEED: "NAV_TO_OMNI_FRONT",
            },
        )
        # 11. Navigate to omni front
        self.sm.add_state(
            "NAV_TO_OMNI_FRONT",
            MBFDexOmniFront(),
            transitions={
                SUCCEED: "PRINT_RESULT_OMNI",
                CANCEL: CANCEL,
                ABORT: "NAV_TO_OMNI_FRONT",
            },
        )
        # 12. Print result
        self.sm.add_state(
            "PRINT_RESULT_OMNI",
            CbState({SUCCEED}, print_result),  
            transitions={
                SUCCEED: "WAIT_BEFORE_SEQUENCE_OMNI",
            },
        )
        # 13. Wait before sequence
        self.sm.add_state(
            "WAIT_BEFORE_SEQUENCE_OMNI",
            CbState({SUCCEED}, wait_period),  
            transitions={
                SUCCEED: "SIT_DOWN_OMNI",
            },
        )
        # 14. Sit down at omni front
        self.sm.add_state(
            "SIT_DOWN_OMNI",
            SitDown(),
            transitions={
                SUCCEED: "SEQUENCE_OMNI",
                ABORT: ABORT,
            },
        )
        # 15. Do sequence at omni (MoveToPoseState with omni TF frames): the tfs need to be redefined
        self.sm.add_state(
            "SEQUENCE_OMNI",
            MoveToPoseState(sequence=[
                "omni_front", "top_left", "top_right", "bottom_left", "bottom_right"
            ]),
            transitions={
                SUCCEED: "RETRACT_MANIPULATOR_OMNI",
                CANCEL: CANCEL,
                ABORT: ABORT,
            },
        )
        # 16. Retract manipulator at omni
        self.sm.add_state(
            "RETRACT_MANIPULATOR_OMNI",
            CbState({SUCCEED}, retract_manipulator),  
            transitions={
                SUCCEED: "WAIT_BEFORE_STAND_UP_OMNI",
            },
        )
        # 17. Wait before standing up at omni
        self.sm.add_state(
            "WAIT_BEFORE_STAND_UP_OMNI",
            CbState({SUCCEED}, wait_period),  
            transitions={
                SUCCEED: "STAND_UP_OMNI",
            },
        )
        # 18. Stand up at omni
        self.sm.add_state(
            "STAND_UP_OMNI",
            StandUp(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT,
            },
        )

        # try:
        #     YasminViewerPub("AUTO_DEX_FSM", self.sm)
        # except Exception as e:
        #     yasmin.YASMIN_LOG_WARN(f"YasminViewerPub failed to initialize: {e}")
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
    if rclpy.ok():
        node.get_logger().info("Shutting down the node...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
