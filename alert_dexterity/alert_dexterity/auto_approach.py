#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

import yasmin
from states.approach import Nav2Dex, SitDown, print_result, wait_period
from states.move_to_pose import MoveToPoseState
from states.align import AlignToMapOrientation
from states.save_poses import SavePosesState
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

        # 1. Navigate to the dexterity pose
        self.sm.add_state(
            "NAVIGATING_TO_POSE",
            Nav2Dex(),
            transitions={
                SUCCEED: "PRINTING_RESULT",
                CANCEL: CANCEL,
                ABORT: ABORT,
            },
        )
        # 2. Print the result of the navigation
        self.sm.add_state(
            "PRINTING_RESULT",
            CbState([SUCCEED], print_result),
            transitions={
                SUCCEED: "WAIT_BEFORE_ALIGN",
            },
        )
        # 3. Wait before aligning to map orientation
        self.sm.add_state(
            "WAIT_BEFORE_ALIGN",
            CbState([SUCCEED], wait_period),
            transitions={
                SUCCEED: "ALIGN_TO_MAP_ORIENTATION",
            },
        )
        # 4. Align to map orientation
        self.sm.add_state(
            "ALIGN_TO_MAP_ORIENTATION",
            AlignToMapOrientation(),
            transitions={
                SUCCEED: "WAIT_BEFORE_SIT",
                CANCEL: CANCEL,
                ABORT: ABORT,
            },
        )
        # 5. Wait before sitting down
        self.sm.add_state(
            "WAIT_BEFORE_SIT",
            CbState([SUCCEED], wait_period),
            transitions={
                SUCCEED: "SITTING_DOWN",
            },
        )
        # 6. Sit down
        self.sm.add_state(
            "SITTING_DOWN",
            SitDown(),
            transitions={
                SUCCEED: "WAITING_FOR_SEQUENCE",
                ABORT: ABORT,
            },
        )
        # 7. Wait for a sequence before moving to the linear front
        self.sm.add_state(
            "WAITING_FOR_SEQUENCE",
            CbState([SUCCEED], wait_period ),
            transitions={
                SUCCEED: "SAVE_POSES",
            },
        )
        # 8. Save all required poses
        self.sm.add_state(
            "SAVE_POSES",
            SavePosesState(),
            transitions={
                SUCCEED: "MOVE_TO_LINEAR_FRONT",
                CANCEL: CANCEL,
                ABORT: ABORT,
            },
        )
        # 9. Move to the linear front pose
        self.sm.add_state(
            "MOVE_TO_LINEAR_FRONT",
            MoveToPoseState(),
            transitions={
                SUCCEED: SUCCEED,
                CANCEL: CANCEL,
                ABORT: ABORT,
            },
        )

        YasminViewerPub("AUTO_DEX_FSM", self.sm)
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
