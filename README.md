# Auto Approach Demo

This package demonstrates autonomous navigation and post-navigation actions for a robot (e.g., Spot) using ROS 2, Yasmin state machines, and Webots Spot services.

## Features

- Navigates to a dynamically computed goal using `nav2`.
- Waits for a configurable period after reaching the goal.
- Commands the robot to sit (lie down) via the `/Spot/lie_down` service.
- Modular state machine using Yasmin.

## Requirements

- [yasmin](https://github.com/uleroboticsgroup/yasmin)
- [Webots simulation](https://github.com/MASKOR/webots_ros2_spot/tree/kinova_gen3)

## Usage

1. **Launch The following in webots kinova gen3 branch**
   ```
   ros2 launch webots_spot spot_launch.py
   ros2 launch bring_up_alert_nav alert_nav_launch.py #octo_navigation/feature/bring_up
   ros2 launch webots_spot dexboard_launch.py
   ```


3. **Run the demo**:

   ```bash
   ros2 run alert_dexterity auto_approach.py
   ```

## File Overview

- `auto_approach.py`: Main script containing the Yasmin state machine and all logic.

## State Machine Flow

1. **NAVIGATING_TO_POSE**: Uses Nav2 to move to the computed goal.
2. **PRINTING_RESULT**: Logs the navigation result.
3. **WAIT_BEFORE_SIT**: Waits for a configurable period (default: 5 seconds).
4. **SITTING_DOWN**: Calls the `/Spot/lie_down` service to make the robot sit.

## TODO

- Create/fix states for dexterity sequence


