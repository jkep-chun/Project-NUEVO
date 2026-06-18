# Project NUEVO
![](/assets/NUEVO.png)

Lab project material for the MAE 162 robotics course (Winter/Spring 2026).

## Overview
Our finished autonomous rover is designed to navigate a predefined maze-like map populated with arbitrary obstacles, traversing ~16 meters in three minutes or less while picking up and assembling mock burger ingredients, identifying the target customer, and delivering the mock burger. Its main features are: differential drive, a 2DOF manipulator to pick and place mock burger ingredients, fusion of AMCL pose estimates and odometry for localization, computer vision (CV) for traffic light detection and customer identification, and path planning with the novel LAPF algorithm and path smoothing, all designed with modularity and scale in mind, being easily configurable for additional burger ingredients in any order, changes in venue layout, and reordering of tasks.

## Problem
The fast-food industry is a promising frontier for autonomy. While cashiers have in many instances been replaced by kiosks, cooks and servers have not. While appliances and food processing technologies have cut many steps in food preparation from scratch, humans are still responsible for assembling and serving food. Our problem was therefore identifying all aspects of this process (represented by assembling and delivering a burger to a customer) and automating them.

### Localization
Localization proved to be the most difficult challenge. Though Project NUEVO's software stack provides pose fusion with IMU and mock GPS, environmental factors render the IMU's heading and 'GPS' position estimates unreliable. The presence of strong magnetic fields in the venue interfere with the IMU's magnetometer, removing ability to track absolute heading. The 'GPS' is also subject to error. Being a mock GPS, estimating position using CV on ArUco markers at fixed points of the world frame, the position measurements were subject to zero shift and sensitivity error when the world tags are dislodged or not placed accurately. Furthermore, being reliant on a camera fixed about the course, if the robot is outside the camera's FOV, the 'GPS' cannot measure its position.

Thus, given our purposes, the software stack of Project NUEVO gives us only pure odometry for localization. However, having glued additional rubber to our tires for increased traction, our wheel diameters became lopsided, impeding our ability to drive straight simply using odometry for feedback. Another problem is accounting for the extra ~25mm traveled (as computed by odometry) over the ramp without an IMU. Since manually tuning waypoints is simply poor practice and lacks robustness, we turned to Navigation 2's Adaptive Monte Carlo Localization (AMCL). Fusing its pose estimate with the odometry pose estimate corrected both the heading drift from mismatched wheels and the position discrepancy brought by the ramp.

### Ingredient Manipulation
Burger assembly requires stacking three or more ingredients of more or less circular shape with varying diameters. Our hardware team members therefore designed a claw to encapsulate the various ingredients with rubber bands.

### Customer Identification
The recommended method of customer identification is ascertaining the target customer's gender since there are only two opposite-gendered customers. Many groups had difficulty with this approach, and our group, being less savvy with neural networks and models sought another way. We noted that in fast food environments, identifying a customer simply by gender would never work since there would presumably be multiple customers of the same gender at a given point in time. Therefore we chose to compare an image of the target customer to images of current customers, creating a match simply based on features extracted by the Oriented FAST and Rotated BRIEF (ORB) algorithm and compared with cv2's BFMatcher.

## Design And Approach

### Hardware
Our team aimed for hardware simplicity, minimizing the actuators and electro-mechanical components under the presupposition that simplified hardware would entail simplified software. To this end, our final design required only one stepper motor for vertical actuation and one servo for ingredient handling.

### Software
#### Hierarchical FSM
Given our intent to transition between sequential, task-based states, we built a hierarchical FSM. The main mission tasks are carried out sequentially in the EXECUTE state. Possible task states are tabled below.

| State | Description |
| ----- | - |
| WAIT  | Turns in place until detects traffic light; waits for green light |
| NAV   | Takes vertices and generates a smooth path; navigates with pure pursuit or LAPF |
| MANIP | Runs a pick or place sequence, configured according to the ingredient(s) at hand |
| IDENT | Performs image processing to find a customer match |
| PAUSE | Stops the robot for 2.0 seconds (i.e. at the stop sign) |

A full depiction of the state machine is given below.
![](/assets/fsm.png)

#### ROS2
Since Project NUEVO's software stack is self-contained, many ROS2 topics are non-standard. Therefore, to utilize `/nav2_amcl` for localization and `/foxglove_bridge` for teleoperation, custom 'bridge' nodes (`/navigation_bridge` and `/slam_bridge`) act as translators between native Project NUEVO topics and standardized topics. A simplified diagram of node interaction is shown below.
![](/assets/nodes.png)

## Results
The results of AMCL heading fusion can be seen below. Granted, the comparison is exaggerated since the robot was 'lazily' placed at the starting location. Nevertheless, the correction imposed by AMCL makes this all the more impressive.
![](/assets/full_trajectory_amcl_fusion.jpg)

Other brief results are as follows:
- An unintentional byproduct of our gripper design was surprising robustness when misaligned with burger ingredients.
- Furthermore, our choice to match the target customer's image to stored customer profiles rather than simply identify gender yielded a 100% success rate in customer identification. 

## How To Reproduce
We hope future engineering students can further our work. In particular, we hope AMCL, the hierarchical task-based FSM, and linearization about the drive motors' deadband become fixtures in Project NUEVO (none of which require additional hardware).

### Hardware
Building upon the base hardware provided in MAE 162D/E, we added motors for the manipulator: a stepper motor (Nema 17) for vertical actuation and a servo (MG996R) for gripper actuation as well as limit switches for homing and detection of proximity to walls.

### Software
Should another student or team wish to run this work, we recommended following these steps.
```
git clone https://github.com/jkep-chun/Project-NUEVO
cd Project-NUEVO
```
To launch the ROS2 nodes, build and enter the docker container (consult [`manual.md`](docs/manual/MANUAL.md)), then use the following commands in separate terminals:
```
ros2 launch rplidar_c1.launch.py
ros2 launch vision vision_production.launch.py
ros2 launch robot localization.launch.py
ros2 run robot robot
```
To tune the robot's behavior for a different environment or hardware configuration, create new task types, or reorder the task sequence, see these files found [`here`](ros2_ws/src/robot/robot/).
```
ros2_ws/src/robot/robot/
├── hardware_map.py          Physical parameters, e.g. wheel diameter, LiDAR offset, etc.
└── fsm_helpers/
    ├── firmware_helpers.py  Auxiliary methods, e.g. approach_ingredient_table()
    ├── task_planner.py      Mission sequence; can be configured for isolated tests
    ├── task.py              Mission task definitions
    ├── course_parameters.py Venue/map specific geometry, i.e. waypoints
    ├── ingredient_helpers.py Burger ingredient geometry
    └── burgerbot_parameters.py Rover tuning parameters
```

## Repository Structure
```
├── firmware/       Arduino firmware and firmware-specific docs
├── nuevo_ui/       Raspberry Pi bridge + web UI
├── ros2_ws/        ROS2 workspace containing most of this group's unique code
├── tlv_protocol/   TLV type definitions, payload schemas, generators
├── NUEVO board/    PCB design files (schematics, layouts, BOM)
├── mechanical/     CAD files for chassis and manipulators
├── docs/           Cross-project architecture, protocol, and design docs
└── assets/         Shared repo assets
```

## Technologies
- **Embedded**: Arduino (C/C++)
- **High-Level**: ROS2 (Python/C++), Raspberry Pi 5
- **Communication**: UART serial protocol
- **Sensors**: Camera, LiDAR, encoders, limit switches
- **Hardware**: Custom PCB, stepper/servo/DC motors
