Please document the packages here:
    1.  robust_navigation: 
        This package contain most of the main code of the moving base except for sensor integration code
        Launch:
            keyboard_control.launch: use the keyboard to control the robot
                Prereq: upload team4_ws/src/robot_base_controller/src/base_motor_controller/base_motor_controller.ino
                        to the arduino, then turn on the motor power.
                        run "roslaunch robust_navigation keyboard_control.launch"
            robot_pose_estimation.launch: use the apriltag detector and odomotry of motor encoder to estimate the pose
                Prereq: upload team4_ws/src/robot_base_controller/src/base_motor_controller/base_motor_controller.ino
                        to the arduino, then turn on the motor power.
                        run "roslaunch robust_navigation robot_pose_estimation.launch"
                Important Published Topic:
                        /estimate_position/pose: the estimated position
                        /estimate_position/angle: the estimated angle
                        /estimate_position/pose_stderr: the estimated position standard derivation
                        /estimate_position/angle_stderr: the estimated angle standard derivation
                        /cmdvel: the topic to control motor velocity
        Src:
            keyboard_control.py: map keyboard input to motor output, it should use motor, keyboard package
            pose_estimation_tw.py: do location estimation of the robot, it should use kinect, apriltag, motor package
            TODO: check out why sometimes it fluctuate a lot
            TODO: keep publishing the location even when the robot have not see any apriltags
            nav_state: moves the robot along given waypoints, using estimated position
            TODO: Complete the test with pose_estimation_tw.py
            
    2.  robot_base_controller:
        This package contains all of the robot base sensor/actuator code
        Launch:
        Src:
            base_motor_controller.py: initialize the motor actuator and encoder in ROS
            TODO: integrate force sensor and create another controller.py
        Scripts:
            base_motor_controller/controller.ino: Aruduino code for base_motor_controller.py, please upload this before running base_motor_controller.py
            TODO: integrate force sensor code

    3.  task_switcher:
        This package contains all the code for using keyboard to switch task (keyboard as sensor)
        Launch:
            task_switcher.launch: use the keyboard as task selector, press <Esc> to escape keyboard detection mode
                Prereq: set up TASK_LIST in the /team4_ws/src/task_switcher/src/task_switcher.py
                Important Published Topic:
                        /current_task: the current task number, 100 before first pressed
        Src:
            task_switcher.py: read keyboard input and publish it

    4.  hand_controller:
        This package contains all of the code for sensors on the hand
        Launch:
            hand_controller.launch: transmit data from imu and buttons to ROS
                Prereq: TODO
                Important Published Topic:
                        /freeze_bool: freeze or not
                        /release_bool: release gripper or not
                        /hand_control: euler angle of the imu [x,y,z]
        Src:
            hand_controller.py: read the IMU and buttons data through UDP
            TODO: write the Prereq, test the code and write the filter for hand_control topic and check semantic of the boolean topics
        Scripts:
            TODO: complete the hand controller code on arduino

    5.  dynamixel_motor:
        This package contains all the required drivers and controllers for the dynamixel motors. It does not need to be             changed!
        
    6.  arm_controller:
        This package controlls the movement of the arm and gripper on the robot base.
        Launch:
            launch_motor.launch: initializes the dynamixels and their controllers. Does not need to be launched separately!
                Prereq: Connect all the dynamixels correctly and update their id in /config/joint1.yaml and /config/joint2.yaml
            arm_controller.launch: launches launch_motor.launch and runs joint_state_publisher.py
                Prereq: Connect all the dynamixels correctly and update their id in /config/joint1.yaml and /config/joint2.yaml
                Important Published Topic:
                        /joint_states: combines the information from the two dynamixels (position, velocity, effort)
        Src:
            joint_state_publisher.py: takes information from both internal dynamixel controllers and combines them into one topic
            move_joint.py: moves the arm
            TODO: complete the arm code and build interface between the different tasks
        

