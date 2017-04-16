# Robotis Mini Control

Includes a RobotisMiniControlInterface in python to interface real or simulated robots (send commands).
It also includes controllers to simulate the robot behavior and the pypot-based node to connect to the real robot.
Contains a walker interface (simulation or real) to command trajectories for walking, and a walker demo python script (provide flag --real for real robot, default is simulated).

 - control.launch -> launches controllers for the robot to simulate in gazebo
 - poppy_joint_state_publisher.launch -> launches the pypot based node to connect to the real robot (real state, send commands)
 - walker_real_robot.launch -> launches the pypot based node and a walker interface in real mode
