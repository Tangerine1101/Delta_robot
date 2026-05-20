# workflow
## threading
thread 1: PLC <-> PC
thread 2: CLI and scheduler
thread 3: Image processing

### thread 1
communication between pc and plc via ethernet
commandID
package:
1. pc package

{
    "commandID": 0,
    "argument_number": 0,
    "argument_x": [0.0] * argument_number,
    "argument_y": [0.0] * argument_number,
    "argument_z": [0.0] * argument_number,
    "argument_e": [0] * argument_number, # end effector state along trajectory: 0 open, 1 pick
    "argument_time": [0.0] * argument_number,
    "doing_bit": 1,
}
2. plc package

{
    "pos_angular": [theta1, theta2, theta3]
    "pos_EE": [x, y, z] # catersian coordinate of end effector
    "task_doing": 0,
    "task_state": 0
}
3. commandID (also task_doing and task_state)
COMMAND_ID = {
    "stop": 0,
    "goto_relative": 1,
    "goto_absolute": 2,
    "go_trajectory": 3,
    "calibrate": 4,
    "pick": 5,
    "release": 6,
}

### thread 2
CLI mode or auto mode aka scheduler(will be developed later)

CLI mode - create a package:
- stop: stop all motor
- go <theta1> <theta2> <theta3> : turn motors a theta degreepackage
- goto <x> <y> <z> : move end effector
- go_trajectory <defined_trajectory> : hardcode define some example trajectories for testing purpose
- calib : send calibrate command
- pick/release : send pick/release command to control the end effector

since the struct is defined in sysmac studio, so even if an member of array or argument is unused, it must be send (just send as 0) 

# trajectory

optimize speed by reduce movement into 4 points and 2 phases: goto and pick phase where goto is when the robot move from current position to the conveyor and pick phase is when robot pick the object and move it to above of sort zone and release the end effector - a suction cup
C_pick/B_goto->   --------------------  D_pick/A_goto
                 /
                /
        B_pick->| <- C_goto
                |
                | <- D_goto (a bit higher than A_pick so when the robot press down, the cup will suck and pick up object perpectly)
        A_pick->|
