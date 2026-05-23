# update 23/5
- Increase from 3 -> 4 DOF. To avoid recalculate IK and recode on PLC, the 4th dof will act as a second end effector which rotate the suction cup via a stepper.
- new hardware member: a siemen S7 1200 plc
Omron NX plc: control 3 main motors via etherCAT + end effector. can't handle high frequency output/input
Siemen S7 1200: handle high frequency input/output. control 2 stepper in conveyor and 4th dof motor, read encoder feedback on conveyor for real speed calculate.
- 3 new command: rotate - rotate the 4th dof, change_speed - change conveyor's speed.

# workflow
## threading
thread 1: PLCs <-> PC
thread 2: CLI and scheduler
thread 3: Image processing
(*new) thread 4: GUI -> web interface + database later

### thread 1
communication between pc and plc via ethernet
commandID
package:
1. pc package
 to omron plc package:
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
 to siemen plc package:
 {
    "CommandID": 0,
    "rotate": 0.0,
    "speed": 0.0
 }
2. plc package
 from omron plc:
{
    "pos_angular": [theta1, theta2, theta3]
    "pos_EE": [x, y, z] # catersian coordinate of end effector
    "task_doing": 0,
    "task_state": 0
}
 from siemen plc:
 {
    "rotate_current": 0.0,
    "speed_current": 0.0,
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
    "rotate_absolute": 7,
    "change_speed": 8,
    "plan_siemen": 9
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

# physical workspace
description from doc/workspace.png

Conveyor Belt: A horizontal conveyor belt. Products move from along the conveyor.

Camera Zone: A rectangular detection area located on the upstream.

Delta Robot Work Zone: A circular workspace located downstream, overlapping the conveyor and separated with the camera zone.

Bins: Two destination squares located near the conveyor, completely inside the robot's circular work zone.

# work flow logic
image processing thread detect products and create and maintain python objects when with members: objectID, current location, direction/angle.
scheduler thread make plan to pick each object: freeze the conveyor's speed, move the end effector to predicted spot
communication thread translate above pick plans into packages: one package for Omron plc and one for Siemen plc 
(*new, didn't implement) interface thread: create a intuitive GUI to track the robot working process. will be implement into a web-base interface + database later.

main workflow: detect n objects (n is global variable = objects in camera zone + objects in working zone) -> freeze conveyor speed -> move to object -> pick + rotate object and move to bins -> wait for rotating then release -> update n = n -1 -> update conveyor speed -> loop