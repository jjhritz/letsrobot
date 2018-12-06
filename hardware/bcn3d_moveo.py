"""@package docstring
Supports BCN3D MOVEO 3D-printed G-code powered robot arm.

The MOVEO is itself essentially a repurposed 3D printer, powered by an Arduino MEGA running Marlin drivers.
To this end, the controller must output a limited set of G-Code commands to manipulate the stepper motors.
This implementation is an extension of the existing serial_board interface.

Reference Marlin G-code documentation at: http://marlinfw.org/meta/gcode/

Module: bcn3d_moveo.py
Author: John J. Hritz <john-j-hritz@sbcglobal.net>
Date: 2018/12/5
"""

# TODO: This is ripe for an OOP implementation. Once we get this first pass working, it should be refactored to that end

import hardware.serial_board as serial_board

module = None

arm_axes = {}
arm_constraints = {}
arm_position = {}

# The string names of the joints driven by stepper motors
steppers = ('base', 'shoulder', 'elbow', 'wrist elevation', 'wrist rotation')
# The string names of the joints driven by servos
servos = ('gripper')


def bind_axes(robot_config):
    """Defines which G-Code axis controls which joint on the arm.

    :param robot_config: The ConfigParser object representation of letsrobot.conf
    :return: None
    """
    global arm_axes
    arm_axes[steppers[0]] = robot_config.get('bcn3d_moveo', 'base')
    arm_axes[steppers[1]] = robot_config.get('bcn3d_moveo', 'shoulder')
    arm_axes[steppers[2]] = robot_config.get('bcn3d_moveo', 'elbow')
    arm_axes[steppers[3]] = robot_config.get('bcn3d_moveo', 'wrist_elevation')
    arm_axes[steppers[4]] = robot_config.get('bcn3d_moveo', 'wrist_rotation')
    arm_axes[servos[0]] = robot_config.get('bcn3d_moveo', 'gripper')


def min_key(joint):
    """Constructs and returns a string in the form "<joint> min"

    :param joint: The name of the joint to be referenced
    :return: The name of the joint with the substring " min" appended
    """

    return joint + ' min'


def max_key(joint):
    """Constructs and returns a string in the form "<joint> max"

    :param joint: The name of the joint to be referenced
    :return: The name of the joint with the substring " max" appended
    """

    return joint + ' max'


def set_constraints(robot_config):
    """Sets the movement constraints on the arm to prevent damage to the arm or its environment.

    Sets the minimum and maximum coordinate positions on each axis so they can be checked whenever the arm is moved.
    :param robot_config: The ConfigParser object representation of letsrobot.conf
    :return: None
    """
    global arm_constraints
    arm_constraints[min_key(steppers[0])] = robot_config.getint('bcn3d_moveo', 'base_min')
    arm_constraints[max_key(steppers[0])] = robot_config.getint('bcn3d_moveo', 'base_max')
    arm_constraints[min_key(steppers[1])] = robot_config.getint('bcn3d_moveo', 'shoulder_min')
    arm_constraints[max_key(steppers[1])] = robot_config.getint('bcn3d_moveo', 'shoulder_max')
    arm_constraints[min_key(steppers[2])] = robot_config.getint('bcn3d_moveo', 'elbow_min')
    arm_constraints[max_key(steppers[2])] = robot_config.getint('bcn3d_moveo', 'elbow_max')
    arm_constraints[min_key(steppers[3])] = robot_config.getint('bcn3d_moveo', 'wrist_e_min')
    arm_constraints[max_key(steppers[3])] = robot_config.getint('bcn3d_moveo', 'wrist_e_max')
    arm_constraints[min_key(steppers[4])] = robot_config.getint('bcn3d_moveo', 'wrist_r_min')
    arm_constraints[max_key(steppers[4])] = robot_config.getint('bcn3d_moveo', 'wrist_r_max')
    arm_constraints[min_key(servos[0])] = robot_config.getint('bcn3d_moveo', 'gripper_min')
    arm_constraints[max_key(servos[0])] = robot_config.getint('bcn3d_moveo', 'gripper_max')


def set_start_position(robot_config):
    """Sets the position for each joint when the robot starts up.

    :param robot_config: The ConfigParser object representation of letsrobot.conf
    :return: None
    """

    global arm_position
    arm_position[steppers[0]] = robot_config.getint('bcn3d_moveo', 'base_s')
    arm_position[steppers[1]] = robot_config.getint('bcn3d_moveo', 'shoulder_s')
    arm_position[steppers[2]] = robot_config.getint('bcn3d_moveo', 'elbow_s')
    arm_position[steppers[3]] = robot_config.getint('bcn3d_moveo', 'wrist_e_s')
    arm_position[steppers[4]] = robot_config.getint('bcn3d_moveo', 'wrist_r_s')
    arm_position[servos[0]] = robot_config.getint('bcn3d_moveo', 'gripper_s')


def check_constraints(joint, pos):
    """Checks if a movement position exceeds a joint's constraints.
    If the position does not exceed the joint's constraints, the position is returned.
    It the position DOES exceed the joint's constraints, the function returns the constraint that was exceeded.

    :param joint: The name of the joint that is being moved.
    :param pos: The position the arm is trying to move to
    :return: The position the arm will be allowed to move to, per the constraints
    """

    global arm_constraints

    # construct strings to match the keys in the constraint dictionary
    min_constr = min_key(joint)
    max_constr = max_key(joint)

    if pos < arm_constraints[min_constr]:
        return arm_constraints[min_constr]

    elif pos > arm_constraints[max_constr]:
        return arm_constraints[max_constr]

    else:
        return pos


def move_arm_to(base_pos=arm_position[steppers[0]],
                shoul_pos=arm_position[steppers[1]],
                elbow_pos=arm_position[steppers[2]],
                elev_pos=arm_position[steppers[3]],
                rot_pos=arm_position[steppers[4]],
                grip_pos=arm_position[servos[0]]):

    """Orders a movement of the arm to an absolute position.
    Checks the passed positions against the joint constraints, constructs the G-Code commands based on those checks,
    and forwards the commands to the serial port.

    :param base_pos: The new coordinate of the base.  Default: the current position.
    :param shoul_pos: The new coordinate of the shoulder.  Default: the current position.
    :param elbow_pos: The new coordinate of the elbow.  Default: the current position.
    :param elev_pos: The new coordinate of the wrist elevation.  Default: the current position.
    :param rot_pos: The new coordinate of the rotation servo.  Default: the current position.
    :param grip_pos: The new position of the gripper servo.  Default: the current position.
    :return: The new arm position.
    """

    global arm_position
    global ser

    arm_position[steppers[0]] = check_constraints(steppers[0], base_pos)
    arm_position[steppers[1]] = check_constraints(steppers[1], shoul_pos)
    arm_position[steppers[2]] = check_constraints(steppers[2], elbow_pos)
    arm_position[steppers[3]] = check_constraints(steppers[3], elev_pos)
    arm_position[steppers[4]] = check_constraints(steppers[4], rot_pos)
    arm_position[servos[0]] = check_constraints(servos[0], grip_pos)

    # Switch the arm to Absolute Positioning using G90
    serial_board.sendSerialCommand(ser, "G90")

    # Construct G-Code string for movement using the G0 Linear Move command.
    # Form: G0 E<pos> X<pos> Y<pos> Z<pos>
    gcode = "G0 "

    for joint in steppers:
        gcode += arm_axes[joint] + arm_position[joint] + " "

    # Clean up trailing whitespace
    gcode = gcode.rstrip(" ")

    serial_board.sendSerialCommand(ser, gcode)

    # Construct G-code string for manipulating the gripper using the M280 Servo Position command
    # Form: M280 P<index> S<pos>
    gcode = "M280 " + arm_axes[servos[0]] + " S" + grip_pos
    serial_board.sendSerialCommand(ser, gcode)

    return arm_position


def move_arm_by(base_inc=0, shoul_inc=0, elbow_inc=0, elev_inc=0, rot_inc=0, grip_inc=0):
    """Orders a movement of the arm to a new position relative to its current one.
    Essentially just a wrapper for move_arm_to that calculates the new position based on the increment.

    :param base_inc: Movement increment of the base.
    :param shoul_inc: Movement increment of the shoulder.
    :param elbow_inc: Movement increment of the elbow.
    :param elev_inc: Movement increment of the wrist elevation.
    :param rot_inc: Movement increment of the wrist rotation.
    :param grip_inc: Movement increment of the gripper.
    :return: The new arm position
    """

    arm_position[steppers[0]] += base_inc
    arm_position[steppers[1]] += shoul_inc
    arm_position[steppers[2]] += elbow_inc
    arm_position[steppers[3]] += elev_inc
    arm_position[steppers[4]] += rot_inc
    arm_position[servos[0]] += grip_inc

    return move_arm_to()


def setup(robot_config):
    """Initialize options for the arm.

    :param robot_config: The ConfigParser object representation of letsrobot.conf
    :return: None
    """
    global module

    # Set up the serial interface
    serial_board.setup(robot_config)

    bind_axes(robot_config)
    set_constraints(robot_config)
    set_start_position(robot_config)


def move(args):
    """Interprets command received from the server and sends G-code to the serial port.
    This function is intended to take simple string commands from the server and convert them into the less
    human-readable G-code the MOVEO expects.

    :param args: The command string received from the server
    :return: None
    """
    # Your custom command interpreter code goes here

    #


if __name__ == '__main__':
    import sys

    try:
        from configparser import ConfigParser
        robot_config = ConfigParser()
    except ImportError:
        print("Missing configparser module (python -m pip install configparser)")
        sys.exit()

    try:
        robot_config.readfp(open('letsrobot.conf'))
    except IOError:
        print(
            "unable to read letsrobot.conf, please check that you have copied letsrobot.sample.conf to letsrobot.conf and modified it appropriately.")
        sys.exit()
    except:
        print("Error in letsrobot.conf:", sys.exc_info()[0])
        sys.exit()

    setup(robot_config)
    print(steppers)
