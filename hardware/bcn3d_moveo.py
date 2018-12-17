"""@package docstring
Supports BCN3D MOVEO 3D-printed G-code powered robot arm.

The MOVEO is itself essentially a repurposed 3D printer, powered by an Arduino MEGA running Marlin drivers.
To this end, the controller must output a limited set of G-Code commands to manipulate the stepper motors.
This implementation is an extension of the existing serial_board interface.

Commands on the LetsRobot.tv site must be formatted as joint,move_type,coordinate
Where:
joint: is the string name of a stepper motor or servo
move_type: is a string, either "inc" or "abs" (for "incremental" or "absolute" movement)
coordinate: is an integer, and the G-code coordinate the joint will move to.

You can command multiple joints at once by separating command groups with semicolons, like this:
base,inc,5;shoulder,abs,94;gripper,abs,10

Note that, all moves in a single command are made at once.  As such, for each joint, all incremental movements will be
summed before the the final G-code command is generated and only the last absolute move will be used.
For your own sanity, only command each joint once in a long command.

Remember to set the number of extruders you're using in the Marlin firmware's Configuration.h file.

Reference Marlin G-code documentation at: http://marlinfw.org/meta/gcode/

Special thanks to Nocturnal, who helped figure out how to manipulate multiple extruder axes.

Module: bcn3d_moveo.py
Author: John J. Hritz <john-j-hritz@sbcglobal.net>
Date: 2018/12/5
"""

# TODO: This is ripe for an OOP implementation. Once we get this first pass working, it should be refactored to that end

import hardware.serial_board as serial_board
import logging

if __name__ == '__main__':
    import sys

    try:
        from configparser import ConfigParser
        robot_config = ConfigParser()
    except ImportError:
        print("Missing configparser module (python -m pip install configparser)")
        sys.exit()

log = logging.getLogger('hardware/bcn3d_moveo')

# The string names of the joints driven by stepper motors
steppers = ('base', 'shoulder', 'elbow', 'wrist elevation', 'wrist rotation')
# The string names of the joints driven by servos
servos = tuple(['gripper'])

multiple_extruders = False
current_extruder = 0

arm_axes = {}
steps_per_unit = {}
arm_constraints = {}
arm_position = {steppers[0]: 0, steppers[1]: 0, steppers[2]: 0, steppers[3]: 0, steppers[4]: 0, servos[0]: 0}


def bind_axes(robot_config: ConfigParser):
    """Defines which G-Code axis controls which joint on the arm.

    :param robot_config: The ConfigParser object representation of letsrobot.conf
    :return: None
    """
    global arm_axes
    arm_axes[steppers[0]] = robot_config.get('bcn3d_moveo', 'base')
    log.debug(steppers[0],)
    arm_axes[steppers[1]] = robot_config.get('bcn3d_moveo', 'shoulder')
    arm_axes[steppers[2]] = robot_config.get('bcn3d_moveo', 'elbow')
    arm_axes[steppers[3]] = robot_config.get('bcn3d_moveo', 'wrist_elevation')
    arm_axes[steppers[4]] = robot_config.get('bcn3d_moveo', 'wrist_rotation')
    arm_axes[servos[0]] = robot_config.get('bcn3d_moveo', 'gripper')


def set_steps_per_unit(robot_config: ConfigParser):
    """Sets the steps per unit in the Marlin firmware.

    :param robot_config:
    :return:
    """


def min_key(joint: str) -> str:
    """Constructs and returns a string in the form "<joint> min"

    :param joint: The name of the joint to be referenced
    :return: The name of the joint with the substring " min" appended
    """

    return joint + ' min'


def max_key(joint: str) -> str:
    """Constructs and returns a string in the form "<joint> max"

    :param joint: The name of the joint to be referenced
    :return: The name of the joint with the substring " max" appended
    """

    return joint + ' max'


def set_constraints(robot_config: ConfigParser):
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


def set_start_position(robot_config: ConfigParser):
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


def check_constraints(joint: str, pos: int) -> int:
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


def sendSerialCommand(ser: Serial, command: str):
    """Encodes and writes a string to the serial port

    :param ser: The PySerial object interface for the serial port
    :param command: The non-encoded string to be written.
    :return: None
    """

    log.info("serial send: ", str(command.lower()))
    ser.write(command.encode('ascii') + b"\r\n")     # write a string


def send_gcode(gcode: str):
    """Sends the G-code line to the serial port and prints it to the console for debugging purposes.

    :param gcode: The G-code string to be sent to the arm
    :return: None
    """
    if serial_board.ser:
        sendSerialCommand(serial_board.ser, gcode)
    print(gcode)


def move_arm_to(base_pos: int = None,
                shoul_pos: int = None,
                elbow_pos: int = None,
                elev_pos: int = None,
                rot_pos: int = None,
                grip_pos: int = None) -> list:

    """Orders a movement of the arm to an absolute position.
    Checks the passed positions against the joint constraints, constructs the G-Code commands based on those checks,
    and forwards the commands to the serial port.

    :param base_pos: The new coordinate of the base.  Default: None; will use current position.
    :param shoul_pos: The new coordinate of the shoulder.  Default: None; will use current position.
    :param elbow_pos: The new coordinate of the elbow.  Default: None; will use current position.
    :param elev_pos: The new coordinate of the wrist elevation.  Default: None; will use current position.
    :param rot_pos: The new coordinate of the rotation servo.  Default: None; will use current position.
    :param grip_pos: The new position of the gripper servo.  Default: None; will use current position.
    :return: The new arm position.
    """

    global arm_position
    global arm_axes
    global multiple_extruders
    global current_extruder

    if base_pos:
        arm_position[steppers[0]] = check_constraints(steppers[0], base_pos)
    if shoul_pos:
        arm_position[steppers[1]] = check_constraints(steppers[1], shoul_pos)
    if elbow_pos:
        arm_position[steppers[2]] = check_constraints(steppers[2], elbow_pos)
    if elev_pos:
        arm_position[steppers[3]] = check_constraints(steppers[3], elev_pos)
    if rot_pos:
        arm_position[steppers[4]] = check_constraints(steppers[4], rot_pos)
    if grip_pos:
        arm_position[servos[0]] = check_constraints(servos[0], grip_pos)

    # Switch the arm to Absolute Positioning using G90
    send_gcode("G90")

    # Construct G-Code string for movement using the G0 Linear Move command.
    # Form: G0 E<pos> X<pos> Y<pos> Z<pos>
    gcode = "G0 "

    for joint in steppers:
        # Change tool if using multiple extruders and the next extruder is not the current extruder
        if multiple_extruders and ('E' in arm_axes[joint]) and (int(arm_axes[joint].lstrip('E')) != current_extruder):
            # Send existing command
            send_gcode(gcode)

            extruder_num = int(arm_axes[joint].lstrip('E'))

            # Construct and send tool change (T) command
            gcode = "T" + str(extruder_num)
            send_gcode(gcode)
            current_extruder = extruder_num

            # Begin new G-Code string
            gcode = "G0 "

        # Append joint to gcode move command; remove any trailing digits from the axis name
        gcode += arm_axes[joint].rstrip('1234567890') + str(arm_position[joint]) + " "

    # Clean up trailing whitespace
    gcode = gcode.rstrip(" ")

    send_gcode(gcode)

    # Construct G-code string for manipulating the gripper using the M280 Servo Position command
    # Form: M280 P<index> S<pos>
    gcode = "M280 " + arm_axes[servos[0]] + " S" + str(arm_position[servos[0]])

    send_gcode(gcode)

    return arm_position


def move_arm_by(base_inc=0, shoul_inc=0, elbow_inc=0, elev_inc=0, rot_inc=0, grip_inc=0) -> list:
    """Orders a movement of the arm to a new position relative to its current one.
    Essentially just a wrapper for move_arm_to()
    that calculates the new position based on the increment.

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


def parse_command(command: str) -> list:
    """Splits the command into a list of tuples in the form (joint, inc|abs, distance).
    Where joint is the name of the joint (corresponds to an item in steppers or servos),
    inc|abs is either the string "inc" or "abs" and signals whether relative or absolute movement is to be used,
    distance is the coordinate for axis movement in the g-code

    :param command: The complete command, as received from LetsRobot.tv
    :return: The list of tuples containing all the movement commands received
    """

    split_list = command.split(';')
    command_list = []
    for subgroup in split_list:
        command_list.append(tuple(subgroup.split(',')))
    if __name__ == "__main__":
        print("Parsed command: ", command_list)

    return command_list


def set_joint_position(joint: str, move_type: str, coordinate: int):
    """ Sets the position of a joint for next movement.
    Will check constraints for movement before updating position.  Should be used in place of move_arm_to when building
    synchronous moves from a larger queue.

    :param joint: The joint to be moved.
    :param move_type: 'inc' for incremental movement or 'abs' for absolute movement.
    :param coordinate: The G-Code coordinate the joint will move to
    :return: None
    """

    global arm_position
    try:
        if move_type == "abs":
            arm_position[joint] = check_constraints(joint, coordinate)
        elif move_type == "inc":
            arm_position[joint] = check_constraints(joint, arm_position[joint] + coordinate)
        else:
            if __name__ == "__main__":
                print("Malformed command: ", move_type, " is not a valid move type. ", (joint, move_type, coordinate))

    except KeyError:
        if __name__ == "__main__":
            print("Malformed command: ", joint, " is not a valid joint. ", (joint, move_type, coordinate))
    except TypeError:
        if __name__ == "__main__":
            print("Malformed command: ", str(coordinate), " is not an integer. ", (joint, move_type, coordinate))


def setup(robot_config: ConfigParser):
    """Initialize options for the arm.
    Sets up serial board interface and reads user-defined setting from letsrobot.conf for axes, constraints, and
    start position.  Enables cold extrusion to allow extruder axes to be controlled without a hot end.

    :param robot_config: The ConfigParser object representation of letsrobot.conf
    :return: None
    """
    global multiple_extruders

    """
    if __name__ != "__main__":
        # Set up the serial interface
        serial_board.setup(robot_config)
    """
    # Set up the serial interface
    serial_board.setup(robot_config)

    multiple_extruders = robot_config.getboolean('bcn3d_moveo', 'multiple_extruders')

    bind_axes(robot_config)
    set_constraints(robot_config)
    set_start_position(robot_config)

    # Empty the incoming serial buffer
    if serial_board.ser:
        buffer_size = serial_board.ser.inWaiting()
        for line in serial_board.ser.read(buffer_size).decode("ascii"):
            print(line, end='')

    # Disable Cold Extrusion checking and set minimum extrusion temp to 0
    send_gcode("M302 P1 S0")

    # Move the arm to its startup position
    move_arm_to()


def move(args):
    """Interprets command received from the server and sends G-code to the serial port.
    This function is intended to take simple string commands from the server and convert them into the less
    human-readable G-code the MOVEO expects.

    :param args: The command dictionary received from the server
    :return: None
    """
    # Your custom command interpreter code goes here
    command_list = parse_command(args['command'])

    for command in command_list:
        try:
            set_joint_position(command[0], command[1], int(command[2]))
        except IndexError:
            print("Malformed Command: ", command, " only has ", len(command), " parts, not 3. ",
                  "Should be of form (joint, move_type, coordinate).")
        except TypeError or ValueError:
            print("Malformed Command: ", command, " is not of form (str, str, int).")

    move_arm_to()


if __name__ == '__main__':
    try:
        robot_config.readfp(open('letsrobot.conf'))
    except IOError:
        print("unable to read letsrobot.conf, please check that you have copied letsrobot.sample.conf to ",
              "letsrobot.conf and modified it appropriately.")
        sys.exit()
    except:
        print("Error in letsrobot.conf:", sys.exc_info()[0])
        sys.exit()

    setup(robot_config)
    print(steppers)
    print(servos)
    print(arm_axes)
    print(arm_constraints)
    print(arm_position)

    mode = input("Manual mode? Y/N:")
    if mode.upper() == "Y":
        print("Manual mode")
    else:
        import random
        print("Auto mode")
        random.seed()

        def random_command() -> str:
            """Constructs a random movement command for testing purposes.
            Command will contain between 1 and 6 segments, such that a single joint will be commanded only once
            or not at all.  Some coordinates will be beyond the default constraints.

            :return: A properly formatted command string that can be parsed and interpreted.
            """

            num_parts = random.randint(1, 6)
            used_joints = random.sample(list(steppers + servos), num_parts)
            command = ""
            for joint in used_joints:
                move_type = random.choice(["inc", "abs"])
                coordinate = random.randint(-200, 200)
                command += joint + ',' + move_type + ',' + str(coordinate) + ';'

            # Strip trailing semicolon
            command = command.rstrip(';')

            return command

    while True:
        print()
        print("Starting position:", arm_position)
        if mode.upper() == "Y":
            command = input('Enter command: ')
        else:
            command = random_command()
            print(command)

        command_list = parse_command(command)

        for command in command_list:
            try:
                set_joint_position(command[0], command[1], int(command[2]))
            except IndexError:
                print("Malformed Command: ", command, " only has ", len(command), " parts, not 3. ",
                      "Should be of form (joint, move_type, coordinate).")
            except TypeError or ValueError:
                print("Malformed Command: ", command, " is not of form (str, str, int).")

        print(arm_position)
        move_arm_to()
        input("Press enter to continue.")
