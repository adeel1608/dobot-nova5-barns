#!/usr/bin/env python3
import time
import argparse
from manipulate_move_new import run_skill   # ← the only import you need
import rclpy
from rclpy.node import Node  # Add this import
from dobot_msgs_v3.srv import ServoJ
import re

def load_commands_from_file(path):
    """
    Read `say_hi.txt`, extract every sequence of seven floats from
    ServoJ(j1,j2,j3,j4,j5,j6,t=...), and return a list of tuples.
    """
    pattern = re.compile(r'ServoJ\(\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*t=([-\d\.]+)\)')
    cmds = []
    with open(path, 'r') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                vals = tuple(float(m.group(i)) for i in range(1,8))
                cmds.append(vals)
    return cmds

class ServoJCommander(Node):
    def __init__(self):
        super().__init__('servoj_commander')
        self.cli = self.create_client(ServoJ, '/dobot_bringup_v3/srv/ServoJ')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for ServoJ service...')
        self.get_logger().info('Service ready.')

    def send_one(self, j1,j2,j3,j4,j5,j6,t):
        req = ServoJ.Request()
        req.j1, req.j2, req.j3 = j1, j2, j3
        req.j4, req.j5, req.j6 = j4, j5, j6
        req.t = t
        self.get_logger().info(f'Sending: {j1:.3f},{j2:.3f},{j3:.3f},{j4:.3f},{j5:.3f},{j6:.3f} @ t={t}')
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error(f'Failed: {fut.exception()}')

# ──────────────────────────────────────────────────────────────────
# 1)  PUT EACH ROUTINE INTO A FUNCTION
# ──────────────────────────────────────────────────────────────────
def home_north():
    """Espresso machine point commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("gotoJ_deg", 0, 30, -130, -100, -90, 0) #Main home 

def home_north_east():
    """Grinder points commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0) #Home 45 degree clockwise from main home

def home_east():
    """Train portafilter 1 commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -90, 30, -130, -100, -90, 0) #Home 90 degree clockwise from main home

def home_south_east():
    """Train portafilter 2 commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -135, 30, -130, -100, -90, 0) #Home 135 degree clockwise from main home

def home_south():
    """Train portafilter 3 commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", 180, 30, -130, -100, -90, 0) #Home 180 degree counter-clockwise from main home

def home_south_west():
    """Espresso machine point commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", 135, 30, -130, -100, -90, 0) #Home 135 degree counter-clockwise from main home

def home_west():
    """Grinder points commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", 90, 30, -130, -100, -90, 0) #Home 90 degree counter-clockwise from main home

def home_north_west():
    """Espresso machine point commands here"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", 45, 30, -130, -100, -90, 0) #Home 45 degree counter-clockwise from main home

def say_hi():
    #---Extra waving mode lol---#
    # Create and initialize the ServoJ commander
    rclpy.init()
    node = ServoJCommander()
    
    # Load and execute the ServoJ commands
    commands = load_commands_from_file('/home/adeel/barns_ws/src/pickn_place/pickn_place/say_hi.txt')
    for j1,j2,j3,j4,j5,j6,t in commands:
        node.send_one(j1,j2,j3,j4,j5,j6,0.1)
        time.sleep(0.06)
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

def unmount_port_1():
    """Unmount portafilter 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.2) 
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0)
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path

def grinder_port_1():
    """Grinder port 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True) 
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("approach_machine", "espresso_grinder", "tamper", True)
    time.sleep(1.0)
    run_skill("mount_machine", "espresso_grinder", "tamper", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)

def grinder_port_2():
    """Grinder port 2"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True) 
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("approach_machine", "espresso_grinder", "tamper", True)
    time.sleep(1.0)
    run_skill("mount_machine", "espresso_grinder", "tamper", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)

def grinder_port_3():
    """Grinder port 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True) 
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("approach_machine", "espresso_grinder", "tamper", True)
    time.sleep(1.0)
    run_skill("mount_machine", "espresso_grinder", "tamper", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)

def mount_port_1():
    """Mount portafilter 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_1", True)
    run_skill("mount_machine", "three_group_espresso", "group_1", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.2)
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home
def mount_port_2():
    """Mount portafilter 2"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_2", True)
    run_skill("mount_machine", "three_group_espresso", "group_2", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.2)
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home
def mount_port_3():
    """Mount portafilter 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 64.697701,-59.130959,-55.757610,-65.111610,21.462168,0.000163)
    run_skill("approach_machine", "three_group_espresso", "group_3", True)
    run_skill("mount_machine", "three_group_espresso", "group_3", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.2)
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def pick_pitcher_port_1():
    """Pick pitcher 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Mount the pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("moveEE", 0, -240, 0, 0, 0, 0)
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0)
    run_skill("set_gripper_position", 255, 90) #pitcher gripper grip
    run_skill("moveEE", 0, 0, 20, 0, 0, 0) #go up to pick pitcher
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345)#Move to midpoint
def pick_pitcher_port_2():
    """Mount pitcher 2"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Mount the pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 255, 90) #pitcher gripper grip
    run_skill("moveEE", 0, 0, 20, 0, 0, 0) #go up to pick pitcher
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345)#Move to midpoint
def pick_pitcher_port_3():
    """Mount pitcher 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Mount the pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("moveEE", 0, 240, 0, 0, 0, 0)
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0)
    run_skill("set_gripper_position", 255, 90) #pitcher gripper grip
    run_skill("moveEE", 0, 0, 20, 0, 0, 0) #go up to pick pitcher
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345)#Move to midpoint

def pour_pitcher_port_1():
    """Pour pitcher 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Pour the pitcher---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("gotoJ_deg", 85.997716,-17.821510,-132.407788,-29.390120,-73.953046,-0.093618, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
def pour_pitcher_port_2():
    """Pour pitcher """
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Pour the pitcher---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("gotoJ_deg", 85.997716,-17.821510,-132.407788,-29.390120,-73.953046,-0.093618, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
def pour_pitcher_port_3():
    """Pour pitcher 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Pour the pitcher---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("gotoJ_deg", 85.997716,-17.821510,-132.407788,-29.390120,-73.953046,-0.093618, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint


def hot_water_port_1():
    """Get hot water in pitcher 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Get hot water in pitcher 1---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("moveEE", 100, 0, 0, 0, 0, 0) #Move in
    time.sleep(8.0)
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -100, 0, 0, 0, 0, 0) #Move out
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint

def place_pitcher_port_1():
    """Place pitcher 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Place the pitcher---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("moveEE", 0, -240, 10, 0, 0, 0) #move right to the pitcher point
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move in
    run_skill("set_gripper_position", 75, 0) #Open gripper
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move in
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def place_pitcher_port_2():
    """Place pitcher 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Place the pitcher---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 75, 0) #Open gripper
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def place_pitcher_port_3():
    """Place pitcher 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Place the pitcher---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("moveEE", 0, 240, 10, 0, 0, 0) #move right to the pitcher point
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move in
    run_skill("set_gripper_position", 75, 255) #Open gripper
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move in
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def unmount_port_2():
    """Unmount portafilter 2"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.2) 
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0)
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)

def mount_port_2():
    """Mount portafilter 2"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_2", True)
    run_skill("mount_machine", "three_group_espresso", "group_2", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.2)
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home


def hot_water_port_2():
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("moveEE", 100, 0, 0, 0, 0, 0) #Move in
    time.sleep(8.0)
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -100, 0, 0, 0, 0, 0) #Move out
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint



def unmount_port_3():
    """Unmount portafilter 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.2) 
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0)
    run_skill("gotoJ_deg", 64.697701,-59.130959,-55.757610,-65.111610,21.462168,0.000163)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)


def hot_water_port_3():
    """Get hot water in pitcher 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Get hot water in pitcher 3---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("moveEE", 100, 0, 0, 0, 0, 0) #Move in
    time.sleep(8.0)
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -100, 0, 0, 0, 0, 0) #Move out
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint



def cleaner_port_1():
    """Cleaner port 1"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    run_skill("moveJ_deg", 122, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 110.132784,7.214473,-140.810214,-44.785857,-70.153760,-4.137882)
    run_skill("moveEE", 0, 150, 0, 0, 0, 0)
    #---Twist around---#
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -180)
    #---Cleaning loop---#
    run_skill("moveEE", 0, 0, -77.5, 0, 0, 0)
    run_skill("moveEE", 0, 0, 0, -1.25, 0, 0)
    run_skill("moveEE", 0, 0, 0, 1.25, 1.25, 0)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, 1.25)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, -1.25)
    run_skill("moveEE", 0, 0, 0, 0, 1.25, 0)
    run_skill("moveEE", 0, 0, 77.5, 0, 0, 0)
    run_skill("moveEE", -88, 0, 0, 0, 0, 0)
    run_skill("moveEE", 0, 0, -77.5, 0, 0, 0)
    run_skill("moveEE", 0, 0, 0, -1.25, 0, 0)
    run_skill("moveEE", 0, 0, 0, 1.25, 1.25, 0)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, 1.25)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, -1.25)
    run_skill("moveEE", 0, 0, 0, 0, 1.25, 0)
    run_skill("moveEE", 0, 0, 77.5, 0, 0, 0)
    #---Twist around---#
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 180)
    #---Move Towards Espresso machine---#
    run_skill("gotoJ_deg", 110.132784,7.214473,-140.810214,-44.785857,-70.153760,-4.137882)
    run_skill("gotoJ_deg", 89.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)
    run_skill("moveJ_deg", -122, 0, 0, 0, 0, 0)

def cleaner_port_2():
    """Cleaner port 2"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("moveJ_deg", 122, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 110.132784,7.214473,-140.810214,-44.785857,-70.153760,-4.137882)
    run_skill("moveEE", 0, 150, 0, 0, 0, 0)
    #---Twist around---#
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -180)
    #---Cleaning loop---#
    run_skill("moveEE", 0, 0, -77.5, 0, 0, 0)
    run_skill("moveEE", 0, 0, 0, -1.25, 0, 0)
    run_skill("moveEE", 0, 0, 0, 1.25, 1.25, 0)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, 1.25)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, -1.25)
    run_skill("moveEE", 0, 0, 0, 0, 1.25, 0)
    run_skill("moveEE", 0, 0, 77.5, 0, 0, 0)
    run_skill("moveEE", -88, 0, 0, 0, 0, 0)
    run_skill("moveEE", 0, 0, -77.5, 0, 0, 0)
    run_skill("moveEE", 0, 0, 0, -1.25, 0, 0)
    run_skill("moveEE", 0, 0, 0, 1.25, 1.25, 0)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, 1.25)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, -1.25)
    run_skill("moveEE", 0, 0, 0, 0, 1.25, 0)
    run_skill("moveEE", 0, 0, 77.5, 0, 0, 0)
    #---Twist around---#
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 180)
    #---Move Towards Espresso machine---#
    run_skill("gotoJ_deg", 110.132784,7.214473,-140.810214,-44.785857,-70.153760,-4.137882)
    run_skill("gotoJ_deg", 89.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)
    run_skill("moveJ_deg", -32, 0, 0, 0, 0, 0)

def cleaner_port_3(): 
    """Cleaner port 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("moveJ_deg", 122, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 110.132784,7.214473,-140.810214,-44.785857,-70.153760,-4.137882)
    run_skill("moveEE", 0, 150, 0, 0, 0, 0)
    #---Twist around---#
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -180)
    #---Cleaning loop---#
    run_skill("moveEE", 0, 0, -77.5, 0, 0, 0)
    run_skill("moveEE", 0, 0, 0, -1.25, 0, 0)
    run_skill("moveEE", 0, 0, 0, 1.25, 1.25, 0)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, 1.25)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, -1.25)
    run_skill("moveEE", 0, 0, 0, 0, 1.25, 0)
    run_skill("moveEE", 0, 0, 77.5, 0, 0, 0)
    run_skill("moveEE", -88, 0, 0, 0, 0, 0)
    run_skill("moveEE", 0, 0, -77.5, 0, 0, 0)
    run_skill("moveEE", 0, 0, 0, -1.25, 0, 0)
    run_skill("moveEE", 0, 0, 0, 1.25, 1.25, 0)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, 1.25)
    run_skill("moveEE", 0, 0, 0, 0, -1.25, -1.25)
    run_skill("moveEE", 0, 0, 0, 0, 1.25, 0)
    run_skill("moveEE", 0, 0, 77.5, 0, 0, 0)
    #---Twist around---#
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 180)
    #---Move Towards Espresso machine---#
    run_skill("gotoJ_deg", 110.132784,7.214473,-140.810214,-44.785857,-70.153760,-4.137882)
    run_skill("gotoJ_deg", 89.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)
    run_skill("moveJ_deg", -32, 0, 0, 0, 0, 0)

def milk_frothing():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0, 1.0, 0.2)
    for i in range(3):
        time.sleep(1.0)
        run_skill("move_to", "left_steam_wand", 0.12)
    run_skill("get_machine_position", "left_steam_wand")
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0)
    run_skill("gotoJ_deg", 10.067266,-47.009254,-129.773193,-3.069591,-79.829887,-0.112283)
    run_skill("moveEE", 0, 20, -160, 0, 0, 0)
    run_skill("moveEE", 88, 0, 0, 0, 0, 0)
    run_skill("set_gripper_position", 255, 95)
    # run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -77, 0, 20, 0, 0, 0)
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0, 1.0, 0.2)
    run_skill("approach_machine", "left_steam_wand", "deep_froth", True)
    run_skill("mount_machine", "left_steam_wand", "deep_froth", True)
    run_skill("mount_machine", "left_steam_wand", "light_froth", True)
    run_skill("approach_machine", "left_steam_wand", "light_froth", True)
    run_skill("moveEE", -30, 30, -30, 0, 0, 0)
    run_skill("gotoJ_deg", -42.960346,-27.362787,-119.737602,-32.588718,-42.844364, 0.0, 1.0, 0.2)
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    time.sleep(5.0) #Random wait time
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100)
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0)
    run_skill("gotoJ_deg", 10.067266,-47.009254,-129.773193,-3.069591,-79.829887,-0.112283)
    run_skill("moveEE", 0, 20, -160, 0, 0, 0)
    run_skill("moveEE", 88, 0, 0, 0, 0, 0)
    # run_skill("set_gripper_position", 255, 95)
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -77, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 0, 30, -130, -100, -90, 0)

CUP_PARAMS = {
    '7oz': {
        'twist_back':   (-142.260873, -17.875853, 10.033241, 8.226858, -0.089241, -47.128327),
        'approach':     (200, 0, 0, 0, 0, 0),
        'grip_width':   139,
        'retreat':      (-350, 0, 0, 0, 0, 0),
    },
    '9oz': {
        'twist_back':   (-94.891786, -3.735069, 0.159065, 4.110916, -0.085106, 0.174636),
        'approach':     (200, 0, 0, 0, 0, 0),
        'grip_width':   139,
        'retreat':      (-200, 0, 0, 0, 0, 0),
    },
    '12oz': {
        'twist_back':   (-59.968582, -15.674384, 7.995781, 8.130459, 0.019112, 35.111396),
        'approach':     (200, 0, 0, 0, 0, 0),
        'grip_width':   139,
        'retreat':      (-350, 0, 0, 0, 0, 0),
    },
}

def grab_cup(size):
    params = CUP_PARAMS.get(size)
    if not params:
        print(f"[ERROR] unknown cup size {size!r}")
        return
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", *params['twist_back'])
    run_skill("moveEE", *params['approach'])
    run_skill("set_gripper_position", 255, params['grip_width'])
    run_skill("moveEE", *params['retreat'])
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)

PLACE_PARAMS = {
    'stage_1': { 
        'twist': (50,0,0,0,0,0),
        'pose': (74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964),
        'stage_home': (106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'twist_back': (-64.032688, 0, 0, 0, 0, 0),
        'twist_serve': (31.983258, 0, 0, 0, 0, 0),
        'pick': (74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478),
        'above_serve': (114.123176,-17.424376,-139.079935,-23.095763,-65.826354,-0.154358),
        'serve': (114.103052,-48.360801,-140.051323,8.823533,-65.854705,-0.182434)
    },
    'stage_2': { 
        'twist': (50,0,0,0,0,0),
        'pose': (74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964),
        'stage_home': (106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'twist_back': (-64.032688, 0, 0, 0, 0, 0),
        'twist_serve': (31.983258, 0, 0, 0, 0, 0),
        'pick': (74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478),
        'above_serve': (114.123176,-17.424376,-139.079935,-23.095763,-65.826354,-0.154358),
        'serve': (114.103052,-48.360801,-140.051323,8.823533,-65.854705,-0.182434)
    },
}

def place_cup(stage):
    p = PLACE_PARAMS.get(stage)
    if not p:
        print(f"[ERROR] unknown stage {stage!r}")
        return
    run_skill("moveJ_deg", *p['twist'])
    run_skill("gotoJ_deg", *p['pose'])
    run_skill("set_gripper_position", 50, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0,0,150,0,0,0)
    run_skill("gotoJ_deg", *p['stage_home'])
    run_skill("moveJ_deg", *p['twist_back'])
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def serve(stage):
    p = PLACE_PARAMS.get(stage)
    if not p:
        print(f"[ERROR] unknown stage {stage!r}")
        return
    #Pickup from staging place in delivery
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("moveJ_deg", *p['twist_serve'])#run_skill("moveJ_deg", 31.983258, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", *p['pick'])#run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478)
    run_skill("moveEE", 0, 0, -140, 0, 0, 0)
    run_skill("set_gripper_position", 55, 125)
    run_skill("set_servo_timing", 0.20)
    run_skill("gotoJ_deg", *p['pick'])#run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478, 1.0, 0.2)
    run_skill("gotoJ_deg", *p['above_serve'])
    run_skill("gotoJ_deg", *p['serve'])
    run_skill("set_gripper_position", 55, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 140, 0, 0, 0)
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
        
def home(position):
    """
    position should be one of:
      'north', 'north_east', 'east',
      'south_east', 'south', 'south_west',
      'west', 'north_west'
    This will call the corresponding home_<position>().
    """
    func_name = f"home_{position}"      # e.g. "home_north_east"
    fn = globals().get(func_name, None)
    if not fn:
        print(f"[ERROR] no function named {func_name}")
        return
    print(f"--- running {func_name}() ---")
    fn()

def pull_espresso(port):
    """
    port should be 'port_1', 'port_2' or 'port_3'.
    This will call, in order:
      unmount_<port>(), grinder_<port>(), mount_<port>(),
      pick_pitcher_<port>(), pour_pitcher_<port>(),
      hot_water_<port>(), place_pitcher_<port>()
    """
    steps = [
        'unmount',
        'grinder',
        'mount',
        'pick_pitcher',
        'pour_pitcher',
        'hot_water',
        'place_pitcher',
    ]

    for step in steps:
        func_name = f"{step}_{port}"               # e.g. "unmount_port_2"
        fn = globals().get(func_name, None)
        if not fn:
            print(f"[ERROR] no function named {func_name}")
            return
        print(f"--- running {func_name}() ---")
        fn()

def pick_cup(size, stage):
    """
    size: '7oz', '9oz', or '12oz'
    stage: 'stage_1' or 'stage_2'
    """
    grab_cup(size)
    place_cup(stage)


def get_machine_position():
    """Machine position updates here (when starting from home position)."""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("moveJ_deg", -35, 0, 0, 0, 0, 0)
    for i in range(3):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.12)
    run_skill("get_machine_position", "espresso_grinder")
    run_skill("gotoJ_deg", 7.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
    for i in range(3):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.12)
    run_skill("get_machine_position", "three_group_espresso")
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def random_tests():
    print("Press ENTER to proceed...")
    input()
    run_skill("refresh_position")
    print("Press ENTER to proceed...")
    input()
    run_skill("gotoJ_deg", 58.688404,-22.412622,-122.100609,-34.847675,-34.263073,-0.521874) #Espresso home

# Add more sequences here as plain Python functions…

# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    "home": home,
    "pull_espresso":pull_espresso,
    "pick_cup": pick_cup,
    "serve": serve,
    "get_machine_position": get_machine_position,
    "test":random_tests
    # "demo": demo_raw,
}

# ------------------------------------------------------------------
#  CLI – interactive menu that keeps prompting until you quit
# ------------------------------------------------------------------
import inspect

def _main():
    names = list(SEQUENCES.keys())
    while True:
        print("\n🔧 Sequences:")
        for i, name in enumerate(names, start=1):
            print(f"  {i}) {name}")
        print("  q) Quit\n")

        choice = input("Which? ").strip()
        if choice.lower() in ("q", "quit", "exit"):
            print("Bye!")
            break

        # allow numeric or name lookup
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(names):
                seq_name = names[idx]
            else:
                print(f"[ERROR] Invalid number: {choice}")
                continue
        else:
            seq_name = choice

        fn = SEQUENCES.get(seq_name)
        if not fn:
            print(f"[ERROR] Unknown sequence: {seq_name}")
            continue

        # auto-prompt for parameters
        sig = inspect.signature(fn)
        kwargs = {}
        for param in sig.parameters.values():
            val = input(f"{param.name}? ").strip()
            kwargs[param.name] = val

        print(f"\n▶ Running {seq_name} with {kwargs}\n")
        fn(**kwargs)


if __name__ == "__main__":
    _main()
