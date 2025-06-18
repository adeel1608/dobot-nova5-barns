#!/usr/bin/env python3
"""
sequences.py  –  high-level routines ***and*** a tiny CLI to run one.

Usage examples
--------------
python3 sequences.py teach
python3 sequences.py mount --loop      # repeat until Ctrl-C
python3 sequences.py --list            # show available names
"""
import time
import argparse
from manipulate_move import run_skill   # ← the only import you need
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

def espresso_port_1():
    '''Port 1'''
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down #Move down
    #---Go to the grinder---#
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True) 
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 19, -19, 0, 0, 0, 0) #Pushing the button #Pushing the button
    time.sleep(2.0)
    run_skill("gotoJ_deg", -38.448997,-66.328377,-92.612915,-16.689844,-89.484047,-0.952721)# Tamping (need to teach this and button based on marker)# Tamping (need to teach this and button based on marker)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_1", True)
    run_skill("mount_machine", "three_group_espresso", "group_1", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper #Open gripper
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #---Get a cup---#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -86, 0, 0, 0, 0, 0) #Twist back to align with the cups
    run_skill("gotoJ_deg", 14.608274,-10.505052,-46.947823,-32.527233,90.108307,-165.236038) #Current 12oz cup position
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -200, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    #---Grab pitcher---#
    # run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, -240, 0, 0, 0, 0) #Move right to align with pitcher port 1
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move infront to grab pitcher port 1
    run_skill("set_gripper_position", 255, 90) #Grab pitcher
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", 0, 0, 15, 0, 0, 0) #pick the pitcher up
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("approach_machine", "three_group_espresso", "pour_espresso_pitcher_2", True) #Move to midpoint
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup 
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, -240, 0, 0, 0, 0) #Move right to align with pitcher port 1
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move infront to grab pitcher port 1
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Mid point
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def espresso_port_2():
    '''Port 2'''
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Go to the grinder---#
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 19, -19, 0, 0, 0, 0) #Pushing the button
    time.sleep(2.0)
    run_skill("gotoJ_deg", -38.448997,-66.328377,-92.612915,-16.689844,-89.484047,-0.952721)# Tamping (need to teach this and button based on marker)# run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_2", True)
    run_skill("mount_machine", "three_group_espresso", "group_2", True)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, 2, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #---Get a cup---#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -86, 0, 0, 0, 0, 0) #Twist back to align with the cups
    run_skill("gotoJ_deg", 14.608274,-10.505052,-46.947823,-32.527233,90.108307,-165.236038) #Current 12oz cup position
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -200, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    # #---Grab pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 255, 90) #Grab pitcher
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", 0, 0, 15, 0, 0, 0) #pick the pitcher up
    run_skill("approach_machine", "three_group_espresso", "pour_espresso_pitcher_2", True) #Move to midpoint
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    
def espresso_port_3():
    """Espresso port 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Go to the grinder---#
    run_skill("gotoJ_deg", 64.697701,-59.130959,-55.757610,-65.111610,21.462168,0.000163)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 19, -19, 0, 0, 0, 0) #Pushing the button
    time.sleep(2.0)
    run_skill("gotoJ_deg", -38.448997,-66.328377,-92.612915,-16.689844,-89.484047,-0.952721)# Tamping (need to teach this and button based on marker)# run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 64.697701,-59.130959,-55.757610,-65.111610,21.462168,0.000163)#A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_3", True)
    run_skill("mount_machine", "three_group_espresso", "group_3", True)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, 7.5, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("moveEE", -50, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #---Get a cup---#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -86, 0, 0, 0, 0, 0) #Twist back to align with the cups
    run_skill("gotoJ_deg", 14.608274,-10.505052,-46.947823,-32.527233,90.108307,-165.236038) #Current 12oz cup position
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -200, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    #---Grab pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, 240, 0, 0, 0, 0) #Move left to align with pitcher port 3
    run_skill("enforce_rxry")
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move infront to grab pitcher port 3
    run_skill("set_gripper_position", 255, 90) #Grab pitcher
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", 0, 0, 15, 0, 0, 0) #pick the pitcher up
    run_skill("enforce_rxry")
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("enforce_rxry")
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, 240, 0, 0, 0, 0) #Move left to align with pitcher port 3
    run_skill("enforce_rxry")
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move in
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("enforce_rxry")
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def americano_port_1():
    '''Port 1'''
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Go to the grinder---#
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 19, -19, 0, 0, 0, 0) #Pushing the button
    time.sleep(2.0)
    run_skill("gotoJ_deg", -38.448997,-66.328377,-92.612915,-16.689844,-89.484047,-0.952721)# Tamping (need to teach this and button based on marker)# run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_1", True)
    run_skill("mount_machine", "three_group_espresso", "group_1", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #---Get a cup---#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -86, 0, 0, 0, 0, 0) #Twist back to align with the cups
    run_skill("gotoJ_deg", 14.608274,-10.505052,-46.947823,-32.527233,90.108307,-165.236038) #Current 12oz cup position
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -200, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    #---Grab pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, -240, 0, 0, 0, 0) #Move right to align with pitcher port 1
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move infront to grab pitcher port 1
    run_skill("set_gripper_position", 255, 90) #Grab pitcher
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", 0, 0, 15, 0, 0, 0) #pick the pitcher up
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("approach_machine", "three_group_espresso", "pour_espresso_pitcher_2", True) #Move to midpoint
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveEE", -5, 0, 0, 0, 0, 0)
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("enforce_rxry")
    run_skill("moveEE", 125, 0, 0, 0, 0, 0) #Move in
    time.sleep(8.0)
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -125, 0, 0, 0, 0, 0) #Move out
    run_skill("enforce_rxry")
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveEE", -5, 0, 0, 0, 0, 0)
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, -240, 0, 0, 0, 0) #Move right to align with pitcher port 1
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move infront to grab pitcher port 1
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def americano_port_2():
    '''Port 2'''
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Go to the grinder---#
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 19, -19, 0, 0, 0, 0) #Pushing the button
    time.sleep(2.0)
    run_skill("gotoJ_deg", -38.448997,-66.328377,-92.612915,-16.689844,-89.484047,-0.952721)# Tamping (need to teach this and button based on marker)# run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_2", True)
    run_skill("mount_machine", "three_group_espresso", "group_2", True)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, 2, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #---Get a cup---#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -86, 0, 0, 0, 0, 0) #Twist back to align with the cups
    run_skill("gotoJ_deg", 14.608274,-10.505052,-46.947823,-32.527233,90.108307,-165.236038) #Current 12oz cup position
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -200, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    # #---Grab pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 255, 90) #Grab pitcher
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", 0, 0, 15, 0, 0, 0) #pick the pitcher up
    run_skill("approach_machine", "three_group_espresso", "pour_espresso_pitcher_2", True) #Move to midpoint
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("enforce_rxry")
    run_skill("moveEE", 125, 0, 0, 0, 0, 0)
    time.sleep(8.0) #Random wait time
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -125, 0, 0, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    
def americano_port_3():
    """Espresso port 3"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Go to the grinder---#
    run_skill("gotoJ_deg", 64.697701,-59.130959,-55.757610,-65.111610,21.462168,0.000163)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 19, -19, 0, 0, 0, 0) #Pushing the button
    time.sleep(2.0)
    run_skill("gotoJ_deg", -38.448997,-66.328377,-92.612915,-16.689844,-89.484047,-0.952721)# Tamping (need to teach this and button based on marker)# run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 64.697701,-59.130959,-55.757610,-65.111610,21.462168,0.000163)
    run_skill("approach_machine", "three_group_espresso", "group_3", True)
    run_skill("mount_machine", "three_group_espresso", "group_3", True)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, 7.5, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("moveEE", -50, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #---Get a cup---#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -86, 0, 0, 0, 0, 0) #Twist back to align with the cups
    run_skill("gotoJ_deg", 14.608274,-10.505052,-46.947823,-32.527233,90.108307,-165.236038) #Current 12oz cup position
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -200, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    #---Grab pitcher---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, 240, 0, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("moveEE", 131.614857, 0, 0, 0, 0, 0) #Move infront to grab pitcher port 1
    run_skill("set_gripper_position", 255, 90) #Grab pitcher
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", 0, 0, 15, 0, 0, 0) #pick the pitcher up
    run_skill("enforce_rxry")
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("enforce_rxry")
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("enforce_rxry")
    run_skill("moveEE", 125, 0, 0, 0, 0, 0)
    time.sleep(5.0) #Random wait time
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -125, 0, 0, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("enforce_rxry")
    run_skill("moveEE", 0, 240, 0, 0, 0, 0)
    run_skill("enforce_rxry")
    run_skill("moveEE", 131.614857, 0, -10, 0, 0, 0)
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -131.614857, 0, 0, 0, 0, 0) #Move back
    run_skill("enforce_rxry")
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def clean_port_1():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Move Towards Cleaner---#
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
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
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_1", True)
    run_skill("mount_machine", "three_group_espresso", "group_1", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    #---Go home---#
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def clean_port_2():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Move Towards Cleaner---#
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", 32, 0, 0, 0, 0, 0)
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
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_2", True)
    run_skill("mount_machine", "three_group_espresso", "group_2", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    #---Go home---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def clean_port_3():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount portafilter---#
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #---Move Towards Cleaner---#
    run_skill("gotoJ_deg", 64.697701,-59.130959,-55.757610,-65.111610,21.462168,0.000163)
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", 32, 0, 0, 0, 0, 0)
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
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_3", True)
    run_skill("mount_machine", "three_group_espresso", "group_3", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    #---Go home---#
    run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

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

def serve_cup():
     #Pickup from staging place in delivery
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("moveJ_deg", 31.983258, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478)
    run_skill("moveEE", 0, 0, -140, 0, 0, 0)
    run_skill("set_gripper_position", 55, 125)
    time.sleep(1.0)
    run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478, 1.0, 0.2)
    run_skill("gotoJ_deg", 114.123176,-17.424376,-139.079935,-23.095763,-65.826354,-0.154358, 1.0, 0.2)
    run_skill("gotoJ_deg", 114.103052,-48.360801,-140.051323,8.823533,-65.854705,-0.182434, 1.0, 0.2)
    run_skill("set_gripper_position", 55, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 140, 0, 0, 0)
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    
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
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("set_servo_timing", 0.15)
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    #---Go home---#
    run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

# Add more sequences here as plain Python functions…

# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    "home": home_north,
    
    "espresso_port_1": espresso_port_1,
    "espresso_port_2": espresso_port_2,
    "espresso_port_3": espresso_port_3,
    "americano_port_1": americano_port_1,
    "americano_port_2": americano_port_2,
    "americano_port_3": americano_port_3,
    "clean_port_1": clean_port_1,
    "clean_port_2": clean_port_2,
    "clean_port_3": clean_port_3,
    "serve_cup": serve_cup,
    "get_machine_position": get_machine_position,
    "random_tests": random_tests,
    # "demo": demo_raw,
}

# ------------------------------------------------------------------
#  CLI – interactive menu that keeps prompting until you quit
# ------------------------------------------------------------------
def _main():
    print("🔧  Pick-and-Place Interactive Menu")
    print("Type the name to run a sequence, or 'q' to quit.\n")

    while True:
        # 1) show the current list
        print("Available sequences:")
        for name in SEQUENCES:
            print(f"  • {name}")

        # 2) prompt the user
        choice = input("\nWhich sequence? (q to exit) ").strip().lower()

        if choice in ("q", "quit", "exit"):
            print("Bye!")
            break

        if choice not in SEQUENCES:
            print(f"❌  '{choice}' is not a valid sequence. Try again.\n")
            continue

        # 3) run the chosen sequence *once*
        try:
            SEQUENCES[choice]()        # ← call the function
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted. Returning to menu.\n")
        else:
            print("\n✅  Finished. Back to menu.\n")


if __name__ == "__main__":
    _main()
