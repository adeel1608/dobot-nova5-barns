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
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6) 
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down #Move down
    #---Go to the grinder---#
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
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
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", "group_1", True)
    run_skill("mount_machine", "three_group_espresso", "group_1", True)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)
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
    time.sleep(0.5)
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
    run_skill("gotoJ_deg", 54.948524,-2.658781,-118.554352,-81.672974,-62.213612,15.281714)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("gotoJ_deg", 59.260826,-24.256765,-117.766205,-37.977230,-31.219501,-0.000075)#run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
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
    time.sleep(0.5)
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
    run_skill("gotoJ_deg", -13.971616,0.391470,-134.839050,-52.114738,-99.035011,-4.152503)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("gotoJ_deg", -11.653136,-23.143291,-123.020676,-31.719095,-98.926636,-4.151273)#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
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
    time.sleep(0.5)
    run_skill("gotoJ_deg", -38.448997,-66.328377,-92.612915,-16.689844,-89.484047,-0.952721)# Tamping (need to teach this and button based on marker)# run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    #---Mount the portafilter---#
    run_skill("gotoJ_deg", -5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815) #A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 26.767265,-30.200554,-115.892044,-33.907444,-17.660034,-0.000201) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 25.267460,-33.068756,-110.554199,-36.858906,-16.902269,0.000000)#run_skill("approach_machine", "three_group_espresso", "group_1", True)
    run_skill("gotoJ_deg", 25.283249,-28.951748,-109.164742,-42.406891,-16.884111,0.002184)#run_skill("mount_machine", "three_group_espresso", "group_1", True)
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
    run_skill("set_servo_timing", 0.10) #Max speed
    run_skill("set_gripper_position", 255, 0)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("gotoJ_deg", 32.493378,-17.182411,-134.412003,-26.294939,-54.549332,-5.702014)#run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    run_skill("set_gripper_position", 255, 255) #close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0) #Move down
    #------------------------------------------------Go to the grinder--------------------------------------------------#
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -38.489487,-67.277573,-85.163345,-25.213760,-91.449356,0.213506)#run_skill("moveEE", 19, -19, 0, 0, 0, 0) #Pushing the button
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    # time.sleep(0.5)
    run_skill("gotoJ_deg", -40.431055,-64.315591,-91.667289,-23.157440,-94.419250,0.302350)# Tamping (need to teach this and button based on marker)# run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp
    # time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", -32.837723,-2.957932,-128.257645,-89.085014,-79.229942,9.602360) #Espresso grinder home
    # #----------------------------------------------------Mount the portafilter-----------------------------#
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 57.162277,-2.957932,-128.257645,-89.085014,-79.229942,9.602360)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)#A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", 51.648415,-38.281811,-97.828384,-43.889969,7.960432,0.000048)#A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", 49.159821,-41.850647,-92.341583,-45.888557,7.333865,0.000000)# run_skill("approach_machine", "three_group_espresso", "group_2", True)
    run_skill("gotoJ_deg", 49.156429,-38.538681,-91.044006,-50.460274,7.359858,0.000062)# run_skill("mount_machine", "three_group_espresso", "group_2", True)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, 2, 0, 0, 0)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("move_portafilter_arc",47)
    run_skill("set_gripper_position", 255, 0) #Open gripper
    run_skill("set_servo_timing", 0.10) #Max speed
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #-----------------------------------------------------Get a cup-----------------------------------#
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
    # # #--------------------------------------------------Grab pitcher----------------------------------#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("gotoJ_deg", 22.801836,-45.788181,-123.834488,-4.497542,-66.242622,-3.080353)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 255, 90) #Grab pitcher
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", 0, 0, 15, 0, 0, 0) #pick the pitcher up
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344)#run_skill("approach_machine", "three_group_espresso", "pour_espresso_pitcher_2", True) #Move to midpoint
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.10) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6) #Random wait time
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6) #Random wait time
    run_skill("moveEE", 125, 0, 0, 0, 0, 0)
    time.sleep(8.0) #Random wait time
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -125, 0, 0, 0, 0, 0)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6) #Random wait time
    run_skill("gotoJ_deg", 83.502903,-18.497363,-131.219489,-29.906347,-76.448125,-0.076788, 1.0, 0.2) #Move to the cup
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100, 1.0, 0.2) #Tilt the pitcher to pour
    run_skill("set_servo_timing", 0.1) #Increase robot speed
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344) #Move to midpoint
    run_skill("gotoJ_deg", 22.801836,-45.788181,-123.834488,-4.497542,-66.242622,-3.080353)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("gotoJ_deg", 34.671349,-38.978451,-145.993790,5.428446,-55.282234,-0.268344)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True) #Move down to the level of the pitcher
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
    time.sleep(0.5)
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
    run_skill("gotoJ_deg", 62.681637,-60.740463,-52.229523,-67.053505,19.194101,0.000000)#run_skill("approach_machine", "three_group_espresso", "group_3", True)
    run_skill("gotoJ_deg", 62.709465,-58.428642,-50.593380,-70.970383,19.180241,0.000391)#run_skill("mount_machine", "three_group_espresso", "group_3", True)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, 4.5, 0, 0, 0)
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

# ─── CUP GRAB PARAMETERS ────────────────────────────────────────────────────────
# twist_back: joint angles to back off before approach
# approach:   end-effector offsets to move in
# grip_width: how wide to open the gripper
# retreat:    end-effector offsets to back away
GRAB_CUP_PARAMS = {
    '12oz': {
        'twist_back':   (-142.260873, -17.875853,  10.033241,   8.226858,  -0.089241, -47.128327),
        'approach':     ( 200,        0,          0,          0,          0,         0),
        'grip_width':   139,
        'retreat':      (-350,        0,          0,          0,          0,         0),
    },
    '9oz': {
        'twist_back':   ( -94.810739,  -3.781208,   0.171860,   4.133488,  -0.141411,   0.198394),
        'approach':     ( 284,         5,           -10,          0,          0,         0),
        'grip_width':   144,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
    '7oz': {
        'twist_back':   ( -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037),
        'approach':     ( 245,         15,           -10,          0,          0,         0),
        'grip_width':   140,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
}
# ─── CUP PLACEMENT / SERVE PARAMETERS ────────────────────────────────────────────
# twist:       initial joint-1 twist into staging area
# pose:        joint targets to move into place over cup
# stage_home:  safe “staging” home pose
# twist_back:  untwist back towards machine
# twist_serve: joint-1 twist to present cup
# pick/above_serve/serve:
#   intermediate poses for the serve_cup() routine
PLACE_CUP_PARAMS = {
    'stage_1': {
        'twist':        (  66,   0,   0,   0,    0,    0),
        'pose':         ( 154.125778,-52.670326,-116.881065,-10.357515,-25.897715,0.057296),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_2': {
        # identical to stage_1 for now; tweak if you need a different trajectory
        'twist':        (  69,   0,   0,   0,    0,    0),
        'pose':         ( 159.539737,-56.095770,-101.541641,-22.208326,-20.521090,0.048888),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
}

def _prep_for_grab():
    """Perform common refresh and initial positioning steps before grabbing a cup."""
    run_skill("refresh_position")
    time.sleep(2.0)

def grab_cup(size):
    """
    Approaches and picks up the specified cup.
    
    Args:
        size (str): Cup size key, e.g. '7oz', '9oz', '12oz'.
    """
    params = GRAB_CUP_PARAMS.get(size)
    if not params:
        print(f"[ERROR] unknown cup size: {size!r}")
        return
    _prep_for_grab()
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chosen for grabbing the cup
    # Rotate joint angles to back away before approach
    run_skill("moveJ_deg", *params['twist_back'])
    # Move end-effector into approach position
    run_skill("moveEE", *params['approach'])
    # Close gripper to grasp the cup
    run_skill("set_gripper_position", 255, params['grip_width'])
    # Retract after gripping
    run_skill("moveEE", *params['retreat'])
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)

def place_cup(stage):
    """
    Places the currently held cup at the specified stage.
    
    Args:
        stage (str): Placement stage key, e.g. 'stage_1', 'stage_2'.
    """
    params = PLACE_CUP_PARAMS.get(stage)
    if not params:
        print(f"[ERROR] unknown stage: {stage!r}")
        return
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)
    # Move into staging twist angle
    run_skill("moveJ_deg", *params['twist'])
    # Move to target placement pose
    run_skill("gotoJ_deg", *params['pose'])
    # Open gripper to release cup
    run_skill("set_gripper_position", 50, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0,0,150,0,0,0) # Move up after placing cup
    run_skill("gotoJ_deg", *params['stage_home']) # Move to staging home position
    run_skill("moveJ_deg", *params['twist_back']) # Untwist back towards machine
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def pick_cup(size, stage):
    """
    Full pick-and-place sequence for cups.
    
    Args:
        size (str): Cup size key (e.g., '7oz', '9oz', '12oz').
        stage (str): Placement stage key ('stage_1' or 'stage_2').
    """
    grab_cup(size)
    place_cup(stage)

def serve(stage):
    params = PLACE_CUP_PARAMS.get(stage)
    if not params:
        print(f"[ERROR] unknown stage {stage!r}")
        return
    #Pickup from staging place in delivery
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("moveJ_deg", *params['twist_serve'])#run_skill("moveJ_deg", 31.983258, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", *params['pick'])#run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478)
    run_skill("moveEE", 0, 0, -140, 0, 0, 0)
    run_skill("set_gripper_position", 55, 125)
    run_skill("set_servo_timing", 0.20)
    run_skill("gotoJ_deg", *params['pick'])#run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478, 1.0, 0.2)
    run_skill("gotoJ_deg", *params['above_serve'])
    run_skill("gotoJ_deg", *params['serve'])
    run_skill("set_gripper_position", 55, 0)
    run_skill("set_servo_timing", 0.10)
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 140, 0, 0, 0)
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def get_machine_position():
    """Machine position updates here (when starting from home position)."""
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "portafilter_cleaner", 0.12)
    run_skill("get_machine_position", "portafilter_cleaner")
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", 0.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.12)
    run_skill("get_machine_position", "espresso_grinder")
    run_skill("gotoJ_deg", 7.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.12)
    run_skill("get_machine_position", "three_group_espresso")
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
PULL_ESPRESSO_PARAMS = {
    'port_1': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_1",
        'group_number':         "group_1",
        'below_port':   (23.433992,-34.688675,-112.293900,-33.728462,-24.236135,0.000000),
        'move_back':   (-5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815),
    },
    'port_2': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_2",
        'group_number':         "group_2",
        'below_port':   (48.987930,-42.926491,-94.531250,-42.794937,4.182595,0.000000),
        'move_back':   (88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554),
    },
    'port_3': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_3",
        'group_number':         "group_3",
        'below_port':   (63.598370,-58.609657,-61.130093,-60.226448,15.355281,0.000000),
        'move_back':   (88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554),
    },
}

def _prep_for_espresso():
    """Perform common refresh and initial positioning steps before grabbing a cup."""
    run_skill("refresh_position")
    time.sleep(2.0)

def unmount(port):
    """Unmount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return
    _prep_for_espresso()
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", *params['home']) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)  
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0)
    run_skill("gotoJ_deg", *params['below_port']) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", *params['move_back']) #A point in the plan chosen to avoid hitting anything in the path
    if port in ('port_2', 'port_3'):
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)

def grinder(port):
    """Grinder port"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return
    run_skill("gotoJ_deg", *Espresso_grinder_home) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True) 
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("approach_machine", "espresso_grinder", "tamper", True)
    time.sleep(1.0)
    run_skill("mount_machine", "espresso_grinder", "tamper", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", *Espresso_grinder_home)

def mount(port, skip_initial_move=False):
    """Mount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return

    if port in ('port_2', 'port_3'):
        if not skip_initial_move:
            run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

    run_skill("gotoJ_deg", *params['move_back'])  # A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", *params['below_port'])  # A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", params['group_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['group_number'], True)

    if port == 'port_1':
        run_skill("moveEE", 0, 0, 3.5, 0, 0, 0)
    elif port == 'port_2':
        run_skill("moveEE", 0, 0, 5, 0, 0, 0)
    elif port == 'port_3':
        run_skill("moveEE", 0, 0, 7, 0, 0, 0)

    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("move_portafilter_arc", 47)
    run_skill("set_gripper_position", 255, 0)  # Open gripper
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("gotoJ_deg", *params['home'])  # Espresso home


def pull_espresso(port):
    """
    Executes the full pull-espresso sequence on the given port:
    1) unmount → 2) grind & tamp → 3) remount.
    """
    # Validate port up front
    if port not in PULL_ESPRESSO_PARAMS:
        print(f"[ERROR] unknown port: {port!r}")
        return

    for fn in (unmount, grinder, mount):
        print(f"--- running {fn.__name__}({port}) ---")
        fn(port)

def pick_pitcher(port):
    """Pick pitcher"""
    #---Pick the pitcher---#
    run_skill("gotoJ_deg", *Espresso_home) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    if port == 'port_1':
        run_skill("moveEE", 0, -240, 0, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 255, 100)
        run_skill("moveEE", -103.413977, 0, 10, 0, 0, 0)
    elif port == 'port_2':
        run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
        run_skill("set_gripper_position", 255, 100)
        run_skill("gotoJ_deg", 23.034803,-44.195574,-116.188958,-19.410888,-66.975725,-0.169034, 1.0, 0.2)
    elif port == 'port_3':
        run_skill("moveEE", 0, 240, 0, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 255, 100)
        run_skill("moveEE", -103.413977, 0, 10, 0, 0, 0)
    else:
        print(f"[ERROR] unknown port: {port!r}")
        return
    run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)    

def pour_pitcher(stage):
    """
    Tilt the pitcher over the specified cup target.
    Cups must match keys in POUR_PARAMS: e.g. 'stage_1', 'stage_2'.
    """
    if stage not in ('stage_1', 'stage_2'):
        print(f"[ERROR] unknown stage: {stage!r}")
        return
    elif stage == 'stage_1':        
        run_skill("moveJ_deg", 90.160210, 10.716150, 0.203157, -10.883145, -0.001922, 0.060433, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 138.578024,-22.115324,-126.765855,-38.694157,-57.964712,3.173887, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 140.953509,-26.271451,-120.302336,-47.003274,-57.967889,-102.690158, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 138.578024,-22.115324,-126.765855,-38.694157,-57.964712,3.173887, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 121.236795,-29.537004,-136.110522,-14.093591,-58.933034,-0.146524)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)
    else:
        run_skill("moveJ_deg", 90.160210, 10.716150, 0.203157, -10.883145, -0.001922, 0.060433, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 145.885393,-26.409357,-118.242817,-43.978546,-58.850444,-0.109599, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 145.462132,-32.355488,-108.068238,-53.572020,-58.845487,-105.385536, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 145.885393,-26.409357,-118.242817,-43.978546,-58.850444,-0.109599, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 121.236795,-29.537004,-136.110522,-14.093591,-58.933034,-0.146524)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)

def hot_water():
    """Get hot water in pitcher"""
    run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)  
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("moveEE", 100, 0, 0, 0, 0, 0) #Move in
    time.sleep(8.0)
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -100, 0, 0, 0, 0, 0) #Move out
    run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)  

def return_pitcher(port):
    """Return pitcher to its home position."""
    #---Return the pitcher---#
    if port == 'port_1':
        run_skill("moveEE", 0, -240, 10, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 75, 0)
        run_skill("moveEE", -103.413977, 0, -10, 0, 0, 0)
    elif port == 'port_2':
        run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
        run_skill("set_gripper_position", 75, 0) #pitcher gripper grip
    elif port == 'port_3':
        run_skill("moveEE", 0, 240, 10, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 75, 0)
        run_skill("moveEE", -103.413977, 0, -10, 0, 0, 0)
    else:
        print(f"[ERROR] unknown port: {port!r}")
        return
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("gotoJ_deg", *Espresso_home) #Espresso home
    
CLEANER_PARAMS = {
    'port_1': {
        'grinder_home':    (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942,   9.602360),
        'rotate_amount':   122,
        'clean_point':     (110.132784,   7.214473, -140.810214, -44.785857, -70.153760,  -4.137882),
        'linear_push':     (   0,        150,         0,         0,         0,         0),
        'twist_around':      (   0,          0,         0,         0,         0,       -180),
        'clean_pattern': [
            (0, 0, -77.5, 0, 0, 0),
            (0, 0, 0, -1.25, 0, 0),
            (0, 0, 0, 1.25, 1.25, 0),
            (0, 0, 0, 0, -1.25, 1.25),
            (0, 0, 0, 0, -1.25, -1.25),
            (0, 0, 0, 0, 1.25, 0),
            (0, 0, 77.5, 0, 0, 0),
            (-88, 0, 0, 0, 0, 0),
            (0, 0, -77.5, 0, 0, 0),
            (0, 0, 0, -1.25, 0, 0),
            (0, 0, 0, 1.25, 1.25, 0),
            (0, 0, 0, 0, -1.25, 1.25),
            (0, 0, 0, 0, -1.25, -1.25),
            (0, 0, 0, 0, 1.25, 0),
            (0, 0, 77.5, 0, 0, 0),
        ],
        'twist_back':      (   0,          0,         0,         0,         0,       180),
        'return_home':     ( 89.162277, -2.957932, -128.257645, -89.085014, -79.229942,   9.602360),
        'final_offset':  (-122, 0, 0, 0, 0, 0),
    },
    'port_2': {
        'grinder_home':    (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942,   9.602360),
        'rotate_amount':   122,
        'clean_point':     (110.132784,   7.214473, -140.810214, -44.785857, -70.153760,  -4.137882),
        'linear_push':     (   0,        150,         0,         0,         0,         0),
        'twist_around':      (   0,          0,         0,         0,         0,       -180),
        'clean_pattern': [
            (0, 0, -77.5, 0, 0, 0),
            (0, 0, 0, -1.25, 0, 0),
            (0, 0, 0, 1.25, 1.25, 0),
            (0, 0, 0, 0, -1.25, 1.25),
            (0, 0, 0, 0, -1.25, -1.25),
            (0, 0, 0, 0, 1.25, 0),
            (0, 0, 77.5, 0, 0, 0),
            (-88, 0, 0, 0, 0, 0),
            (0, 0, -77.5, 0, 0, 0),
            (0, 0, 0, -1.25, 0, 0),
            (0, 0, 0, 1.25, 1.25, 0),
            (0, 0, 0, 0, -1.25, 1.25),
            (0, 0, 0, 0, -1.25, -1.25),
            (0, 0, 0, 0, 1.25, 0),
            (0, 0, 77.5, 0, 0, 0),
        ],
        'twist_back':      (   0,          0,         0,         0,         0,       180),
        'return_home':     ( 89.162277, -2.957932, -128.257645, -89.085014, -79.229942,   9.602360),
        'final_offset':  (-32, 0, 0, 0, 0, 0),
    },
    'port_3': {
        'grinder_home':    (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942,   9.602360),
        'rotate_amount':   122,
        'clean_point':     (110.132784,   7.214473, -140.810214, -44.785857, -70.153760,  -4.137882),
        'linear_push':     (   0,        150,         0,         0,         0,         0),
        'twist_around':      (   0,          0,         0,         0,         0,       -180),
        'clean_pattern': [
            (0, 0, -77.5, 0, 0, 0),
            (0, 0, 0, -1.25, 0, 0),
            (0, 0, 0, 1.25, 1.25, 0),
            (0, 0, 0, 0, -1.25, 1.25),
            (0, 0, 0, 0, -1.25, -1.25),
            (0, 0, 0, 0, 1.25, 0),
            (0, 0, 77.5, 0, 0, 0),
            (-88, 0, 0, 0, 0, 0),
            (0, 0, -77.5, 0, 0, 0),
            (0, 0, 0, -1.25, 0, 0),
            (0, 0, 0, 1.25, 1.25, 0),
            (0, 0, 0, 0, -1.25, 1.25),
            (0, 0, 0, 0, -1.25, -1.25),
            (0, 0, 0, 0, 1.25, 0),
            (0, 0, 77.5, 0, 0, 0),
        ],
        'twist_back':      (   0,          0,         0,         0,         0,       180),
        'return_home':     ( 89.162277, -2.957932, -128.257645, -89.085014, -79.229942,   9.602360),
        'final_offset':  (-32, 0, 0, 0, 0, 0),
    },
}

def clean(port):
    """
    Perform the cleaning sequence on the specified port.

    Args:
        port (str): 'port_1', 'port_2', or 'port_3'.
    """
    unmount(port)  # Unmount the portafilter if necessary
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", -88, 0, 0, 0, 0, -135)
    run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", 0, 0, 100, 0, 0, 0)
    run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0)
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    mount(port, skip_initial_move=False)

def random_tests():
    """STAGE 1 PORT 2"""
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.12)
    run_skill("get_machine_position", "three_group_espresso")
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    run_skill("gotoJ_deg", 31.073321,-45.397456,-135.760453,1.382899,-58.936363,-0.211361)
    run_skill("gotoJ_deg", 61.115904,-46.853232,-120.979236,-11.767174,-28.893833,-0.439449)
    run_skill("gotoJ_deg", 49.157661,-51.153538,-104.162788,-24.392244,-40.855900,-0.309051)
    run_skill("gotoJ_deg", 61.115904,-46.853232,-120.979236,-11.767174,-28.893833,-0.439449)
    run_skill("gotoJ_deg", 31.073321,-45.397456,-135.760453,1.382899,-58.936363,-0.211361)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    ###################################################################################################
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    run_skill("gotoJ_deg", 31.073321,-45.397456,-135.760453,1.382899,-58.936363,-0.211361)
    run_skill("gotoJ_deg", -11.120760,-45.804448,-127.729592,-6.277576,-101.131449,-0.057911)
    run_skill("gotoJ_deg", -8.430291,-49.447607,-110.024799,-20.339796,-98.443658,-0.063492)
    run_skill("gotoJ_deg", -11.120760,-45.804448,-127.729592,-6.277576,-101.131449,-0.057911)
    run_skill("gotoJ_deg", 31.073321,-45.397456,-135.760453,1.382899,-58.936363,-0.211361)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)  


def test(port):
    """Machine position updates here (when starting from home position)."""
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "portafilter_cleaner", 0.12)
    run_skill("get_machine_position", "portafilter_cleaner")
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", 0.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.12)
    run_skill("get_machine_position", "espresso_grinder")
    run_skill("gotoJ_deg", 7.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.12)
    run_skill("get_machine_position", "three_group_espresso")
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    """Unmount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return
    _prep_for_espresso()
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", *params['home']) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6) 
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0)
    run_skill("gotoJ_deg", *params['below_port']) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", *params['move_back']) #A point in the plan chosen to avoid hitting anything in the path
    if port in ('port_2', 'port_3'):
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    """Grinder port"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return
    run_skill("gotoJ_deg", *Espresso_grinder_home) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True) 
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("approach_machine", "espresso_grinder", "tamper", True)
    time.sleep(1.0)
    run_skill("mount_machine", "espresso_grinder", "tamper", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    """Mount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return

    if port in ('port_2', 'port_3'):
        run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", *params['move_back'])  # A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", *params['below_port'])  # A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", params['group_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['group_number'], True)

    if port == 'port_1':
        run_skill("moveEE", 0, 0, 3, 0, 0, 0)
    elif port == 'port_2':
        run_skill("moveEE", 0, 0, 5, 0, 0, 0)
    elif port == 'port_3':
        run_skill("moveEE", 0, 0, 7, 0, 0, 0)

    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("move_portafilter_arc", 47)
    run_skill("set_gripper_position", 255, 0)  # Open gripper
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("gotoJ_deg", *params['home'])  # Espresso home
    """Unmount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return
    _prep_for_espresso()
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", *params['home']) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6) 
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0)
    run_skill("gotoJ_deg", *params['below_port']) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", *params['move_back']) #A point in the plan chosen to avoid hitting anything in the path
    if port in ('port_2', 'port_3'):
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", -88, 0, 0, 0, 0, -135)
    run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", 0, 0, 100, 0, 0, 0)
    run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0)
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    """Mount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return

    if port in ('port_2', 'port_3'):
        run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", *params['move_back'])  # A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", *params['below_port'])  # A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", params['group_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['group_number'], True)
    if port == 'port_1':
        run_skill("moveEE", 0, 0, 3, 0, 0, 0)
    elif port == 'port_2':
        run_skill("moveEE", 0, 0, 5, 0, 0, 0)
    elif port == 'port_3':
        run_skill("moveEE", 0, 0, 7, 0, 0, 0)
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("move_portafilter_arc", 47)
    run_skill("set_gripper_position", 255, 0)  # Open gripper
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("gotoJ_deg", *params['home'])  # Espresso home
# Add more sequences here as plain Python functions…


# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    "home": home_north,
    "home_north_east": home_north_east,
    "home_west":home_west,
    "espresso_port_1": espresso_port_1,
    "espresso_port_2": espresso_port_2,
    "espresso_port_3": espresso_port_3,
    "americano_port_1": americano_port_1,
    "americano_port_2": americano_port_2,
    "americano_port_3": americano_port_3,
    "clean_port_1": clean_port_1,
    "clean_port_2": clean_port_2,
    "clean_port_3": clean_port_3,
    "milk_frothing": milk_frothing,
    "get_machine_position": get_machine_position,
    "random_tests": random_tests,
    "unmount_port_1": lambda: unmount("port_1"),
    "unmount_port_2": lambda: unmount("port_2"),
    "unmount_port_3": lambda: unmount("port_3"),
    "grinder_port": lambda: grinder("port_1"),
    "mount_port_1": lambda: mount("port_1"),
    "mount_port_2": lambda: mount("port_2"),
    "mount_port_3": lambda: mount("port_3"),
    "pick_pitcher_port_1": lambda: pick_pitcher("port_1"),
    "pick_pitcher_port_2": lambda: pick_pitcher("port_2"),
    "pick_pitcher_port_3": lambda: pick_pitcher("port_3"),
    "pour_pitcher_cup_1": lambda: pour_pitcher("stage_1"),
    "pour_pitcher_cup_2": lambda: pour_pitcher("stage_2"),
    "return_pitcher_port_1": lambda: return_pitcher("port_1"),
    "return_pitcher_port_2": lambda: return_pitcher("port_2"),
    "return_pitcher_port_3": lambda: return_pitcher("port_3"),
    "hot_water": hot_water,
    "test":lambda: test("port_3"),
    "grab_cup_1": lambda: grab_cup("12oz"),
    "grab_cup_2": lambda: grab_cup("9oz"),
    "grab_cup_3": lambda: grab_cup("7oz"),
    "place_cup_1": lambda: place_cup("stage_1"),
    "place_cup_2": lambda: place_cup("stage_2"),
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
