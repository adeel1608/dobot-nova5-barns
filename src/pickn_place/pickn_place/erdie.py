#!/usr/bin/env python3
import time
import argparse
from turtle import position
from manipulate_move_v3 import run_skill   # ← the only import you need
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

HOME_ANGLES = {
    'north':       (   0, 30, -130, -100,  -90,    0),
    'north_east': ( -45, 30, -130, -100,  -90,    0),
    'east':       ( -90, 30, -130, -100,  -90,    0),
    'south_east': (-135, 30, -130, -100,  -90,    0),
    'south':      ( 180, 30, -130, -100,  -90,    0),
    'south_west': ( 135, 30, -130, -100,  -90,    0),
    'west':       (  90, 30, -130, -100,  -90,    0),
    'north_west': (  45, 30, -130, -100,  -90,    0),
}

def home(**params):
    """
    Move the robot to one of the predefined home orientations.
    
    Args:
        position (str): One of 'north', 'north_east', 'east',
                        'south_east', 'south', 'south_west',
                        'west', 'north_west'.
    """
    position = params.get("position")
    angles = HOME_ANGLES.get(str(position))
    if not angles:
        print(f"[ERROR] unknown home position: {position!r}")
        return
    run_skill("gotoJ_deg", *angles)

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
        'grip_width':   150,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
    '7oz': {
        'twist_back':   ( -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037),
        'approach':     ( 260,         15,           -10,          0,          0,         0),
        'grip_width':   150,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
}

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

def grab_cup(**params):
    """
    Approaches and picks up the specified cup.
    
    Args:
        size (str): Cup size key, e.g. '7oz', '9oz', '12oz'.
    """
    size = params.get("size")
    cup_params = GRAB_CUP_PARAMS.get(str(size))
    if not cup_params:
        print(f"[ERROR] unknown cup size: {size!r}, using default 12oz")
        cup_params = GRAB_CUP_PARAMS.get("12oz")
        if not cup_params:
            print("[ERROR] Default 12oz parameters not found. Cannot proceed.")
            return
    
    run_skill("gotoJ_deg", *Espresso_home) #Espresso home
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chosen for grabbing the cup
    # Rotate joint angles to back away before approach
    run_skill("moveJ_deg", *cup_params['twist_back'])
    # Move end-effector into approach position
    run_skill("moveEE", *cup_params['approach'])
    # Close gripper to grasp the cup
    run_skill("set_gripper_position", 255, cup_params['grip_width'])
    # Retract after gripping
    run_skill("moveEE", *cup_params['retreat'])
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)

def place_cup(**params):
    """
    Places the currently held cup at the specified stage.
    
    Args:
        stage (str): Placement stage key, e.g. 'stage_1', 'stage_2'.
    """
    stage = params.get("stage")
    params = PLACE_CUP_PARAMS.get(str(stage))
    if not params:
        print(f"[ERROR] unknown stage: {stage!r}, using default stage_1")
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

def serve(**params):
    stage = params.get("stage")
    params = PLACE_CUP_PARAMS.get(str(stage))
    if not params:
        print(f"[ERROR] unknown stage {stage!r}, using default stage_1")
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

below_espresso_port = None
mount_espresso_port = None


def unmount(**params):
    """
    Unmount portafilter from espresso group for cleaning or grinding.
    
    This function performs the complete portafilter unmounting sequence:
    1. Moves to espresso home position
    2. Approaches and mounts to the specified portafilter group
    3. Closes gripper to secure portafilter
    4. Releases tension and adjusts orientation
    5. Rotates portafilter to unlock position
    6. Safely retracts and moves to clear path
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if portafilter unmounted successfully, False otherwise
        
    Example:
        success = unmount(port='port_1')
        if success:
            print("Portafilter unmounted successfully")
    """
    global below_espresso_port, mount_espresso_port
    try:
        port = params.get("port")
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] unknown port number: {port!r}, available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"📤 Starting portafilter unmount sequence for {port}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *port_params['home'])
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Approach the portafilter group
        print(f"🎯 Approaching portafilter {port_params['portafilter_number']}...")
        approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
        if approach_result is False:
            print("[ERROR] Failed to approach portafilter")
            return False
        
        # Step 3: Mount to the portafilter for secure grip
        print("🔧 Mounting to portafilter...")
        mount_result = run_skill("mount_machine", "three_group_espresso", port_params['portafilter_number'], True)
        if mount_result is False:
            print("[ERROR] Failed to mount to portafilter")
            return False
        
        # Step 4: Close gripper to secure portafilter
        print("🤏 Securing portafilter with gripper...")
        grip_result = run_skill("set_gripper_position", 255, 255)
        if grip_result is False:
            print("[ERROR] Failed to close gripper")
            return False
        
        # Step 5: Release tension for smooth operation
        print("😌 Releasing tension...")
        tension_result = run_skill("release_tension")
        if tension_result is False:
            print("[ERROR] Failed to release tension")
            return False
        
        # Step 6: Enforce proper orientation (first time)
        print("📐 Enforcing proper orientation...")
        orient_result1 = run_skill("enforce_rxry")
        if orient_result1 is False:
            print("[ERROR] Failed to enforce orientation (first attempt)")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 7: Enforce proper orientation (second time for stability)
        print("📐 Re-enforcing orientation for stability...")
        orient_result2 = run_skill("enforce_rxry")
        if orient_result2 is False:
            print("[ERROR] Failed to re-enforce orientation")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 8: Rotate portafilter to unlock (-45 degrees)
        print("🔄 Rotating portafilter to unlock...")
        rotate_result = run_skill("move_portafilter_arc", -45)
        if rotate_result is False:
            print("[ERROR] Failed to rotate portafilter")
            return False
        
        # Step 9: Release tension after rotation
        print("😌 Releasing tension after rotation...")
        tension_result2 = run_skill("release_tension")
        mount_espresso_port = run_skill("current_angles")  # Capture mount position
        if tension_result2 is False:
            print("[ERROR] Failed to release tension after rotation")
            return False
        
        # Step 10: Move end effector down to clear portafilter
        print("⬇️ Moving down to clear portafilter...")
        clear_result = run_skill("moveEE", 0, 0, -35, 0, 0, 0)
        below_espresso_port = run_skill("current_angles")  # Capture below position
        if clear_result is False:
            print("[ERROR] Failed to move down to clear portafilter")
            return False
        
        # Step 11: Move to position below port
        print("📍 Moving to position below port...")
        below_result = run_skill("gotoJ_deg", *port_params['below_port'])
        if below_result is False:
            print("[ERROR] Failed to move to position below port")
            return False
        
        # Step 12: Move back to avoid collisions
        print("⬅️ Moving back to avoid collisions...")
        back_result = run_skill("gotoJ_deg", *port_params['move_back'])
        if back_result is False:
            print("[ERROR] Failed to move back")
            return False
        
        # Step 13: Special handling for ports 2 and 3 (additional navigation)
        if port in ('port_2', 'port_3'):
            print("🔄 Executing special navigation for port 2/3...")
            nav1_result = run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
            if nav1_result is False:
                print("[ERROR] Failed special navigation step 1")
                return False
            
            nav2_result = run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
            if nav2_result is False:
                print("[ERROR] Failed special navigation step 2")
                return False
        
        print(f"✅ Portafilter unmount sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during unmount: {e}")
        return False

def grinder(**params):
    """
    Grind coffee and tamp portafilter at the grinder station.
    
    This function performs the complete grinding and tamping workflow:
    1. Moves to grinder home position
    2. Approaches grinder for coffee grinding
    3. Mounts to grinder to activate grinding
    4. Moves to tamper for coffee compaction
    5. Performs tamping motion
    6. Returns to grinder area and then home
    
    Args:
        port (str): Source port identifier (for parameter validation)
        
    Returns:
        bool: True if grinding and tamping completed successfully, False otherwise
        
    Example:
        success = grinder(port='port_1')
        if success:
            print("Coffee ground and tamped successfully")
    """
    try:
        port = params.get("port")
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] unknown port number: {port!r}, available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"☕ Starting grinding and tamping sequence for {port}")
        
        # Step 1: Move to grinder home position
        print("🏠 Moving to grinder home position...")
        home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if home_result is False:
            print("[ERROR] Failed to move to grinder home")
            return False
        
        # Step 2: Approach the grinder
        print("🎯 Approaching grinder...")
        approach_result = run_skill("approach_machine", "espresso_grinder", "grinder", True)
        if approach_result is False:
            print("[ERROR] Failed to approach grinder")
            return False
        
        # Step 3: Mount to grinder to activate grinding
        print("⚙️ Mounting to grinder for grinding...")
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder", True)
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        
        # Step 4: Approach tamper station
        print("🎯 Approaching tamper...")
        tamper_approach_result = run_skill("approach_machine", "espresso_grinder", "tamper", True)
        if tamper_approach_result is False:
            print("[ERROR] Failed to approach tamper")
            return False
        
        time.sleep(1.0)  # Allow positioning time
        
        # Step 5: Mount to tamper for positioning
        print("📍 Positioning at tamper...")
        tamper_mount_result = run_skill("mount_machine", "espresso_grinder", "tamper", True)
        if tamper_mount_result is False:
            print("[ERROR] Failed to mount to tamper")
            return False
        
        # Step 6: Move up to prepare for tamping
        print("⬆️ Moving up to prepare for tamping...")
        up_result = run_skill("moveEE", 0, 0, 45, 0, 0, 0)
        if up_result is False:
            print("[ERROR] Failed to move up for tamping")
            return False
        
        time.sleep(1.0)  # Allow settling time
        
        # Step 7: Perform tamping motion (move down)
        print("🔨 Performing tamping motion...")
        tamp_result = run_skill("moveEE", 0, 0, -95, 0, 0, 0)
        if tamp_result is False:
            print("[ERROR] Failed to perform tamping motion")
            return False
        
        # Step 8: Return to grinder area
        print("🔄 Returning to grinder area...")
        return_grinder_result = run_skill("approach_machine", "espresso_grinder", "grinder", True)
        if return_grinder_result is False:
            print("[ERROR] Failed to return to grinder area")
            return False
        
        # Step 9: Return to grinder home
        print("🏠 Returning to grinder home...")
        final_home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if final_home_result is False:
            print("[ERROR] Failed to return to grinder home")
            return False
        
        print(f"✅ Grinding and tamping sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during grinding: {e}")
        return False

def mount(**params):
    """
    Mount portafilter back to espresso group after grinding.
    
    This function performs the complete portafilter mounting sequence:
    1. Handles special navigation for ports 2 and 3
    2. Moves through safe path to target group
    3. Approaches and mounts to espresso group
    4. Adjusts position based on specific port requirements
    5. Performs orientation enforcement and locking rotation
    6. Opens gripper to release portafilter
    7. Returns to home position
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if portafilter mounted successfully, False otherwise
        
    Example:
        success = mount(port='port_1')
        if success:
            print("Portafilter mounted successfully")
    """
    try:
        port = params.get("port")
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] unknown port number: {port!r}, available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"📥 Starting portafilter mount sequence for {port}")
        
        # Step 1: Special handling for ports 2 and 3 (reverse navigation)
        if port in ('port_2', 'port_3'):
            print("🔄 Executing special navigation for port 2/3...")
            nav1_result = run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0)
            if nav1_result is False:
                print("[ERROR] Failed special navigation step 1")
                return False
            
            nav2_result = run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
            if nav2_result is False:
                print("[ERROR] Failed special navigation step 2")
                return False
        
        # Step 2: Move to safe path position
        print("📍 Moving to safe path position...")
        back_result = run_skill("gotoJ_deg", *port_params['move_back'])
        if back_result is False:
            print("[ERROR] Failed to move to safe path position")
            return False
        
        # Step 3: Move to position below port
        print("📍 Moving to position below port...")
        below_result = run_skill("gotoJ_deg", *port_params['below_port'])
        if below_result is False:
            print("[ERROR] Failed to move to position below port")
            return False
        
        # Step 4: Approach the espresso group
        print(f"🎯 Approaching espresso group {port_params['group_number']}...")
        if below_espresso_port is None:
            print("[ERROR] below_espresso_port not captured, run unmount first")
            return False
        approach_result = run_skill("gotoJ_deg", *below_espresso_port)  # Use captured below position
        if approach_result is False:
            print("[ERROR] Failed to approach espresso group")
            return False
        
        # Step 5: Mount to espresso group
        print("🔧 Mounting to espresso group...")
        if mount_espresso_port is None:
            print("[ERROR] mount_espresso_port not captured, run unmount first")
            return False
        mount_result = run_skill("gotoJ_deg", *mount_espresso_port)  # Use captured mount position
        if mount_result is False:
            print("[ERROR] Failed to mount to espresso group")
            return False
        
        # Step 6: Adjust position based on specific port (fine-tuning)
        print(f"📐 Adjusting position for {port}...")
        if port == 'port_1':
            adjust_result = run_skill("moveEE", 0, 0, 3.5, 0, 0, 0)
        elif port == 'port_2':
            adjust_result = run_skill("moveEE", 0, 0, 5, 0, 0, 0)
        elif port == 'port_3':
            adjust_result = run_skill("moveEE", 0, 0, 7, 0, 0, 0)
        else:
            adjust_result = True  # No adjustment needed
        
        if adjust_result is False:
            print(f"[ERROR] Failed to adjust position for {port}")
            return False
        
        # Step 7: Release tension for smooth operation
        print("😌 Releasing tension...")
        tension_result = run_skill("release_tension")
        if tension_result is False:
            print("[ERROR] Failed to release tension")
            return False
        
        # Step 8: Enforce proper orientation (first time)
        print("📐 Enforcing proper orientation...")
        orient_result1 = run_skill("enforce_rxry")
        if orient_result1 is False:
            print("[ERROR] Failed to enforce orientation (first attempt)")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 9: Enforce proper orientation (second time for stability)
        print("📐 Re-enforcing orientation for stability...")
        orient_result2 = run_skill("enforce_rxry")
        if orient_result2 is False:
            print("[ERROR] Failed to re-enforce orientation")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 10: Rotate portafilter to lock position (+47 degrees)
        print("🔄 Rotating portafilter to lock...")
        rotate_result = run_skill("move_portafilter_arc", 47)
        if rotate_result is False:
            print("[ERROR] Failed to rotate portafilter to lock")
            return False
        
        # Step 11: Open gripper to release portafilter
        print("🤏 Opening gripper to release portafilter...")
        release_result = run_skill("set_gripper_position", 255, 0)
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 12: Move back to portafilter approach position
        print("⬅️ Moving back from portafilter...")
        back_approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
        if back_approach_result is False:
            print("[ERROR] Failed to move back from portafilter")
            return False
        
        # Step 13: Return to espresso home
        print("🏠 Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *port_params['home'])
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Portafilter mount sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during mount: {e}")
        return False

def pick_pitcher(**params):
    """Pick pitcher"""
    port = params.get("port")
    #---Pick the pitcher---#
    run_skill("gotoJ_deg", *Espresso_home) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    if port == 'port_1':
        run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
        run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
        run_skill("set_gripper_position", 255, 100)
        run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
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
        print(f"[ERROR] unknown port: {port!r}, using default port_1")
        return
    run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)    

def pour_pitcher(**params):
    """
    Tilt the pitcher over the specified cup target.
    Cups must match keys in POUR_PARAMS: e.g. 'stage_1', 'stage_2'.
    """
    stage = params.get("stage")
    if stage not in ('stage_1', 'stage_2'):
        print(f"[ERROR] unknown stage: {stage!r}, using default stage_1")
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

def hot_water(**params):
    """Get hot water in pitcher"""
    # run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957)  
    run_skill("gotoJ_deg", 67.341492,-49.798077,-99.061142,-30.809935,-22.613216,-0.457894) 
    run_skill("gotoJ_deg", 61.759601,-52.680407,-91.236745,-35.814578,-28.197699,-0.388979)
    time.sleep(8.0)
    run_skill("gotoJ_deg", 67.341492,-49.798077,-99.061142,-30.809935,-22.613216,-0.457894, 1.0, 0.2) 
    run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)  

def return_pitcher(**params):
    """Return pitcher to its home position."""
    #---Return the pitcher---#
    port = params.get("port")
    if port == 'port_1':
        run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
        run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
        run_skill("set_gripper_position", 75, 0)
        run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
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

def clean(**params):
    """
    Perform the cleaning sequence on the specified port.

    Args:
        port (str): 'port_1', 'port_2', or 'port_3'.
    """
    port = params.get("port")
    
    # Validate port parameter
    if not port or port not in ('port_1', 'port_2', 'port_3'):
        print(f"[ERROR] invalid port: {port!r}, must be 'port_1', 'port_2', or 'port_3'")
        return
    
    # Unmount the portafilter if necessary
    unmount(port=port)  
    
    # Move to cleaning station
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", -88, 0, 0, 0, 0, -135)
    run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", 0, 0, 100, 0, 0, 0)
    run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0)
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    
    # Mount the portafilter back
    mount(port=port)

def drag(**params):
    run_skill("toggle_drag_mode")
    run_skill("set_DO", 2, 1)
    time.sleep(5)
    run_skill("set_DO", 2, 0)
    run_skill("set_DO", 1, 1)
    run_skill("set_DO", 2, 1)
    time.sleep(4.5)
    run_skill("set_DO", 2, 1)
    run_skill("set_DO", 1, 0)
    time.sleep(2)
    run_skill("set_DO", 2, 0)
    run_skill("toggle_drag_mode")

def cold_cup(**params):
    cup_size = params.get("cup_size")
    if cup_size not in ('16oz', '12oz', '9oz', '7oz'):
        print(f"[ERROR] unknown cup size: {cup_size!r}, using default cup_size")
        return
    home(position="west")
    if cup_size == '7oz':
        run_skill("set_gripper_position", 255, 0)
        run_skill("gotoJ_deg", 137.406860,3.501065,-134.504471,-48.814426,-42.387501,-0.108438)
        run_skill("moveEE", 5, 210, 10, 0, 0, 0)
        run_skill("set_gripper_position", 255, 140)
        run_skill("moveEE", 0, 0, -205, 0, 0, 0)
        run_skill("gotoJ_deg", 137.406860,3.501065,-134.504471,-48.814426,-42.387501,-0.108438)
    # elif cup_size == '9oz':
    #     run_skill("gotoJ_deg", 0, 0, 0, 0, 0, 0)
    # elif cup_size == '12oz':
    #     run_skill("gotoJ_deg", 0, 0, 0, 0, 0, 0)
    # elif cup_size == '16oz':
    #     run_skill("set_gripper_position", 255, 0)
    #     run_skill("gotoJ_deg", 142.686200,-21.359396,-113.896270,-44.413189,-37.200442,-0.267903)
    #     #-220.0,425.0,260.0,90.0,0.0,-180.0
    #     run_skill("moveEE", 0, 50, 0, 0, 0, 0)
    #     run_skill("set_gripper_position", 255, 130)
    #     run_skill("moveJ_deg", 0, 0, 0, 0, 0, 2)
    #     run_skill("moveJ_deg", 0, 0, 0, 0, 0, -2)
    #     run_skill("moveEE", 0, 0, -200, 0, 0, 0)
    #     run_skill("moveEE", 0, -150, 0, 0, 0, 0)

def place_cold_cup(**params):
    cold_stage = params.get("cold_stage")
    if cold_stage not in ('1', '2'):
        print(f"[ERROR] unknown cold stage: {cold_stage!r}, using default cold_stage")
        return
    if cold_stage == '1':
        run_skill("gotoJ_deg", -111.215927,-19.601524,-91.144157,-68.881447,-114.195343,0.046140)
        #-281.837637,-453.505297,381.885287,89.857537,0.113032,3.075480
        run_skill("moveEE",0,0,-315,0,0,0)
        run_skill("set_gripper_position", 60, 0)
        time.sleep(0.5)
        run_skill("moveEE",0,0,315,0,0,0)
        home(position="east")
    elif cold_stage == '2':
        run_skill("gotoJ_deg", -121.922080,-29.170114,-76.511387,-73.910323,-124.911454,0.116947)
        #-381.837637,-453.505297,381.885287,89.857537,0.113032,3.075480
        run_skill("moveEE",0,0,-315,0,0,0)
        run_skill("set_gripper_position", 60, 0)
        time.sleep(0.5)
        run_skill("moveEE",0,0,315,0,0,0)

def slush(**params):
    dispence = params.get("dispence")
    if dispence not in ('1', '2'):
        print(f"[ERROR] unknown dispence: {dispence!r}, using default dispence")
        return
    if dispence == '1':
        run_skill("gotoJ_deg", 27.351225,-41.244622,-131.515709,-6.958192,-152.611468,0.0)
        run_skill("gotoJ_deg", 63.173412,-59.329338,-104.639252,-17.729839,-112.244003,0.0, 1.0, 0.2)
        time.sleep(0.2)
        run_skill("gotoJ_deg", 27.351225,-41.244622,-131.515709,-6.958192,-152.611468,0.0, 1.0, 0.2)
        time.sleep(0.2)
    elif dispence == '2':
        run_skill("gotoJ_deg", 27.351225,-41.244622,-131.515709,-6.958192,-152.611468,0.0)
        run_skill("gotoJ_deg", 16.886827,-53.178346,-90.576631,-35.737992,-163.024408,0.0)
        run_skill("gotoJ_deg", 45.045738,-70.610674,-67.497199,-43.970287,-130.385694,-0.683135, 1.0, 0.2)
        time.sleep(0.2)
        run_skill("gotoJ_deg", 16.886827,-53.178346,-90.576631,-35.737992,-163.024408,0.0, 1.0, 0.2)
        time.sleep(0.2)

def milk_frother(**params):
    stage = params.get("stage")
    if stage not in ('1', '2'):
        print(f"[ERROR] unknown stage: {stage!r}, using default stage")
        return
    home(position="north_east")
    run_skill("set_gripper_position", 255, 0)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "left_steam_wand", 0.15,-10,-10)
    run_skill("get_machine_position", "left_steam_wand")
    home(position="north")
    run_skill("gotoJ_deg", 20.847986,-21.981329,-113.153931,-76.829208,-81.786911,-0.050592)
    run_skill("move_to", 'milk_frother_1', 0.175)
    run_skill("approach_tool", 'milk_frother_1', 160)
    approach = run_skill("current_angles")
    if approach is None:
        print("ERROR: Failed to get current angles for approach position")
        return
    print(approach)
    run_skill("grab_tool", 'milk_frother_1', 200,250,255)
    grab = run_skill("current_angles")
    if grab is None:
        print("ERROR: Failed to get current angles for grab position")
        return
    print(grab)
    run_skill("gotoJ_deg", -4.127179,-41.282722,-129.513504,-21.285969,-62.760456,7.227837,1.0,0.2)
    time.sleep(0.2)
    run_skill("set_servo_timing", 0.2)
    run_skill("approach_machine","left_steam_wand","milk_frother")#run_skill("gotoJ_deg", -44.670727,-76.199097,-35.181183,-81.723396,-66.913055,7.227651,1.0,0.2)
    time.sleep(0.2)
    run_skill("mount_machine","left_steam_wand","milk_frother")#run_skill("gotoJ_deg", -46.313637,-73.114960,-35.122364,-67.831657,-69.181740,7.227479,1.0,0.2)
    time.sleep(0.2)
    run_skill("set_DO", 2, 1)
    time.sleep(10)
    run_skill("set_DO", 2, 0)
    time.sleep(2)
    run_skill("approach_machine","left_steam_wand","milk_frother")#run_skill("gotoJ_deg", -44.670727,-76.199097,-35.181183,-81.723396,-66.913055,7.227651,1.0,0.2)
    run_skill("set_servo_timing", 0.1)
    time.sleep(0.2)
    run_skill("gotoJ_deg", -53.498047,-56.063831,-104.329971,-23.914228,-67.359390,3.238193,1.0,0.2)
    time.sleep(0.2)
    if stage == '1':
        run_skill("gotoJ_deg", -117.542499,-27.877248,-91.553736,-69.510481,-86.519990,1.649929,1.0,0.2)
        #-375.511117,-417.178650,300.667141,99.121753,-1.095762,-31.143898
        time.sleep(0.2)
        run_skill("gotoJ_deg", -102.517232,-32.497384,-90.464555,-66.071945,-85.480721,-96.398044,1.0,0.075)
        #-259.762839,-516.626886,266.926627,-142.650683,78.395595,110.925503
        time.sleep(2)
        run_skill("gotoJ_deg", -117.542499,-27.877248,-91.553736,-69.510481,-86.519990,1.649929,1.0,0.2)
        time.sleep(0.2)
    elif stage == '2':
        run_skill("gotoJ_deg", -127.319954,-37.857658,-74.250511,-76.871681,-96.185387,0.116908,1.0,0.2)
        time.sleep(0.2)
        run_skill("gotoJ_deg", -113.384514,-39.535606,-77.602524,-71.924614,-96.217064,-98.112167,1.0,0.075)
        time.sleep(2)
        run_skill("gotoJ_deg", -127.319954,-37.857658,-74.250511,-76.871681,-96.185387,0.116908,1.0,0.2)
        time.sleep(0.2)
    run_skill("gotoJ_deg", -4.127179,-41.282722,-129.513504,-21.285969,-62.760456,7.227837)
    time.sleep(0.2)
    run_skill("gotoJ_deg", 22.345373,-76.252151,-61.342220,-40.423759,-81.360077,11.115391)
    run_skill("gotoJ_deg", *grab)
    run_skill("set_gripper_position", 255, 160)
    run_skill("gotoJ_deg", *approach)
    home(position="north")
    run_skill("set_gripper_position", 255, 0)

def train(**params):
    run_skill("gotoJ_deg", *Espresso_home)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.12)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *Espresso_home)
    input()
    run_skill("gotoJ_deg", -11.610827,-11.667994,-134.749146,-39.220409,-95.688553,-1.029632)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -15.749183,-30.640251,-115.408752,-31.298525,-106.758682,0.620357)#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -11.610827,-11.667994,-134.749146,-39.220409,-95.688553,-1.029632)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", *Espresso_home)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("gotoJ_deg", 25.347992,-23.801352,-130.856094,-22.573463,-63.180656,-2.730590)#run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("gotoJ_deg", *Espresso_home)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("gotoJ_deg", 64.587212,-9.355711,-127.604965,-56.454102,-38.756008,21.765881)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 54.096390,-28.545734,-117.903122,-32.409134,-37.225636,0.833876)#run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 64.587212,-9.355711,-127.604965,-56.454102,-38.756008,21.765881)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", *Espresso_home)
    input()
    run_skill("gotoJ_deg",26.242167,-40.239166,-138.514206,-0.339417,-64.602829,0.470347)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    input()
    run_skill("gotoJ_deg",18.566435,-46.565922,-115.860474,-16.717270,-72.180389,0.604653)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    input()
    run_skill("gotoJ_deg",26.242167,-40.239166,-138.514206,-0.339417,-64.602829,0.470347)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    input()
    run_skill("gotoJ_deg",-16.759531,-41.994652,-121.946518,-16.669151,-109.865021,0.355781)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    input()
    run_skill("gotoJ_deg",-12.652535,-50.013134,-102.164963,-28.419651,-105.758224,0.396221)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
    input()
    run_skill("gotoJ_deg",-16.759531,-41.994652,-121.946518,-16.669151,-109.865021,0.355781)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    input()
    run_skill("gotoJ_deg",26.242167,-40.239166,-138.514206,-0.339417,-64.602829,0.470347)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    input()
    run_skill("gotoJ_deg",60.302876,-40.935081,-120.168037,-24.564249,-28.837940,4.383579)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
    input()
    run_skill("gotoJ_deg",47.204903,-49.090569,-101.349022,-33.651352,-41.890091,2.520284)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3", True)
    input()
    run_skill("gotoJ_deg",60.302876,-40.935081,-120.168037,-24.564249,-28.837940,4.383579)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
    input()
    run_skill("gotoJ_deg",26.242167,-40.239166,-138.514206,-0.339417,-64.602829,0.470347)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    input()
    run_skill("gotoJ_deg", *Espresso_home)
    input()
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    input()
    run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.12)
    run_skill("get_machine_position", "espresso_grinder")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", -33.121265,-44.552822,-107.645233,-56.071480,-84.332321,0.521377)#run_skill("approach_machine", "espresso_grinder", "grinder", True)
    input()
    run_skill("gotoJ_deg", -41.074799,-63.388416,-93.687790,-25.042332,-102.817711,0.539738)#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    input()
    run_skill("gotoJ_deg", -40.242844,-68.110458,-83.789452,-27.348541,-102.431313,0.539642)#run_skill("approach_machine", "espresso_grinder", "tamper", True)
    input()
    run_skill("gotoJ_deg",-40.966518,-67.185211,-91.374519,-19.000040,-102.846008,0.401071)#run_skill("mount_machine", "espresso_grinder", "tamper", True)
    input()
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "portafilter_cleaner", 0.12)
    run_skill("get_machine_position", "portafilter_cleaner")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
    run_skill("gotoJ_deg", -60.447098,5.900374,-126.604424,-58.288120,-61.421124,0.000000)# run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    input()
    run_skill("moveEE", -88, 0, 0, 0, 0, -135)
    input()
    run_skill("gotoJ_deg", -84.207855,-21.851009,-136.895401,-17.596369,-86.559967,-183.362045)#run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
    input()
    run_skill("moveEE", 0, 0, 100, 0, 0, 0)
    input()
    run_skill("gotoJ_deg", -66.577744,-0.893567,-131.774734,-45.521717,-70.645752,-178.476501)# run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
    input()
    run_skill("gotoJ_deg", -66.577751,-17.761707,-139.629929,-20.798386,-70.645760,-178.476501)#run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
    input()
    run_skill("moveEE", 0, 0, 150, 0, 0, 0)
    input()
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    input()
    run_skill("gotoJ_deg", *Espresso_home)
    


def test(**params):
    # run_skill("set_gripper_position", 255, 0)
    # run_skill("gotoJ_deg", -115.464302,-10.409121,-97.179970,-121.420769,-83.191971,2.868970)
    # run_skill("move_to", "milk_frother_1", 0.175)
    # run_skill("approach_tool", "milk_frother_1", 169)
    # approach = run_skill("current_angles")
    # print("approach angles", approach)
    # run_skill("grab_tool", "milk_frother_1", 200,250,255)
    # grab = run_skill("current_angles")
    # print("grab angles", grab)
    # run_skill("moveEE", 0, 0, 100, 0, 0, 0)
    # time.sleep(2)
    # run_skill("gotoJ_deg", *grab,1.0,0.2)
    # run_skill("moveEE", 0, 0, 2.5, 0, 0, 0)
    # time.sleep(0.2)
    # run_skill("set_gripper_position", 255, 169)
    # run_skill("gotoJ_deg", *approach,1.0,0.2)
    # time.sleep(0.2)
    # run_skill("gotoJ_deg", -115.464302,-10.409121,-97.179970,-121.420769,-83.191971,2.868970)
    run_skill("set_gripper_position", 255, 0)
    run_skill("gotoJ_deg", -115.464302,-10.409121,-97.179970,-121.420769,-83.191971,2.868970)
    # run_skill("move_to", "milk_frother_1", 0.175)
    run_skill("gotoJ_deg", -120.88261794420542, -35.18659068805018, -120.53362861519973, -34.0311434580501, -87.37802323053623, 12.24104768535856)
    run_skill("set_gripper_position", 255, 169)
    # approach = run_skill("current_angles")
    # print("approach angles", approach)
    run_skill("gotoJ_deg", -120.49650428863256, -51.27294207806112, -105.98161382415088, -19.964180464405686, -85.63502723285406, 11.09698500372573)
    run_skill("set_gripper_position", 255, 255)
    # grab = run_skill("current_angles")
    # print("grab angles", grab)
    run_skill("moveEE", 0, 0, 100, 0, 0, 0)
    time.sleep(2)
    run_skill("gotoJ_deg", -120.49650428863256, -51.27294207806112, -105.98161382415088, -19.964180464405686, -85.63502723285406, 11.09698500372573,1.0,0.2)
    run_skill("moveEE", 0, 0, 2.5, 0, 0, 0)
    time.sleep(0.2)
    run_skill("set_gripper_position", 255, 169)
    run_skill("gotoJ_deg", -120.88261794420542, -35.18659068805018, -120.53362861519973, -34.0311434580501, -87.37802323053623, 12.24104768535856,1.0,0.2)
    time.sleep(0.2)
    run_skill("gotoJ_deg", -115.464302,-10.409121,-97.179970,-121.420769,-83.191971,2.868970)





# Add more sequences here as plain Python functions…


# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    "train": lambda: train(),
    "test": lambda: test(),
    "say_hi": say_hi,

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

