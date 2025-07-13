#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from dobot_msgs_v3.srv import *
from .dobot_api import *
import os

class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)   
        self.IP = str(os.getenv("IP_address"))
        self.get_logger().info(self.IP) 
        self.srv = self.create_service(AccJ,'/dobot_bringup_v3/srv/AccJ',self.AccJ)
        self.srv = self.create_service(AccL,'/dobot_bringup_v3/srv/AccL',self.AccL)
        self.srv = self.create_service(Arch,'/dobot_bringup_v3/srv/Arch',self.Arch)
        self.srv = self.create_service(CP,'/dobot_bringup_v3/srv/CP',self.CP)
        self.srv = self.create_service(ClearError,'/dobot_bringup_v3/srv/ClearError',self.ClearError)
        self.srv = self.create_service(DI,'/dobot_bringup_v3/srv/DI',self.DI)
        self.srv = self.create_service(DO,'/dobot_bringup_v3/srv/DO',self.DO)
        self.srv = self.create_service(DOExecute,'/dobot_bringup_v3/srv/DOExecute',self.DOExecute)
        self.srv = self.create_service(DOGroup,'/dobot_bringup_v3/srv/DOGroup',self.DOGroup)
        self.srv = self.create_service(DisableRobot,'/dobot_bringup_v3/srv/DisableRobot',self.DisableRobot)
        self.srv = self.create_service(EnableRobot,'/dobot_bringup_v3/srv/EnableRobot',self.EnableRobot)
        self.srv = self.create_service(GetAngle,'/dobot_bringup_v3/srv/GetAngle',self.GetAngle)
        self.srv = self.create_service(GetCoils,'/dobot_bringup_v3/srv/GetCoils',self.GetCoils)
        self.srv = self.create_service(GetErrorID,'/dobot_bringup_v3/srv/GetErrorID',self.GetErrorID)
        self.srv = self.create_service(GetHoldRegs,'/dobot_bringup_v3/srv/GetHoldRegs',self.GetHoldRegs)
        self.srv = self.create_service(GetInBits,'/dobot_bringup_v3/srv/GetInBits',self.GetInBits)
        self.srv = self.create_service(GetInRegs,'/dobot_bringup_v3/srv/GetInRegs',self.GetInRegs)
        self.srv = self.create_service(GetPose,'/dobot_bringup_v3/srv/GetPose',self.GetPose)
        self.srv = self.create_service(ModbusClose,'/dobot_bringup_v3/srv/ModbusClose',self.ModbusClose)
        self.srv = self.create_service(ModbusCreate,'/dobot_bringup_v3/srv/ModbusCreate',self.ModbusCreate)
        self.srv = self.create_service(PayLoad,'/dobot_bringup_v3/srv/PayLoad',self.PayLoad)
        self.srv = self.create_service(ResetRobot,'/dobot_bringup_v3/srv/ResetRobot',self.ResetRobot)
        self.srv = self.create_service(RobotMode,'/dobot_bringup_v3/srv/RobotMode',self.RobotMode)
        self.srv = self.create_service(SetCoils,'/dobot_bringup_v3/srv/SetCoils',self.SetCoils)
        self.srv = self.create_service(SetHoldRegs,'/dobot_bringup_v3/srv/SetHoldRegs',self.SetHoldRegs)
        self.srv = self.create_service(SetPayload,'/dobot_bringup_v3/srv/SetPayload',self.SetPayload)
        self.srv = self.create_service(SpeedFactor,'/dobot_bringup_v3/srv/SpeedFactor',self.SpeedFactor)
        self.srv = self.create_service(SpeedJ,'/dobot_bringup_v3/srv/SpeedJ',self.SpeedJ)
        self.srv = self.create_service(SpeedL,'/dobot_bringup_v3/srv/SpeedL',self.SpeedL)
        self.srv = self.create_service(Tool,'/dobot_bringup_v3/srv/Tool',self.Tool)
        self.srv = self.create_service(ToolDI,'/dobot_bringup_v3/srv/ToolDI',self.ToolDI)
        self.srv = self.create_service(ToolDO,'/dobot_bringup_v3/srv/ToolDO',self.ToolDO)
        self.srv = self.create_service(ToolDOExecute,'/dobot_bringup_v3/srv/ToolDOExecute',self.ToolDOExecute)
        self.srv = self.create_service(User,'/dobot_bringup_v3/srv/User',self.User)
        self.srv = self.create_service(JointMovJ,'/dobot_bringup_v3/srv/JointMovJ',self.JointMovJ)
        self.srv = self.create_service(MovJ,'/dobot_bringup_v3/srv/MovJ',self.MovJ)
        self.srv = self.create_service(MovJIO,'/dobot_bringup_v3/srv/MovJIO',self.MovJIO)
        self.srv = self.create_service(MovL,'/dobot_bringup_v3/srv/MovL',self.MovL)
        self.srv = self.create_service(ServoJ,'/dobot_bringup_v3/srv/ServoJ',self.ServoJ)
        self.srv = self.create_service(ServoP,'/dobot_bringup_v3/srv/ServoP',self.ServoP)
        self.srv = self.create_service(MovLIO,'/dobot_bringup_v3/srv/MovLIO',self.MovLIO)
        self.srv = self.create_service(MoveJog,'/dobot_bringup_v3/srv/MoveJog',self.MoveJog)
        self.srv = self.create_service(RelMovJ,'/dobot_bringup_v3/srv/RelMovJ',self.RelMovJ)
        self.srv = self.create_service(RelMovL,'/dobot_bringup_v3/srv/RelMovL',self.RelMovL)
        self.srv = self.create_service(Sync,'/dobot_bringup_v3/srv/Sync',self.Sync)
        self.srv = self.create_service(StartDrag, '/dobot_bringup_v3/srv/StartDrag', self.StartDrag)
        self.srv = self.create_service(StopDrag, '/dobot_bringup_v3/srv/StopDrag', self.StopDrag)
        self.srv = self.create_service(SetGripperPosition, '/dobot_bringup_v3/srv/SetGripperPosition', self.SetGripperPosition)
        self.srv = self.create_service(GetGripperPosition, '/dobot_bringup_v3/srv/GetGripperPosition', self.GetGripperPosition)
        self.srv = self.create_service(InverseSolution,'/dobot_bringup_v3/srv/InverseSolution',self.InverseSolution)
        self.srv = self.create_service(PositiveSolution,'/dobot_bringup_v3/srv/PositiveSolution',self.PositiveSolution)
        self.connect() 

    def connect(self):
        try:
           self.get_logger().info("connection:29999")
           self.get_logger().info("connection:30003")
           self.dashboard = DobotApiDashboard(self.IP, 29999)
           self.move = DobotApiMove(self.IP,30003)
           self.get_logger().info("connection succeeded:29999,30003")
        except:
            self.get_logger().info("Connection failed!!!")

    def EnableRobot(self, request, response):                                           
        return_t = self.dashboard.EnableRobot([request.load])
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                        
        return response 
    
    def ClearError(self, request, response):                                          
        return_t = self.dashboard.ClearError()
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                        
        return response 
    
    def ResetRobot(self, request, response):                                          
        return_t = self.dashboard.ResetRobot()
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                        
        return response 
    
    def PayLoad(self, request, response):                                          
        return_t = self.dashboard.PayLoad(request.weight,request.inertia)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                        
        self.get_logger().info(return_t)                                        
        return response 
    
    def SetPayload(self, request, response):                                          
        return_t = self.dashboard.SetPayload(request.weight,request.inertia)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                         
        self.get_logger().info(return_t)                                        
        return response 
    
    def GetPose(self, request, response):                                          
        return_t = self.dashboard.GetPose(request.user,request.tool)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)   
        response.pose = return_t[return_t.find("{"):return_t.find("}")+1]                                         
        self.get_logger().info(return_t)                                        
        return response 
    
    def GetAngle(self, request, response):                                           
        return_t = self.dashboard.GetAngle()
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.angle = return_t[return_t.find("{"):return_t.find("}")+1]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def RobotMode(self, request, response):                                          
        return_t = self.dashboard.RobotMode()
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.mode = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ModbusCreate(self, request, response):                                           
        return_t = self.dashboard.ModbusCreate(request.ip,request.port,request.slave_id,request.is_rtu)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.index = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetInBits(self, request, response):                                           
        return_t = self.dashboard.GetInBits(request.index,request.addr,request.count)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.value = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetInRegs(self, request, response):                                          
        return_t = self.dashboard.GetInRegs(request.index,request.addr,request.count,request.val_type)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.value = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetHoldRegs(self, request, response):
        """
        Handles reading Modbus holding registers.
        """
        # Log the request parameters
        self.get_logger().info(
            f"GetHoldRegs called with: index={request.index}, addr={request.addr}, count={request.count}, val_type={request.val_type}"
        )
        
        # Call the dashboard's GetHoldRegs method
        return_t = self.dashboard.GetHoldRegs(
            request.index, request.addr, request.count, request.val_type
        )
        
        # Parse the response
        try:
            return_tt = return_t[:return_t.find("{") - 1]
            response.res = int(return_tt)
            response.value = return_t[return_t.find("{") + 1 : return_t.find("}")]
        except Exception as e:
            self.get_logger().error(f"Error parsing GetHoldRegs response: {e}")
            response.res = -1
            response.value = ""
        
        # Log the full response for debugging
        self.get_logger().info(f"GetHoldRegs response: {return_t}")
        return response

    def GetCoils(self, request, response):                                          
        return_t = self.dashboard.GetCoils(request.index,request.addr,request.count)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.value = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SetCoils(self, request, response):                                          
        return_t = self.dashboard.SetCoils(request.index,request.addr,request.count,request.val_tab)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                        
        self.get_logger().info(return_t)                                     
        return response 
    
    def SetHoldRegs(self, request, response):
        """
        Handles writing to Modbus holding registers.
        """
        # Log the request parameters
        self.get_logger().info(
            f"SetHoldRegs called with: index={request.index}, addr={request.addr}, count={request.count}, val_tab={request.val_tab}, val_type={request.val_type}"
        )
        
        # Call the dashboard's SetHoldRegs method
        return_t = self.dashboard.SetHoldRegs(
            request.index, request.addr, request.count, request.val_tab, request.val_type
        )
        
        # Parse the response
        try:
            return_tt = return_t[:return_t.find("{") - 1]
            response.res = int(return_tt)
        except Exception as e:
            self.get_logger().error(f"Error parsing SetHoldRegs response: {e}")
            response.res = -1
        
        # Log the full response for debugging
        self.get_logger().info(f"SetHoldRegs response: {return_t}")
        return response
        
    def GetGripperPosition(self, request, response):
        """
        Retrieves the gripper position from the high byte of Register 2002 and responds with the value.
        """
        try:
            # Read Register 2002 (1 register) using GetHoldRegs
            return_t = self.dashboard.GetHoldRegs(
                request.index, 2002, 1, "U16"
            )

            # Parse response
            return_tt = return_t[:return_t.find("{") - 1]
            reg_values = return_t[return_t.find("{") + 1: return_t.find("}")].split(",")

            # Debugging log for raw register values
            self.get_logger().info(f"GetGripperPosition: Raw register value from 2002: {reg_values}")

            # Decode the high byte of Register 2002 for the gripper position
            reg_2002 = int(reg_values[0])  # Full 16-bit value from Register 2002
            position = (reg_2002 >> 8) & 0xFF  # Extract the high byte (most significant byte)

            # Log the decoded position
            self.get_logger().info(f"Decoded gripper position (high byte of 2002): {position}")

            # Populate response
            response.position = position
        except Exception as e:
            # Handle any exceptions and populate error response
            response.position = 0
            self.get_logger().error(f"Error in GetGripperPosition: {e}")
        return response

    def SetGripperPosition(self, request, response):
        """
        Directly handles the SetGripperPosition service call.
        Writes values for action request, position (low byte only), speed, and force to the gripper registers.
        The high byte of the position is fixed at 0xFF (11111111).
        """
        try:
            # Clamp values to valid ranges
            position = max(0, min(255, request.position))  # 8-bit position (low byte only)
            speed = max(0, min(255, request.speed))        # 8-bit speed
            force = max(0, min(255, request.force))        # 8-bit force

            # Encode ACTION REQUEST (Byte 0)
            action_request = 9  # ACTION REQUEST = 9

            # Reserved byte (Byte 1)
            reserved_byte = 0  # Fixed reserved value

            # Encode POSITION REQUEST (Bytes 2 and 3)
            high_byte_position = 0xFF                     # Fixed high byte of position
            low_byte_position = position                  # Low byte of position

            # Encode SPEED and FORCE (Bytes 4 and 5)
            encoded_speed = speed  # SPEED (high byte of 2002)
            encoded_force = force  # FORCE (low byte of 2002)

            # Prepare values for Modbus (as U16 registers)
            val_tab = f"{(action_request << 8) | reserved_byte}," \
                    f"{(high_byte_position << 8) | low_byte_position}," \
                    f"{(encoded_speed << 8) | encoded_force}"

            # Log the values being written to SetHoldRegs
            self.get_logger().info(
                f"SetHoldRegs called with: index={request.index}, addr=1000, count=3, "
                f"val_tab={val_tab}, val_type=U16"
            )

            # Write values to addr: 1000 using SetHoldRegs
            return_t = self.dashboard.SetHoldRegs(
                request.index, 1000, 3, val_tab, "U16"
            )

            # Parse response
            return_tt = return_t[:return_t.find("{") - 1]
            response.res = int(return_tt)
            response.message = (
                f"Gripper data written successfully: action_request={action_request}, position={position}, speed={speed}, force={force}"
            )
            self.get_logger().info(response.message)
        except Exception as e:
            response.res = -1
            response.message = f"Error in SetGripperPosition: {e}"
            self.get_logger().error(response.message)
        return response
    
    def ModbusClose(self, request, response):                                          
        return_t = self.dashboard.ModbusClose(request.index)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                         
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetErrorID(self, request, response):                                           
        return_t = self.dashboard.GetErrorID()
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DisableRobot(self, request, response):                                           
        return_t = self.dashboard.DisableRobot()
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DOExecute(self, request, response):                                       
        return_t = self.dashboard.DOExecute(request.index,request.status)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SpeedFactor(self, request, response):                                       
        return_t = self.dashboard.SpeedFactor(request.ratio)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def CP(self, request, response):                                       
        return_t = self.dashboard.CP(request.r)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SpeedJ(self, request, response):                                       
        return_t = self.dashboard.SpeedJ(request.r)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SpeedL(self, request, response):                                       
        return_t = self.dashboard.SpeedL(request.r)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 

    def Tool(self, request, response):                                       
        return_t = self.dashboard.Tool(request.index)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def User(self, request, response):                                       
        return_t = self.dashboard.User(request.index)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DOGroup(self, request, response):                                       
        return_t = self.dashboard.DOGroup(request.args)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DO(self, request, response):                                       
        return_t = self.dashboard.DO(request.index,request.status)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DI(self, request, response):                                       
        return_t = self.dashboard.ToolDO(request.index,0)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ToolDO(self, request, response):                                       
        return_t = self.dashboard.ToolDO(request.index,request.status)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ToolDOExecute(self, request, response):                                       
        return_t = self.dashboard.ToolDOExecute(request.index,request.status)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ToolDI(self, request, response):                                       
        return_t = self.dashboard.ToolDI(request.index)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 

    def AccJ(self, request, response):                                     
        return_t = self.dashboard.AccJ(request.r)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def AccL(self, request, response):                                      
        return_t = self.dashboard.AccL(request.r)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def Arch(self, request, response):                                        
        return_t = self.dashboard.Arch(request.index)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def MovJ(self, request, response):                                
        return_t = self.move.MovJ(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ServoP(self, request, response):                                
        return_t = self.move.ServoP(request.x,request.y,request.z,request.rx,request.ry,request.rz)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ServoJ(self, request, response):                                
        return_t = self.move.ServoJ(request.j1,request.j2,request.j3,request.j4,request.j5,request.j6,request.t,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 

    def MovL(self, request, response):                                
        return_t = self.move.MovL(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def MovJIO(self, request, response):                                
        return_t = self.move.MovJIO(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def MovLIO(self, request, response):                                
        return_t = self.move.MovLIO(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def JointMovJ(self, request, response):                                
        return_t = self.move.JointMovJ(request.j1,request.j2,request.j3,request.j4,request.j5,request.j6,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def RelMovJ(self, request, response):                                
        return_t = self.move.RelMovJ(request.offset1,request.offset2,request.offset3,request.offset4,request.offset5,request.offset6,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def RelMovL(self, request, response):                               
        return_t = self.move.RelMovL(request.offset1,request.offset2,request.offset3,request.offset4,request.offset5,request.offset6,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def Sync(self, request, response):                                
        return_t = self.move.Sync()
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def MoveJog(self, request, response):                                
        return_t = self.move.MoveJog(request.axis_id,request.param_value)
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)                                           
        self.get_logger().info(return_t)                                     
        return response

    def StartDrag(self, request, response):
        try:
            return_t = self.dashboard.StartDrag()
            return_tt = return_t[:return_t.find("{")-1]
            response.res = int(return_tt)
            self.get_logger().info(return_t)
        except Exception as e:
            self.get_logger().error(f"StartDrag error: {e}")
            response.res = -1
        return response

    def StopDrag(self, request, response):
        try:
            return_t = self.dashboard.StopDrag()
            return_tt = return_t[:return_t.find("{")-1]
            response.res = int(return_tt)
            self.get_logger().info(return_t)
        except Exception as e:
            self.get_logger().error(f"StopDrag error: {e}")
            response.res = -1
        return response
    
    def InverseSolution(self, request, response):
        return_t = self.dashboard.InverseSolution(
            request.x, request.y, request.z, request.rx,
            request.ry, request.rz, request.user, request.tool
        )
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.angle = return_t[return_t.find("{"):return_t.find("}")+1]
        self.get_logger().info(return_t)
        return response

    def PositiveSolution(self, request, response):
        return_t = self.dashboard.PositiveSolution(
            request.j1, request.j2, request.j3, request.j4,
            request.j5, request.j6, request.user, request.tool
        )
        return_tt = return_t[:return_t.find("{")-1]
        response.res = int(return_tt)
        response.pose = return_t[return_t.find("{"):return_t.find("}")+1]
        self.get_logger().info(return_t)
        return response

def main(args=None):                                
    rclpy.init(args=args)                            
    node = adderServer("dobot_bringup_v3")      
    rclpy.spin(node)                                
    node.destroy_node()                             
    rclpy.shutdown()

