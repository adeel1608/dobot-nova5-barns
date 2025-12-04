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
        self.srv = self.create_service(Circle3,'/dobot_bringup_v3/srv/Circle3',self.Circle3)
        self.srv = self.create_service(Arc,'/dobot_bringup_v3/srv/Arc',self.Arc)
        self.srv = self.create_service(SetTool, '/dobot_bringup_v3/srv/SetTool', self.SetTool)
        self.connect() 

    def connect(self):
        try:
           self.get_logger().info("connection:29999")
           self.get_logger().info("connection:30003")
           self.dashboard = DobotApiDashboard(self.IP, 29999)
           self.move = DobotApiMove(self.IP,30003)
           self.get_logger().info("connection succeeded:29999,30003")
           self.connection_lost = False
           self.reconnect_attempts = 0
        except:
            self.get_logger().info("Connection failed!!!")
            self.connection_lost = True
    
    def reconnect(self):
        """
        Attempt to reconnect to the robot when connection is lost.
        Returns True if reconnection successful, False otherwise.
        """
        if self.reconnect_attempts >= 20:
            self.get_logger().error("Maximum reconnection attempts reached (20). Giving up.")
            return False
        
        self.reconnect_attempts += 1
        self.get_logger().warn(f"Attempting to reconnect... (Attempt {self.reconnect_attempts}/20)")
        
        try:
            # Close existing connections if any
            if hasattr(self, 'dashboard') and self.dashboard:
                try:
                    self.dashboard.close()
                except:
                    pass
            if hasattr(self, 'move') and self.move:
                try:
                    self.move.close()
                except:
                    pass
            
            # Wait a bit before reconnecting
            import time
            time.sleep(2.0)
            
            # Reconnect
            self.connect()
            
            if not self.connection_lost:
                self.get_logger().info("Reconnection successful!")
                self.reconnect_attempts = 0
                return True
            else:
                self.get_logger().error("Reconnection failed")
                return False
        except Exception as e:
            self.get_logger().error(f"Reconnection error: {e}")
            return False
    
    def ensure_connection(self):
        """
        Check if connection is lost and attempt to reconnect if necessary.
        Returns True if connection is good, False if reconnection failed.
        """
        if self.connection_lost:
            self.get_logger().info("Connection was lost, attempting reconnection...")
            return self.reconnect()
        return True
    
    def execute_with_retry(self, command_func, *args, **kwargs):
        """
        Execute a command with automatic retry on connection failure.
        This ensures commands are NOT skipped after reconnection.
        
        Args:
            command_func: The function to call (e.g., self.move.JointMovJ)
            *args, **kwargs: Arguments to pass to the command function
            
        Returns:
            The result from command_func, or error message if all retries fail
        """
        max_retries = 20
        
        for attempt in range(max_retries):
            # Execute the command
            result = command_func(*args, **kwargs)
            
            # Check if it was a connection error
            if isinstance(result, str) and result.startswith("Error"):
                connection_errors = ["Connection reset", "Broken pipe", "timed out", "Connection refused"]
                is_connection_error = any(err in result for err in connection_errors)
                
                if is_connection_error:
                    if not self.connection_lost:
                        self.connection_lost = True
                        self.get_logger().warn(f"Connection lost during command execution! (Attempt {attempt + 1}/{max_retries})")
                    
                    # Attempt to reconnect
                    if self.reconnect():
                        self.get_logger().info(f"Retrying failed command after reconnection...")
                        continue  # Retry the command
                    else:
                        self.get_logger().error("Reconnection failed, cannot retry command")
                        return result  # Return the error
                else:
                    # Non-connection error, return immediately
                    return result
            else:
                # Command succeeded
                return result
        
        # All retries exhausted
        self.get_logger().error(f"Command failed after {max_retries} attempts")
        return "Error: Command failed after retries"
    
    def safe_parse_response(self, return_t):
        """
        Safely parse a response from the robot.
        Returns (success: bool, response_code: int)
        If parsing fails or an error occurred, returns (False, -1)
        Also triggers reconnection on connection errors.
        """
        try:
            # Check if this is an error message
            if return_t.startswith("Error"):
                self.get_logger().error(f"Robot communication error: {return_t}")
                
                # Check if it's a connection error
                connection_errors = ["Connection reset", "Broken pipe", "timed out", "Connection refused"]
                is_connection_error = any(err in return_t for err in connection_errors)
                
                if is_connection_error and not self.connection_lost:
                    self.connection_lost = True
                    self.get_logger().warn("Connection lost! Will attempt to reconnect on next command.")
                
                return (False, -1)
            
            # Try to parse the normal response format
            bracket_pos = return_t.find("{")
            if bracket_pos == -1:
                self.get_logger().error(f"Unexpected response format: {return_t}")
                return (False, -1)
            
            return_tt = return_t[:bracket_pos-1]
            response_code = int(return_tt)
            
            # Reset reconnection counter on successful command
            if hasattr(self, 'reconnect_attempts'):
                self.reconnect_attempts = 0
            
            return (True, response_code)
        except ValueError as e:
            self.get_logger().error(f"Failed to parse response '{return_t}': {e}")
            return (False, -1)
        except Exception as e:
            self.get_logger().error(f"Unexpected error parsing response: {e}")
            return (False, -1)

    def EnableRobot(self, request, response):                                           
        return_t = self.execute_with_retry(self.dashboard.EnableRobot, [request.load])
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                     
        return response
    
    def ClearError(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.ClearError)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                        
        return response 
    
    def ResetRobot(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.ResetRobot)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                        
        return response 
    
    def PayLoad(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.PayLoad, request.weight, request.inertia)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                        
        return response 
    
    def SetPayload(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.SetPayload, request.weight, request.inertia)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                        
        return response 
    
    def GetPose(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.GetPose, request.user, request.tool)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.pose = return_t[return_t.find("{"):return_t.find("}")+1]
            self.get_logger().info(return_t)
        return response 
    
    def GetAngle(self, request, response):                                           
        return_t = self.execute_with_retry(self.dashboard.GetAngle)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.angle = return_t[return_t.find("{"):return_t.find("}")+1]
            self.get_logger().info(return_t)
        return response 
    
    def RobotMode(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.RobotMode)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.mode = return_t[return_t.find("{")+1:return_t.find("}")]
            self.get_logger().info(return_t)
        return response 
    
    def ModbusCreate(self, request, response):                                           
        return_t = self.execute_with_retry(self.dashboard.ModbusCreate, request.ip, request.port, request.slave_id, request.is_rtu)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.index = return_t[return_t.find("{")+1:return_t.find("}")]
            self.get_logger().info(return_t)
        return response 
    
    def GetInBits(self, request, response):                                           
        return_t = self.execute_with_retry(self.dashboard.GetInBits, request.index, request.addr, request.count)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.value = return_t[return_t.find("{")+1:return_t.find("}")]
            self.get_logger().info(return_t)
        return response 
    
    def GetInRegs(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.GetInRegs, request.index, request.addr, request.count, request.val_type)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
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
        
        # Call the dashboard's GetHoldRegs method WITH RETRY
        return_t = self.execute_with_retry(
            self.dashboard.GetHoldRegs,
            request.index, request.addr, request.count, request.val_type
        )
        
        # Parse the response
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.value = return_t[return_t.find("{") + 1 : return_t.find("}")]
            self.get_logger().info(f"GetHoldRegs response: {return_t}")
        else:
            response.value = ""
        return response

    def GetCoils(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.GetCoils, request.index, request.addr, request.count)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.value = return_t[return_t.find("{")+1:return_t.find("}")]
            self.get_logger().info(return_t)
        return response 
    
    def SetCoils(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.SetCoils, request.index, request.addr, request.count, request.val_tab)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
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
        
        # Call the dashboard's SetHoldRegs method WITH RETRY
        return_t = self.execute_with_retry(
            self.dashboard.SetHoldRegs,
            request.index, request.addr, request.count, request.val_tab, request.val_type
        )
        
        # Parse the response
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
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
            success, response_code = self.safe_parse_response(return_t)
            if not success:
                response.position = 0
                return response
            
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

            # Write values to addr: 1000 using SetHoldRegs WITH RETRY
            return_t = self.execute_with_retry(
                self.dashboard.SetHoldRegs,
                request.index, 1000, 3, val_tab, "U16"
            )

            # Parse response
            success, response_code = self.safe_parse_response(return_t)
            response.res = response_code
            response.message = (
                f"Gripper data written successfully: action_request={action_request}, position={position}, speed={speed}, force={force}"
            )
            if success:
                self.get_logger().info(response.message)
        except Exception as e:
            response.res = -1
            response.message = f"Error in SetGripperPosition: {e}"
            self.get_logger().error(response.message)
        return response
    
    def ModbusClose(self, request, response):                                          
        return_t = self.execute_with_retry(self.dashboard.ModbusClose, request.index)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def GetErrorID(self, request, response):                                           
        return_t = self.execute_with_retry(self.dashboard.GetErrorID)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def DisableRobot(self, request, response):                                           
        return_t = self.execute_with_retry(self.dashboard.DisableRobot)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def DOExecute(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.DOExecute, request.index, request.status)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def SpeedFactor(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.SpeedFactor, request.ratio)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def CP(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.CP, request.r)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def SpeedJ(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.SpeedJ, request.r)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def SpeedL(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.SpeedL, request.r)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 

    def Tool(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.Tool, request.index)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def SetTool(self, request, response):
        raw = self.execute_with_retry(self.dashboard.SetTool, request.index, request.table)
        code_str = raw.strip().split(',')[0]
        try:
            code = int(code_str)
        except ValueError:
            self.get_logger().error(f"SetTool: failed to parse return code from '{raw}'")
            code = -1
        response.res = code
        self.get_logger().info(f"SetTool returned: {raw}")
        return response

    def User(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.User, request.index)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def DOGroup(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.DOGroup, request.args)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def DO(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.DO, request.index, request.status)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def DI(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.ToolDO, request.index, 0)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def ToolDO(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.ToolDO, request.index, request.status)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def ToolDOExecute(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.ToolDOExecute, request.index, request.status)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def ToolDI(self, request, response):                                       
        return_t = self.execute_with_retry(self.dashboard.ToolDI, request.index)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 

    def AccJ(self, request, response):                                     
        return_t = self.execute_with_retry(self.dashboard.AccJ, request.r)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def AccL(self, request, response):                                      
        return_t = self.execute_with_retry(self.dashboard.AccL, request.r)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def Arch(self, request, response):                                        
        return_t = self.execute_with_retry(self.dashboard.Arch, request.index)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def MovJ(self, request, response):                                
        return_t = self.execute_with_retry(
            self.move.MovJ,
            request.x, request.y, request.z, request.rx, request.ry, request.rz, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def ServoP(self, request, response):                                
        return_t = self.execute_with_retry(
            self.move.ServoP,
            request.x, request.y, request.z, request.rx, request.ry, request.rz
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def ServoJ(self, request, response):                                
        return_t = self.execute_with_retry(
            self.move.ServoJ,
            request.j1, request.j2, request.j3, request.j4, request.j5, request.j6, request.t, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 

    def MovL(self, request, response):                                
        return_t = self.execute_with_retry(
            self.move.MovL,
            request.x, request.y, request.z, request.rx, request.ry, request.rz, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def MovJIO(self, request, response):                                
        return_t = self.execute_with_retry(
            self.move.MovJIO,
            request.x, request.y, request.z, request.rx, request.ry, request.rz, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def MovLIO(self, request, response):                                
        return_t = self.execute_with_retry(
            self.move.MovLIO,
            request.x, request.y, request.z, request.rx, request.ry, request.rz, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response 
    
    def JointMovJ(self, request, response):
        # Execute with automatic retry on connection failure
        return_t = self.execute_with_retry(
            self.move.JointMovJ,
            request.j1, request.j2, request.j3, request.j4, request.j5, request.j6, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                     
        return response 
    
    def RelMovJ(self, request, response):
        # Execute with automatic retry on connection failure
        return_t = self.execute_with_retry(
            self.move.RelMovJ,
            request.offset1, request.offset2, request.offset3, request.offset4, request.offset5, request.offset6, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                     
        return response 
    
    def RelMovL(self, request, response):
        # Execute with automatic retry on connection failure
        return_t = self.execute_with_retry(
            self.move.RelMovL,
            request.offset1, request.offset2, request.offset3, request.offset4, request.offset5, request.offset6, request.param_value
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                     
        return response 
    
    def Sync(self, request, response):
        # Execute with automatic retry on connection failure
        return_t = self.execute_with_retry(self.move.Sync)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                     
        return response 
    
    def MoveJog(self, request, response):                                
        return_t = self.execute_with_retry(self.move.MoveJog, request.axis_id, request.param_value)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)                                     
        return response

    def StartDrag(self, request, response):
        return_t = self.execute_with_retry(self.dashboard.StartDrag)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response

    def StopDrag(self, request, response):
        return_t = self.execute_with_retry(self.dashboard.StopDrag)
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response
    
    def InverseSolution(self, request, response):
        return_t = self.execute_with_retry(
            self.dashboard.InverseSolution,
            request.x, request.y, request.z, request.rx,
            request.ry, request.rz, request.user, request.tool
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.angle = return_t[return_t.find("{"):return_t.find("}")+1]
            self.get_logger().info(return_t)
        return response

    def PositiveSolution(self, request, response):
        return_t = self.execute_with_retry(
            self.dashboard.PositiveSolution,
            request.j1, request.j2, request.j3, request.j4,
            request.j5, request.j6, request.user, request.tool
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            response.pose = return_t[return_t.find("{"):return_t.find("}")+1]
            self.get_logger().info(return_t)
        return response
    
    def Circle3(self, request, response):
        return_t = self.execute_with_retry(
            self.move.Circle3,
            request.x1, request.y1, request.z1, request.rx1, request.ry1, request.rz1,
            request.x2, request.y2, request.z2, request.rx2, request.ry2, request.rz2,
            request.count,                                   # ← count goes here
            *request.param_value                             # ← explode the list so each element is appended
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response

    def Arc(self, request, response):
        return_t = self.execute_with_retry(
            self.move.Arc,
            request.x1, request.y1, request.z1, request.rx1, request.ry1, request.rz1,
            request.x2, request.y2, request.z2, request.rx2, request.ry2, request.rz2,
            request.param_value                              # ← keep the list intact for Arc()
        )
        success, response_code = self.safe_parse_response(return_t)
        response.res = response_code
        if success:
            self.get_logger().info(return_t)
        return response


def main(args=None):                                
    rclpy.init(args=args)                            
    node = adderServer("dobot_bringup_v3")      
    rclpy.spin(node)                                
    node.destroy_node()                             
    rclpy.shutdown()
