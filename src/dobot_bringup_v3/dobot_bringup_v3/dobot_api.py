#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket

class DobotApi:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket_dobot = 0

        if self.port == 29999 or self.port == 30003:
            try:
                self.socket_dobot = socket.socket()
                
                # Enable TCP keepalive to prevent connection timeouts
                self.socket_dobot.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                
                # Set keepalive parameters (Linux-specific)
                # Start sending keepalive probes after 10 seconds of idle time
                self.socket_dobot.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                # Send keepalive probes every 5 seconds
                self.socket_dobot.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5)
                # Close connection after 3 failed probes
                self.socket_dobot.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                
                # Set socket timeout to 15 seconds to handle rapid command sequences
                # (increased from 5s to prevent connection loss during command bursts)
                self.socket_dobot.settimeout(15.0)
                
                self.socket_dobot.connect((self.ip, self.port))
                print(f"Connected to Dobot at {self.ip}:{self.port} with keepalive enabled")
            except socket.error as e:
                print(f"Failed to connect to Dobot at {self.ip}:{self.port} - {e}")
                self.socket_dobot = None
        else:
            print(f"Connect to dashboard server need use port {self.port}!")

    def send_data(self, string):
        if self.socket_dobot is None:
            print("No valid socket connection.")
            return
        try:
            print(f"Sending: {string}")
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except socket.error as e:
            print(f"Failed to send data: {e}")
            raise

    def wait_reply(self):
        if self.socket_dobot is None:
            print("No valid socket connection.")
            return "No valid socket connection"
        try:
            data = self.socket_dobot.recv(1024)
            data_str = str(data, encoding="utf-8")
            return data_str
        except socket.error as e:
            print(f"Failed to receive data: {e}")
            raise

    def close(self):
        """
        Close the port
        """
        if self.socket_dobot != 0 and self.socket_dobot is not None:
            self.socket_dobot.close()
            self.socket_dobot = None

    def sendRecvMsg(self, string):
        # This is the core function that sends a command and waits for a reply.
        # We handle exceptions here so that errors don't crash the node.
        try:
            self.send_data(string)
            return self.wait_reply()
        except Exception as e:
            # Catch any exception that occurred in send_data or wait_reply
            error_msg = f"Error in sendRecvMsg: {e}"
            print(error_msg)
            return error_msg


class DobotApiDashboard(DobotApi):

    def EnableRobot(self,*dynParams):
        try:
            string = "EnableRobot(" + str(dynParams[0][0]) + ")"
            return self.sendRecvMsg(string)
        except Exception as e:
            error_msg = f"Error enabling robot: {e}"
            print(error_msg)
            return error_msg

    def DisableRobot(self):
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def ClearError(self):
        string = "ClearError()"
        return self.sendRecvMsg(string)

    def ResetRobot(self):
        string = "ResetRobot()"
        return self.sendRecvMsg(string)

    def SpeedFactor(self, speed):
        try:
            string = "SpeedFactor({:d})".format(int(speed))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"SpeedFactor ValueError: {e}"
            print(error_msg)
            return error_msg

    def User(self, index):
        try:
            string = "User({:d})".format(int(index))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"User ValueError: {e}"
            print(error_msg)
            return error_msg

    def Tool(self, index):
        try:
            string = "Tool({:d})".format(int(index))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"Tool ValueError: {e}"
            print(error_msg)
            return error_msg
        
    def SetTool(self, index, table):
        """
        index : tool coordinate system index, range [0–9]
        table : tool coordinate system after modification (format: {x, y, z, rx, ry, rz})
        """
        string = "SetTool({:d},{:s})".format(index, table)
        return self.sendRecvMsg(string)

    def RobotMode(self):
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def PayLoad(self, weight, inertia):
        try:
            string = "PayLoad({:f},{:f})".format(float(weight), float(inertia))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"PayLoad ValueError: {e}"
            print(error_msg)
            return error_msg

    def DO(self, index, status):
        try:
            string = "DO({:d},{:d})".format(int(index), int(status))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"DO ValueError: {e}"
            print(error_msg)
            return error_msg

    def AccJ(self, speed):
        try:
            string = "AccJ({:d})".format(int(speed))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"AccJ ValueError: {e}"
            print(error_msg)
            return error_msg

    def AccL(self, speed):
        try:
            string = "AccL({:d})".format(int(speed))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"AccL ValueError: {e}"
            print(error_msg)
            return error_msg

    def SpeedJ(self, speed):
        try:
            string = "SpeedJ({:d})".format(int(speed))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"SpeedJ ValueError: {e}"
            print(error_msg)
            return error_msg

    def SpeedL(self, speed):
        try:
            string = "SpeedL({:d})".format(int(speed))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"SpeedL ValueError: {e}"
            print(error_msg)
            return error_msg

    def Arch(self, index):
        try:
            string = "Arch({:d})".format(int(index))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"Arch ValueError: {e}"
            print(error_msg)
            return error_msg

    def CP(self, ratio):
        try:
            string = "CP({:d})".format(int(ratio))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"CP ValueError: {e}"
            print(error_msg)
            return error_msg

    def LimZ(self, value):
        try:
            string = "LimZ({:d})".format(int(value))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"LimZ ValueError: {e}"
            print(error_msg)
            return error_msg

    def RunScript(self, project_name):
        string = "RunScript({:s})".format(str(project_name))
        return self.sendRecvMsg(string)

    def StopScript(self):
        string = "StopScript()"
        return self.sendRecvMsg(string)

    def PauseScript(self):
        string = "PauseScript()"
        return self.sendRecvMsg(string)

    def ContinueScript(self):
        string = "ContinueScript()"
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, id, addr, count, type=None):
        try:
            id = int(id)
            addr = int(addr)
            count = int(count)
            if type is not None:
                string = "GetHoldRegs({:d},{:d},{:d},{:s})".format(
                    id, addr, count, str(type))
            else:
                string = "GetHoldRegs({:d},{:d},{:d})".format(
                    id, addr, count)
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"GetHoldRegs ValueError: {e}"
            print(error_msg)
            return error_msg

    def SetHoldRegs(self, id, addr, count, table, type=None):
        # If 'table' is a string of comma-separated values, split and convert them to integers.
        if isinstance(table, str):
            vals_str = table.split(',')
            # Convert each value to an integer, stripping spaces if any
            try:
                vals = [int(v.strip()) for v in vals_str]
            except ValueError as e:
                # If there is a non-integer value, return an error message
                error_msg = f"SetHoldRegs ValueError: Invalid integer in val_tab - {e}"
                print(error_msg)
                return error_msg
        else:
            # If table is already an integer or list, handle accordingly
            if isinstance(table, int):
                vals = [table]
            elif isinstance(table, list):
                # Ensure all are integers
                try:
                    vals = [int(v) for v in table]
                except ValueError as e:
                    error_msg = f"SetHoldRegs ValueError: Invalid integer in val_tab list - {e}"
                    print(error_msg)
                    return error_msg
            else:
                # Unsupported type
                error_msg = f"SetHoldRegs Error: 'table' must be an int, list of ints, or comma-separated string of ints."
                print(error_msg)
                return error_msg

        # Now 'vals' is a list of integers, e.g. [2304, 65535, 65535]
        # Construct the command string
        vals_str_formatted = ",".join(str(v) for v in vals)
        if type is not None:
            # If val_type is provided
            string = f"SetHoldRegs({id},{addr},{count},{vals_str_formatted},{type})"
        else:
            # Without val_type
            string = f"SetHoldRegs({id},{addr},{count},{vals_str_formatted})"

        return self.sendRecvMsg(string)


    def GetErrorID(self):
        string = "GetErrorID()"
        return self.sendRecvMsg(string)

    def DOExecute(self,offset1,offset2):
        try:
            string = "DOExecute({:d},{:d})".format(int(offset1),int(offset2))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"DOExecute ValueError: {e}"
            print(error_msg)
            return error_msg

    def ToolDO(self,offset1,offset2):
        try:
            string = "ToolDO({:d},{:d})".format(int(offset1),int(offset2))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"ToolDO ValueError: {e}"
            print(error_msg)
            return error_msg

    def ToolDOExecute(self,offset1,offset2):
        try:
            string = "ToolDOExecute({:d},{:d})".format(int(offset1),int(offset2))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"ToolDOExecute ValueError: {e}"
            print(error_msg)
            return error_msg

    def SetArmOrientation(self,offset1):
        try:
            string = "SetArmOrientation({:d})".format(int(offset1))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"SetArmOrientation ValueError: {e}"
            print(error_msg)
            return error_msg

    def SetPayload(self, weight, inertia):
        try:
            string = "SetPayLoad({:f},{:f})".format(float(weight), float(inertia))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"SetPayload ValueError: {e}"
            print(error_msg)
            return error_msg

    # def PositiveSolution(self,offset1,offset2,offset3,offset4,user,tool):   
    #     try:
    #         string = "PositiveSolution({:f},{:f},{:f},{:f},{:d},{:d})".format(
    #             float(offset1), float(offset2), float(offset3), float(offset4), int(user), int(tool))
    #         return self.sendRecvMsg(string)
    #     except ValueError as e:
    #         error_msg = f"PositiveSolution ValueError: {e}"
    #         print(error_msg)
    #         return error_msg
        
    def PositiveSolution(self, j1, j2, j3, j4, j5, j6, user, tool):
        string = "PositiveSolution({:f},{:f},{:f},{:f},{:f},{:f},{:d},{:d}".format(
            j1, j2, j3, j4, j5, j6, user, tool) + ")"
        return self.sendRecvMsg(string)

    # def InverseSolution(self,offset1,offset2,offset3,offset4,user,tool,*dynParams):       
    #     try:
    #         string = "InverseSolution({:f},{:f},{:f},{:f},{:d},{:d}".format(
    #             float(offset1), float(offset2), float(offset3), float(offset4), int(user), int(tool))
    #         # dynParams could contain different data; just append as string
    #         for params in dynParams:
    #             string = string + repr(params)
    #         string = string + ")"
    #         return self.sendRecvMsg(string)
    #     except ValueError as e:
    #         error_msg = f"InverseSolution ValueError: {e}"
    #         print(error_msg)
    #         return error_ms
        
    def InverseSolution(self, x, y, z, rx, ry, rz, user, tool, *dynParams):
        string = "InverseSolution({:f},{:f},{:f},{:f},{:f},{:f},{:d},{:d}".format(
            x, y, z, rx, ry, rz, user, tool)
        for params in dynParams:
            print(type(params), params)
            string = string + repr(params)
        string = string + ")"
        return self.sendRecvMsg(string)

    def SetCollisionLevel(self,offset1):
        try:
            string = "SetCollisionLevel({:d})".format(int(offset1))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"SetCollisionLevel ValueError: {e}"
            print(error_msg)
            return error_msg

    def GetAngle(self):
        string = "GetAngle()"
        return self.sendRecvMsg(string)

    def GetPose(self,User=0,Tool=0):
        try:
            User = int(User)
            Tool = int(Tool)
            string = "GetPose(User={:d},Tool={:d})".format(User,Tool)
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"GetPose ValueError: {e}"
            print(error_msg)
            return error_msg

    def EmergencyStop(self):
        string = "EmergencyStop()"
        return self.sendRecvMsg(string)

    def ModbusCreate(self,ip,port,slave_id,isRTU):
        try:
            string ="ModbusCreate({:s},{:d},{:d},{:d})".format(str(ip),int(port),int(slave_id),int(isRTU))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"ModbusCreate ValueError: {e}"
            print(error_msg)
            return error_msg

    def ModbusClose(self,offset1):
        try:
            string = "ModbusClose({:d})".format(int(offset1))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"ModbusClose ValueError: {e}"
            print(error_msg)
            return error_msg

    def GetInBits(self,offset1,offset2,offset3):
        try:
            string = "GetInBits({:d},{:d},{:d})".format(int(offset1),int(offset2),int(offset3))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"GetInBits ValueError: {e}"
            print(error_msg)
            return error_msg

    def GetInRegs(self,offset1,offset2,offset3,*dynParams):
        try:
            string = "GetInRegs({:d},{:d},{:d}".format(int(offset1),int(offset2),int(offset3))
            for params in dynParams:
                # Assuming params is a tuple/list of strings or something convertible
                # If not sure, print to debug: print(type(params), params)
                string = string + str(params[0])
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"GetInRegs ValueError: {e}"
            print(error_msg)
            return error_msg

    def GetCoils(self,offset1,offset2,offset3):
        try:
            string = "GetCoils({:d},{:d},{:d})".format(int(offset1),int(offset2),int(offset3))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"GetCoils ValueError: {e}"
            print(error_msg)
            return error_msg

    def SetCoils(self,offset1,offset2,offset3,offset4):
        try:
            offset1 = int(offset1)
            offset2 = int(offset2)
            offset3 = int(offset3)
            # offset4 could be boolean or int, not sure, just use repr
            string = "SetCoils({:d},{:d},{:d},{}".format(offset1,offset2,offset3, repr(offset4)) + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"SetCoils ValueError: {e}"
            print(error_msg)
            return error_msg

    def DI(self,offset1):
        try:
            string = "DI({:d})".format(int(offset1))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"DI ValueError: {e}"
            print(error_msg)
            return error_msg

    def ToolDI(self,offset1):
        try:
            string = "DI({:d})".format(int(offset1))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"ToolDI ValueError: {e}"
            print(error_msg)
            return error_msg

    def DOGroup(self,*dynParams):
        try:
            string = "DOGroup("
            for params in dynParams[0]:
                string = string + str(params)+","
            string =string+ ")"
            return self.wait_reply()
        except Exception as e:
            error_msg = f"DOGroup error: {e}"
            print(error_msg)
            return error_msg

    def BrakeControl(self,offset1,offset2):
        try:
            string = "BrakeControl({:d},{:d})".format(int(offset1),int(offset2))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"BrakeControl ValueError: {e}"
            print(error_msg)
            return error_msg

    def StartDrag(self):
        string = "StartDrag()"
        return self.sendRecvMsg(string)

    def StopDrag(self):
        string = "StopDrag()"
        return self.sendRecvMsg(string)

    def LoadSwitch(self,offset1):
        try:
            string = "LoadSwitch({:d})".format(int(offset1))
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"LoadSwitch ValueError: {e}"
            print(error_msg)
            return error_msg

    def wait(self):
        string = "wait()"
        return self.sendRecvMsg(string)

    def pause(self):
        string = "pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        string = "continue()"
        return self.sendRecvMsg(string)


class DobotApiMove(DobotApi):
    def MovJ(self, x, y, z, rx, ry, rz, *dynParams):
        try:
            x, y, z, rx, ry, rz = float(x), float(y), float(z), float(rx), float(ry), float(rz)
            string = "MovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
                x, y, z, rx, ry, rz)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            print(string)  
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"MovJ ValueError: {e}"
            print(error_msg)
            return error_msg

    def MovL(self, x, y, z, rx, ry, rz, *dynParams):
        try:
            x, y, z, rx, ry, rz = float(x), float(y), float(z), float(rx), float(ry), float(rz)
            string = "MovL({:f},{:f},{:f},{:f},{:f},{:f}".format(
                x, y, z, rx, ry, rz)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            print(string) 
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"MovL ValueError: {e}"
            print(error_msg)
            return error_msg

    def JointMovJ(self, j1, j2, j3, j4, j5, j6, *dynParams):
        try:
            j1, j2, j3, j4, j5, j6 = float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)
            string = "JointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
                j1, j2, j3, j4, j5, j6)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            print(string)
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"JointMovJ ValueError: {e}"
            print(error_msg)
            return error_msg

    def ServoJ(self, j1, j2, j3, j4, j5, j6, t, *dynParams):
        try:
            j1, j2, j3, j4, j5, j6, t = float(j1), float(j2), float(j3), float(j4), float(j5), float(j6), float(t)
            string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f},t={:f}".format(
                j1, j2, j3, j4, j5, j6, t)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            print(string) 
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"ServoJ ValueError: {e}"
            print(error_msg)
            return error_msg

    def ServoP(self, x, y, z, rx, ry, rz):
        try:
            x, y, z, rx, ry, rz = float(x), float(y), float(z), float(rx), float(ry), float(rz)
            string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f})".format(
                x, y, z, rx, ry, rz)
            print(string) 
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"ServoP ValueError: {e}"
            print(error_msg)
            return error_msg

    def Jump(self):
        print("待定")

    def RelMovJ(self, offset1, offset2, offset3, offset4, offset5, offset6, *dynParams):
        try:
            offsets = [float(offset1), float(offset2), float(offset3), float(offset4), float(offset5), float(offset6)]
            string = "RelMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(*offsets)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"RelMovJ ValueError: {e}"
            print(error_msg)
            return error_msg

    def RelMovL(self, offset1, offset2, offset3, offset4, offset5, offset6, *dynParams):
        try:
            offsets = [float(offset1), float(offset2), float(offset3), float(offset4), float(offset5), float(offset6)]
            string = "RelMovL({:f},{:f},{:f},{:f},{:f},{:f}".format(*offsets)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"RelMovL ValueError: {e}"
            print(error_msg)
            return error_msg

    def MovLIO(self, x, y, z, rx, ry, rz, *dynParams):
        try:
            x, y, z, rx, ry, rz = float(x), float(y), float(z), float(rx), float(ry), float(rz)
            string = "MovLIO({:f},{:f},{:f},{:f},{:f},{:f}".format(
                x, y, z, rx, ry, rz)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"MovLIO ValueError: {e}"
            print(error_msg)
            return error_msg

    def MovJIO(self, x, y, z, rx, ry, rz, *dynParams):
        try:
            x, y, z, rx, ry, rz = float(x), float(y), float(z), float(rx), float(ry), float(rz)
            string = "MovJIO({:f},{:f},{:f},{:f},{:f},{:f}".format(
                x, y, z, rx, ry, rz)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            print(string)
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"MovJIO ValueError: {e}"
            print(error_msg)
            return error_msg

    def Arc(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, *dynParams):
        try:
            coords = [float(x1), float(y1), float(z1), float(rx1), float(ry1), float(rz1),
                      float(x2), float(y2), float(z2), float(rx2), float(ry2), float(rz2)]
            string = "Arc({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f}".format(*coords)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            print(string)
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"Arc ValueError: {e}"
            print(error_msg)
            return error_msg

    def Circle3(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, count, *dynParams):
        try:
            coords = [float(x1), float(y1), float(z1), float(rx1), float(ry1), float(rz1),
                      float(x2), float(y2), float(z2), float(rx2), float(ry2), float(rz2), int(count)]
            string = "Circle3({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(*coords)
            for params in dynParams:
                string = string + "," + str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"Circle3 ValueError: {e}"
            print(error_msg)
            return error_msg

    def MoveJog(self, axis_id=None, *dynParams):
        try:
            if axis_id is not None:
                axis_id = str(axis_id)
                string = "MoveJog({:s}".format(axis_id)
            else:
                string = "MoveJog("

            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except Exception as e:
            error_msg = f"MoveJog error: {e}"
            print(error_msg)
            return error_msg

    def Sync(self):
        string = "Sync()"
        return self.sendRecvMsg(string)

    def RelMovJUser(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, user, *dynParams):
        try:
            offsets = [float(offset_1), float(offset_2), float(offset_3), float(offset_4), float(offset_5), float(offset_6)]
            user = int(user)
            string = "RelMovJUser({:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(*offsets, user)
            for params in dynParams[0]:
                string = string + ","+ str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"RelMovJUser ValueError: {e}"
            print(error_msg)
            return error_msg

    def RelMovJTool(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, tool, *dynParams):
        try:
            offsets = [float(offset_1), float(offset_2), float(offset_3), float(offset_4), float(offset_5), float(offset_6)]
            tool = int(tool)
            string = "RelMovJTool({:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(*offsets, tool)
            for params in dynParams[0]:
                string = string + ","+ str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"RelMovJTool ValueError: {e}"
            print(error_msg)
            return error_msg

    def RelMovLUser(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, user, *dynParams):
        try:
            offsets = [float(offset_1), float(offset_2), float(offset_3), float(offset_4), float(offset_5), float(offset_6)]
            user = int(user)
            string = "RelMovLUser({:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(*offsets, user)
            for params in dynParams[0]:
                string = string + ","+ str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"RelMovLUser ValueError: {e}"
            print(error_msg)
            return error_msg

    def RelMovLTool(self, offset_1, offset_2, offset_3, offset_4, offset_5, offset_6, tool, *dynParams):
        try:
            offsets = [float(offset_1), float(offset_2), float(offset_3), float(offset_4), float(offset_5), float(offset_6)]
            tool = int(tool)
            string = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f},{:d}".format(*offsets, tool)
            for params in dynParams[0]:
                string = string + ","+ str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"RelMovLTool ValueError: {e}"
            print(error_msg)
            return error_msg

    def RelJointMovJ(self, offset1, offset2, offset3, offset4, offset5, offset6, *dynParams):
        try:
            offsets = [float(offset1), float(offset2), float(offset3), float(offset4), float(offset5), float(offset6)]
            string = "RelJointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(*offsets)
            for params in dynParams:
                string = string + "," + str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"RelJointMovJ ValueError: {e}"
            print(error_msg)
            return error_msg

    def MovJExt(self, offset1, *dynParams):
        try:
            offset1 = float(offset1)
            string = "MovJExt({:f}".format(offset1)
            for params in dynParams[0]:
                string = string + "," + str(params)
            string = string + ")"
            return self.sendRecvMsg(string)
        except ValueError as e:
            error_msg = f"MovJExt ValueError: {e}"
            print(error_msg)
            return error_msg

    def SyncAll(self):
        string = "SyncAll()"
        return self.sendRecvMsg(string)

