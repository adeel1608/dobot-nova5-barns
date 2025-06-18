#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from dobot_msgs_v3.srv import (
    InverseSolution,
    PositiveSolution,
    GetPose,
    GetAngle,
    StartDrag,
    StopDrag,
    SetGripperPosition,
    GetGripperPosition,
    ClearError,
    DisableRobot,
    EnableRobot,
    ModbusClose,
    ModbusCreate,
    SetHoldRegs
)

def print_response(res):
    try:
        fields = res._fields_and_field_types
    except AttributeError:
        fields = {slot: None for slot in res.__slots__}
    print("Service response:")
    for field in fields:
        val = getattr(res, field)
        if isinstance(val, (float, int)):
            print(f"  {field}={val:.6f}")
        else:
            print(f"  {field}={val}")


def main():
    rclpy.init()
    node = rclpy.create_node('kinematics_client')

    # Create service clients
    services = {
        'inverse': node.create_client(InverseSolution, '/dobot_bringup_v3/srv/InverseSolution'),
        'forward': node.create_client(PositiveSolution, '/dobot_bringup_v3/srv/PositiveSolution'),
        'pose': node.create_client(GetPose, '/dobot_bringup_v3/srv/GetPose'),
        'angle': node.create_client(GetAngle, '/dobot_bringup_v3/srv/GetAngle'),
        'start_drag': node.create_client(StartDrag, '/dobot_bringup_v3/srv/StartDrag'),
        'stop_drag': node.create_client(StopDrag, '/dobot_bringup_v3/srv/StopDrag'),
        'set_gripper': node.create_client(SetGripperPosition, '/dobot_bringup_v3/srv/SetGripperPosition'),
        'get_gripper': node.create_client(GetGripperPosition, '/dobot_bringup_v3/srv/GetGripperPosition'),
        'clear_error': node.create_client(ClearError, '/dobot_bringup_v3/srv/ClearError'),
        'disable_robot': node.create_client(DisableRobot, '/dobot_bringup_v3/srv/DisableRobot'),
        'enable_robot': node.create_client(EnableRobot, '/dobot_bringup_v3/srv/EnableRobot'),
        'modbus_close': node.create_client(ModbusClose, '/dobot_bringup_v3/srv/ModbusClose'),
        'modbus_create': node.create_client(ModbusCreate, '/dobot_bringup_v3/srv/ModbusCreate'),
        'set_hold_regs': node.create_client(SetHoldRegs, '/dobot_bringup_v3/srv/SetHoldRegs'),
    }

    # Wait for services
    for name, client in services.items():
        node.get_logger().info(f'Waiting for {name} service...')
        client.wait_for_service()

    MENU = [
        "1. Inverse Kinematics",
        "2. Forward Kinematics",
        "3. Get Current Pose",
        "4. Get Current Angles",
        "5. Start Drag",
        "6. Stop Drag",
        "7. Open Gripper",
        "8. Close Gripper",
        "9. Get Gripper Position",
        "10. Initialize",
        "Q. Quit"
    ]

    while True:
        print("\nSelect mode:")
        for item in MENU:
            print(item)
        choice = input("Enter choice: ").strip().lower()

        # Helper to call and print
        def call(name, req):
            future = services[name].call_async(req)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                print_response(future.result())
            else:
                print('Service call failed.')

        if choice in ['1', 'inverse', 'i']:
            vals = input("Enter x,y,z,rx,ry,rz: ").split(',')
            if len(vals) != 6:
                print("Please enter 6 comma-separated values.")
                continue
            try:
                req = InverseSolution.Request(
                    x=float(vals[0]), y=float(vals[1]), z=float(vals[2]),
                    rx=float(vals[3]), ry=float(vals[4]), rz=float(vals[5])
                )
            except ValueError:
                print("Invalid numeric input.")
                continue
            call('inverse', req)

        elif choice in ['2', 'forward', 'f']:
            vals = input("Enter j1,j2,j3,j4,j5,j6: ").split(',')
            if len(vals) != 6:
                print("Please enter 6 comma-separated values.")
                continue
            try:
                req = PositiveSolution.Request(
                    j1=float(vals[0]), j2=float(vals[1]), j3=float(vals[2]),
                    j4=float(vals[3]), j5=float(vals[4]), j6=float(vals[5])
                )
            except ValueError:
                print("Invalid numeric input.")
                continue
            call('forward', req)

        elif choice in ['3', 'pose', 'p']:
            call('pose', GetPose.Request())

        elif choice in ['4', 'angle', 'a']:
            call('angle', GetAngle.Request())

        elif choice in ['5', 'start drag', 'sd']:
            call('start_drag', StartDrag.Request())

        elif choice in ['6', 'stop drag', 'td']:
            call('stop_drag', StopDrag.Request())

        elif choice in ['7', 'open gripper', 'og']:
            call('set_gripper', SetGripperPosition.Request(position=0, speed=255, force=255))

        elif choice in ['8', 'close gripper', 'cg']:
            call('set_gripper', SetGripperPosition.Request(position=255, speed=255, force=255))

        elif choice in ['9', 'get gripper', 'gg']:
            call('get_gripper', GetGripperPosition.Request())

        elif choice in ['10', 'initialize', 'init']:
            print("Initializing sequence...")
            call('clear_error', ClearError.Request())
            time.sleep(0.1)
            call('disable_robot', DisableRobot.Request())
            time.sleep(0.1)
            call('enable_robot', EnableRobot.Request(load=2.0))
            time.sleep(0.1)
            call('start_drag', StartDrag.Request())
            time.sleep(0.1)
            call('stop_drag', StopDrag.Request())
            time.sleep(0.1)
            call('modbus_close', ModbusClose.Request(index=0))
            time.sleep(0.1)
            call('modbus_create', ModbusCreate.Request(ip='127.0.0.1', port=60000, slave_id=9, is_rtu=1))
            time.sleep(0.1)
            call('set_hold_regs', SetHoldRegs.Request(index=0, addr=1000, count=3, val_tab='0,0,0', val_type='int'))
            time.sleep(0.1)
            call('set_hold_regs', SetHoldRegs.Request(index=0, addr=1000, count=3, val_tab='256,0,0', val_type='int'))
            time.sleep(0.1)
            print("Initialization complete.")

        elif choice in ['q', 'quit', 'exit']:
            print('Exiting...')
            break
        else:
            print('Invalid choice, please try again.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrupted by user, shutting down.')
        sys.exit(0)

