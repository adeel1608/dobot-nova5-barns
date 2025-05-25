# services/robot_arm/server.py
import grpc
from concurrent import futures
from . import robot_actions
from .proto import robot_pb2, robot_pb2_grpc  # generated from robot.proto

# Implement the gRPC servicer generated from the proto definition
class RobotArmServicer(robot_pb2_grpc.RobotArmServiceServicer):
    def ExecuteAction(self, request, context):
        """Execute a robotic action based on the request data."""
        action_name = request.name  # e.g., "pour_espresso", "move_arm"
        params = {k: v for k, v in request.params.items()}
        try:
            # Call the corresponding ROS function (synchronously or asynchronously)
            result = robot_actions.perform(action_name, params)
            # Assume result is a dict like {"success": True/False, "error": "..." }
            if not result.get("success", True):
                # Return a failure status
                return robot_pb2.ActionResult(success=False, message=result.get("error", "unknown error"))
            # If success, optionally include any return data (like actual amount poured)
            return robot_pb2.ActionResult(success=True, message="OK")
        except Exception as e:
            # Catch any exceptions from the ROS calls
            return robot_pb2.ActionResult(success=False, message=str(e))

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    robot_pb2_grpc.add_RobotArmServiceServicer_to_server(RobotArmServicer(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()

if __name__ == "__main__":
    serve()
