import rclpy
from interface.msg import ControlSimMessage
from interface.srv import ControlSim
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity


class ControlSimNode(Node):
    def __init__(self):
        super().__init__("control_sim_node")

        self.control_world_service_name = "/world/world_demo/control"
        self.remove_world_service_name = "/world/world_demo/remove"

        self.srv = self.create_service(
            ControlSim,
            self.get_name() + "/command", 
            self.srv_callback)

        self.world_control_client = self.create_client(ControlWorld, self.control_world_service_name)
        self.world_remove_client = self.create_client(DeleteEntity, self.remove_world_service_name)

        flag_service = True
        while flag_service:
            flag_service = False
            for client, name in zip(
            [
                self.world_control_client,
                self.world_remove_client,
            ],
            [
                self.control_world_service_name,
                self.remove_world_service_name,
            ],
        ):
                if not self._check_service(client, name):
                    flag_service = True
            self.get_logger().info("Service not available, waiting...")

    def _check_service(self, client, service_name: str, timeout_sec: float=2.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"The {service_name} service is unavailable!")
            return False
        
        return True

    def srv_callback(self, request, response):
        try:
            msg_fail = f"Error request {request}: "

            if request.control_message.type == ControlSimMessage.PAUSE_MSG or request.control_message.type == ControlSimMessage.UNPAUSE_MSG:
            
                if not self._check_service(self.world_control_client, self.control_world_service_name):
                    msg_fail = f"{self.control_world_service_name} service not available"
                    raise RuntimeError(msg_fail)
                
                pause = (request.control_message.type == ControlSimMessage.PAUSE_MSG)

                control_request = ControlWorld.Request()
                control_request.world_control.pause = pause

                msg_success = f"Simulation {"paused" if pause else "unpaused"} successfully"
                msg_fail = f"Failed to {"pause" if pause else "unpause"} simulation: "

                response = self.call_service(self.world_control_client, control_request, response, msg_success)
            elif request.control_message.type == ControlSimMessage.RESET_MSG:
                if not self._check_service(self.world_control_client, self.control_world_service_name):
                    msg_fail = f"{self.control_world_service_name} service not available"
                    raise RuntimeError(msg_fail)

                control_request = ControlWorld.Request()
                control_request.world_control.reset.all = True

                msg_success = f"Simulation reset successfully"
                msg_fail = f"Failed to reset simulation: "

                response = self.call_service(self.world_control_client, control_request, response, msg_success)
            elif request.control_message.type == ControlSimMessage.REMOVE_MSG:
                if not self._check_service(self.world_remove_client, self.control_world_service_name):
                    msg_fail = f"{self.control_world_service_name} service not available"
                    raise RuntimeError(msg_fail)

                remove_request = DeleteEntity.Request()
                remove_request.entity.name = request.entity_name
                remove_request.entity.type = Entity.MODEL

                msg_success = f"Simulation remove model {request.entity_name} successfully"
                msg_fail = f"Failed to remove model {request.entity_name} simulation: "

                response = self.call_service(self.world_remove_client, remove_request, response, msg_success)
            else:
                msg_fail = f"Unknown command: {request.control_message}"
                raise RuntimeError(msg_fail)
                
        except Exception as e:
            response.success = False
            response.message = msg_fail
            self.get_logger().error(msg_fail + str(e), throttle_duration_sec=5)

        return response

    def call_service(
        self, client, request, response, msg_success: str, timeout_sec: float = 20.0
    ):
        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
                
        if not future.done():
            raise RuntimeError(f"Service call to {client.srv_name} timed out after {timeout_sec} seconds")
            
        service_response = future.result()
        if service_response is None:
            raise RuntimeError(f"Service {client.srv_name} returned None response")
            
        response.success = True
        response.message = msg_success
        self.get_logger().info(msg_success)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControlSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
