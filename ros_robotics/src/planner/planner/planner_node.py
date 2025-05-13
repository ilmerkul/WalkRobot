import rclpy
from control.utils import get_joints
from interface.msg import AgrObs
from rclpy.node import Node


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.agr_obs = self.create_subscription(
            AgrObs, "/observation/aggregated_observation", self.observation_callback, 10
        )
        self.stand_up_pub = self.create_publisher(AgrObs, "stand_up", 10)
        self.move_pub = self.create_publisher(AgrObs, "move", 10)
        self.joint_order = get_joints()

    def observation_callback(self, msg: AgrObs):
        orientation = msg.imu.orientation
        orientation = [orientation.x, orientation.y, orientation.z]
        angular_velocity = msg.imu.angular_velocity
        angular_velocity = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
        linear_acceleration = msg.imu.linear_acceleration
        linear_acceleration = [
            linear_acceleration.x,
            linear_acceleration.y,
            linear_acceleration.z,
        ]
        foots_forces = msg.foots.forces
        position = msg.joint_states.position
        velocity = msg.joint_states.velocity

        if True:
            self.stand_up_pub.publish(msg)
        else:
            self.move_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
