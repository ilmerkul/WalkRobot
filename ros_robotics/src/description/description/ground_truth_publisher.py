import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthTfPublisher(Node):
    def __init__(self):
        super().__init__("ground_truth_tf_publisher")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Pose, "/ground_truth/pose", self.pose_callback, 10
        )
        self.subscription

    def pose_callback(self, msg):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = msg.position.x
        transform.transform.translation.y = msg.position.y
        transform.transform.translation.z = msg.position.z

        transform.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug(
            f"Опубликовано TF: map -> base_link", throttle_duration_sec=1
        )


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
