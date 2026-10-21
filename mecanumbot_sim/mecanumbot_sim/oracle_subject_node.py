import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node


class MecanumbotSimOracleSubjectNode(Node):
    def __init__(self, namespace=""):
        super().__init__("mecanumbot_sim_oracle_subject_node", namespace=namespace)

        self.declare_parameter("source_topic", "/sim/subject_pose_ground_truth")
        self.declare_parameter("target_topic", "subject_pose")
        # The leading trees read the person off people_fusion alone, so the
        # ground truth goes there too, as a one-person PoseArray. Empty
        # disables it, for a run that also starts the real fusion.
        self.declare_parameter("people_topic", "people_fusion")

        source_topic = str(self.get_parameter("source_topic").value)
        target_topic = str(self.get_parameter("target_topic").value)
        people_topic = str(self.get_parameter("people_topic").value)

        self.publisher = self.create_publisher(PoseStamped, target_topic, 10)
        self.people_publisher = (
            self.create_publisher(PoseArray, people_topic, 10) if people_topic else None
        )
        self.create_subscription(PoseStamped, source_topic, self.subject_callback, 10)

        self.get_logger().info(
            f"Oracle subject bridge active: {source_topic} -> {target_topic}"
            + (f", {people_topic}" if people_topic else "")
        )

    def subject_callback(self, msg: PoseStamped) -> None:
        self.publisher.publish(msg)
        if self.people_publisher is not None:
            people = PoseArray()
            people.header = msg.header
            people.poses.append(msg.pose)
            self.people_publisher.publish(people)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimOracleSubjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
