import rclpy
from rclpy.node import Node
from vision_msgs.msg import TargetCoord
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
class Test_node(Node):
    def __init__(self):
        super().__init__('test_node')
        offboard_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.vis_sub = self.create_subscription(TargetCoord,"/vision/custom_target",self.vision_callback,offboard_qos)

    def vision_callback(self,msg):
        if msg.conf < 0.5:
            return
        print(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Test_node()
    node.get_logger().info('你好')
    rclpy.spin(node)
    rclpy.shutdown()
