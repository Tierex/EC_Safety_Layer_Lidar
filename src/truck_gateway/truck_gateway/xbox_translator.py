import rclpy
from rclpy.node import Node
from truck_interfaces.msg import TruckCmd

from sensor_msgs.msg import Joy


class XboxTranslator(Node):
    def __init__(self):
        super().__init__('xbox_translator')

        self.pub = self.create_publisher(TruckCmd, 'truck_cmd', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.get_logger().info("Xbox Translator Node Started")


    def joy_callback(self, msg):
        cmd = TruckCmd()

        # ===== Throttle =====
        rt_raw = msg.axes[5]  # right trigger
        lt_raw = msg.axes[4]  # left trigger

        rt_val = (1.0 - rt_raw) / 2.0
        lt_val = (1.0 - lt_raw) / 2.0

        if rt_val > 0.05:
            cmd.throttle = rt_val * -100
        elif lt_val > 0.05:
            cmd.throttle = lt_val * 100


        # ===== STEERING =====
        if abs(msg.axes[0]) > 0.2:
            cmd.steering = -msg.axes[0] * 4500
        else:
            cmd.steering = 0.0


        # ==== Deadman switch ====
        if not msg.buttons[0]:  # A pressed
            cmd.throttle = 0.0

        # ===== Send message =====
        self.pub.publish(cmd)




def main(args=None):
    rclpy.init(args=args)
    node = XboxTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User stopped the node.")
    except Exception as e:
        node.get_logger().error(f"Node crashed with error: {e}")
    finally:
        rclpy.shutdown()


