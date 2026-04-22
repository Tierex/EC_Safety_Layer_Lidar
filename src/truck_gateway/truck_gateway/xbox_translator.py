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
        rt = msg.axes[5]  # right trigger
        lt = msg.axes[4]  # left trigger

        rt_val = max(0, (rt + 1) / 2)
        lt_val = max(0, (lt + 1) / 2)

        if rt_val > 0.05:
            cmd.throttle = int(rt_val * 100)
        elif lt_val > 0.05:
            cmd.throttle = int(-lt_val * 100)


        # ===== STEERING =====
        if msg.axes[0] < -0.2:
            cmd.steering = int(msg.axes[0] * 4500)
        elif msg.axes[0] > 0.2:
            cmd.steering = int(msg.axes[0] * 4500)
        else:
            cmd.steering = 0


        # ==== Deadman switch ====
        if not msg.buttons[0]:  # A pressed
            cmd.throttle = 0

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


