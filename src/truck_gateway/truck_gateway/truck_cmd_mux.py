import rclpy
from rclpy.node import Node
from truck_interfaces.msg import TruckCmd
from std_msgs.msg import Int16


class TruckCmdMux(Node):
    def __init__(self):
        super().__init__('truck_cmd_mux')

        self.joy_sub = self.create_subscription(TruckCmd, 'truck_cmd', self.joy_callback, 10)
        self.safety_sub = self.create_subscription(Int16, 'safety_signal', self.safety_callback, 10)
        
        self.pub = self.create_publisher(TruckCmd, 'truck_cmd', 10)

        # Variables
        self.latest_joy_cmd = TruckCmd()
        self.safety_state = 0  # Default to 0 (Stop)

        self.get_logger().info("Truck Mux Node Started")

    def safety_callback(self, msg):
        self.safety_state = msg.data
        self.publish_muxed_cmd()

    def joy_callback(self, msg):
        self.latest_joy_cmd = msg
        self.publish_muxed_cmd()

        
    def publish_muxed_cmd(self):
        out_msg = self.latest_joy_cmd

        #Safety check
        match self.safety_state:
            case 0:
                out_msg.throttle = 0.0
                self.get_logger().warn("Stop engaged - 0% throttle", throttle_duration_sec=0.2)

            case 1:
                out_msg.throttle = out_msg.throttle * 0.5
                self.get_logger().warn("Stop engaged - 50% throttle", throttle_duration_sec=0.2)
                
            case 2:
                out_msg.throttle = out_msg.throttle

        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TruckCmdMux()

    try:   
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User stopped the node.")
    except Exception as e:
        node.get_logger().error(f"Node crashed with error: {e}")
    finally:
        rclpy.shutdown()