import rclpy
from rclpy.node import Node

from truck_interfaces.msg import TruckCmd

import can


class Truck_CAN_Node(Node):
    def __init__(self):
        super().__init__('truck_control_node')
        
        # Initialize CAN connection
        try:
            self.bus = can.interface.Bus(bustype="socketcan", channel="can1", bitrate=500000)
            self.get_logger().info("CAN Bus initialized successfully on PCAN_USBBUS1")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN Bus: {str(e)}")
            raise e

        # Initialize variables
        self.last_msg_time = self.get_clock().now()
        self.throttle =0
        self.steering =0

        # Initialize tasks
        self.subscription = self.create_subscription(TruckCmd, 'truck_cmd', self.cmd_callback, 10)
        self.create_timer(0.02, self.Control_Callback) #run at 50Hz

        self.get_logger().info("Truck_CAN Node Started")


    def cmd_callback(self, msg):
        #Save newest request
        self.throttle = int(max(-100, min(100, msg.throttle)))
        self.steering = int(max(-4500, min(4500, msg.steering)))
        #Save time of message
        self.last_msg_time = self.get_clock().now()


    def Control_Callback(self):
        #Check how old latest request is
        now = self.get_clock().now()
        age = now - self.last_msg_time
        age_seconds = age.nanoseconds / 1e9

        #Set to zero if latest message was not recent
        if age_seconds > 0.5:
            self.throttle = 0
            self.steering = 0
            self.get_logger().warn(f"TIMEOUT: No command for {age_seconds:.2f}s! Zeroing outputs.", throttle_duration_sec=1.0)

        #Send over USB-CAN
        self.send_can(0x01, self.throttle)
        self.send_can(0x02, self.steering)

        self.get_logger().info(f"Steer: {self.steering} | Throttle: {self.throttle}", throttle_duration_sec=0.2)


    def send_can(self, can_id, value):
        # Convert to 2-byte signed little-endian
        data = int(value).to_bytes(2, "little", signed=True)
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().warn(f"CAN send failed for ID {hex(can_id)}")


        
def main(args=None):
    rclpy.init(args=args)
    node = Truck_CAN_Node()
    
    try:   
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User stopped the node.")
    except Exception as e:
        node.get_logger().error(f"Node crashed with error: {e}")
    finally:
        node.bus.shutdown()
        rclpy.shutdown()