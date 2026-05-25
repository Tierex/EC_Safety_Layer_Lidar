import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rviz_2d_overlay_msgs.msg import OverlayText

class SafetyOverlayNode(Node):
    def __init__(self):
        super().__init__('safety_overlay_node')
        self.sub = self.create_subscription(Int16, 'safety_signal', self.callback, 10)
        self.pub = self.create_publisher(OverlayText, 'safety_text_overlay', 10)

    def callback(self, msg):
            overlay = OverlayText()
 
            # 1. Map the integer to text and colors
            if msg.data == 0:
                overlay.text = "STATUS: STOP"
                overlay.fg_color.r, overlay.fg_color.g, overlay.fg_color.b, overlay.fg_color.a = 1.0, 0.0, 0.0, 1.0 # Red
            elif msg.data == 1:
                overlay.text = "STATUS: WARNING"
                overlay.fg_color.r, overlay.fg_color.g, overlay.fg_color.b, overlay.fg_color.a = 1.0, 0.5, 0.0, 1.0 # Orange
            else:
                overlay.text = "STATUS: GO"
                overlay.fg_color.r, overlay.fg_color.g, overlay.fg_color.b, overlay.fg_color.a = 0.0, 1.0, 0.0, 1.0 # Green

            # 2. Appearance settings
            overlay.bg_color.r, overlay.bg_color.g, overlay.bg_color.b, overlay.bg_color.a = 0.0, 0.0, 0.0, 0.4 # Semi-transparent black
            overlay.font = "DejaVu Sans Mono"
            overlay.line_width = 2
            overlay.text_size = 20.0

            # 3. Positioning
            overlay.horizontal_alignment = OverlayText.RIGHT
            overlay.vertical_alignment = OverlayText.TOP
            overlay.horizontal_distance = 0
            overlay.vertical_distance = 0
            overlay.width = 260
            overlay.height = 35

            
            
            self.pub.publish(overlay)

def main():
    rclpy.init()
    node = SafetyOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()