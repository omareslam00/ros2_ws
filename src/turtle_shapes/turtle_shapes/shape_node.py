#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.pub = self.create_publisher(String, 'shape', 10)
        self.get_logger().info("ShapeNode started. Enter: 1=heart,2=flower,3=star,0=reset,4=stop")
        # run input in separate thread so rclpy.spin() stays responsive
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def input_loop(self):
        mapping = {
            '1': 'heart',
            '2': 'flower',
            '3': 'star',
            '0': 'reset',
            '4': 'stop',
            'heart': 'heart',
            'flower': 'flower',
            'star': 'star',
            'reset': 'reset',
            'stop': 'stop',
            'q': 'reset',
            'exit': 'reset'
        }
        try:
            while rclpy.ok():
                try:
                    raw = input("Enter shape (1=heart,2=flower,3=star,0=reset,4=stop): ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break
                if raw == '':
                    continue
                shape = mapping.get(raw)
                if shape:
                    msg = String()
                    msg.data = shape
                    self.pub.publish(msg)
                    self.get_logger().info(f"Published: {shape}")
                else:
                    print("Invalid input. Use 1/2/3/0/4 or words.")
        except Exception as e:
            self.get_logger().error(f"Input loop error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

