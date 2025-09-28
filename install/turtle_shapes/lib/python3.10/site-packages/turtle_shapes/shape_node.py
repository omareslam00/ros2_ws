#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.pub = self.create_publisher(String, 'shape', 10)
        self._stop = False
        self.get_logger().info("ShapeNode ready. Press: 1=heart, 2=flower, 3=star, 0=reset")
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def input_loop(self):
        mapping = {
            '1': 'heart',
            '2': 'flower',
            '3': 'star',
            '0': 'reset',
            'heart': 'heart',
            'flower': 'flower',
            'star': 'star',
            'reset': 'reset',
            'q': 'reset',
            'exit': 'reset'
        }
        try:
            while rclpy.ok() and not self._stop:
                try:
                    raw = input("Enter shape (1=heart,2=flower,3=star,0=reset): ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break
                if raw == '':
                    continue
                if raw in mapping:
                    msg = String()
                    msg.data = mapping[raw]
                    self.pub.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")
                else:
                    print("❌ Invalid input. Use 1,2,3 or 0.")
        except Exception as e:
            self.get_logger().error(f"Input loop error: {e}")

    def destroy_node(self):
        self._stop = True
        super().destroy_node()

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
