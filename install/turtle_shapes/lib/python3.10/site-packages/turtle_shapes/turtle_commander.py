#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen
import math
import time
import threading

def angle_diff(a):
    """normalize angle to [-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        self.sub_shape = self.create_subscription(String, 'shape', self.listener_callback, 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.pub_cmd = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose = None
        self.draw_thread = None
        self.running = False
        self.stop_requested = False
        self.lock = threading.Lock()

        self.get_logger().info("TurtleCommander ready. Shapes: heart, flower, star, reset")

    def update_pose(self, msg: Pose):
        self.pose = msg

    def listener_callback(self, msg: String):
        shape = msg.data.strip().lower()
        self.get_logger().info(f"Received command: '{shape}'")
        if shape == 'reset':
            self.reset_world()
            self.reset_world()
            return

        if shape == 'heart':
            points = self.heart_points()
        elif shape == 'flower':
            points = self.flower_points()
        elif shape == 'star':
            points = self.star_points()
        else:
            self.get_logger().warn(f"Unknown command '{shape}'")
            return

        with self.lock:
            if self.running:
                self.request_stop()
                if self.draw_thread is not None:
                    self.draw_thread.join(timeout=0.5)

            self.stop_requested = False
            self.draw_thread = threading.Thread(
                target=self._draw_shape_thread, args=(points,), daemon=True
            )
            self.draw_thread.start()

    def request_stop(self):
        with self.lock:
            self.stop_requested = True

    # ==========================
    # RESET FUNCTION
    # ==========================
    def reset_world(self):
        self.get_logger().info("Resetting world...")

        # Clear background
        clear_client = self.create_client(Empty, '/clear')
        while not clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /clear service...")
        clear_client.call_async(Empty.Request())

        # Teleport turtle to center
        tp_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not tp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /turtle1/teleport_absolute service...")
        req = TeleportAbsolute.Request()
        req.x = 5.5
        req.y = 5.5
        req.theta = 0.0
        tp_client.call_async(req)

        # Pen down again
        self.set_pen(off=0)

    def set_pen(self, r=0, g=0, b=0, width=3, off=0):
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        client.call_async(req)

    # ==========================
    # SHAPE GENERATORS
    # ==========================

    def heart_points(self, steps=350, scale=0.18, rotate=0.0):
        center_x, center_y = 5.5, 5.5
        pts = []
        for i in range(steps):
            t = 2.0 * math.pi * i / steps
            x = 16.0 * math.sin(t)**3
            y = 13.0*math.cos(t) - 5.0*math.cos(2*t) - 2.0*math.cos(3*t) - math.cos(4*t)
            xr = x*math.cos(rotate) - y*math.sin(rotate)
            yr = x*math.sin(rotate) + y*math.cos(rotate)
            pts.append((center_x + xr*scale, center_y + yr*scale))
        return pts
    def flower_points(self, petals=6, steps=400, scale=2.6, rotate=0.0, stem_points=30):
        center_x, center_y = 5.5, 5.5
        petals_pts = []
        for i in range(steps):
            t = 2.0 * math.pi * i / steps
            r = math.cos(petals * t)
            x = r * math.cos(t)
            y = r * math.sin(t)
            xr = x*math.cos(rotate) - y*math.sin(rotate)
            yr = x*math.sin(rotate) + y*math.cos(rotate)
            petals_pts.append((center_x + xr*scale, center_y + yr*scale))

         # find bottom of petals
        min_y = min(y for (_, y) in petals_pts)
        stem_start_x = center_x
        stem_start_y = min_y

      # marker: move silently to bottom of petals
        petals_pts.append(("MOVE", stem_start_x, stem_start_y))

      # stem points
        stem_pts = []
        end_y = 0.5  # bottom of screen
        for i in range(stem_points):
            y = stem_start_y + (end_y - stem_start_y) * (i / max(1, stem_points-1))
            stem_pts.append((stem_start_x, y))

        return petals_pts + stem_pts


    def star_points(self, spikes=5, outer_r=3.5, inner_r=1.4, rotate=math.pi/2, points_per_edge=20):
        """5-pointed star rotated upside down"""
        center_x, center_y = 5.5, 5.5
        vertices = []
        for i in range(spikes * 2):
            r = outer_r if (i % 2 == 0) else inner_r
            angle = rotate + i * math.pi / spikes
            x = center_x + r * math.cos(angle)
            y = center_y + r * math.sin(angle)
            vertices.append((x, y))
        vertices.append(vertices[0])

        pts = []
        for i in range(len(vertices)-1):
            x1, y1 = vertices[i]
            x2, y2 = vertices[i+1]
            for k in range(points_per_edge):
                t = k / float(points_per_edge)
                pts.append((x1*(1.0-t) + x2*t, y1*(1.0-t) + y2*t))
        pts.append(vertices[0])
        return pts

    # ==========================
    # DRAWING THREAD
    # ==========================
    def _draw_shape_thread(self, points):
        self.get_logger().info("Starting FAST draw thread...")
        self.running = True
        try:
            if not points:
                return

            self.set_pen(off=1)  # pen up
            first_x, first_y = points[0]
            if not (isinstance(first_x, str) and first_x == "MOVE"):
                self._move_to_point(first_x, first_y)
            self.set_pen(off=0)  # pen down

            for pt in points[1:]:
                if self.stop_requested:
                    break
                if self.pose is None:
                    continue

                # handle MOVE marker
                if isinstance(pt, tuple) and len(pt) == 3 and pt[0] == "MOVE":
                    _, tx, ty = pt
                    self.set_pen(off=1)
                    self._move_to_point(tx, ty)
                    self.set_pen(off=0)
                    continue

                tx, ty = pt
                reached = False
                while not reached and not self.stop_requested:
                    dx = tx - self.pose.x
                    dy = ty - self.pose.y
                    dist = math.hypot(dx, dy)
                    angle_to_goal = math.atan2(dy, dx)
                    ang_err = angle_diff(angle_to_goal - self.pose.theta)

                    # 🚀 Faster gains and limits
                    kp_lin = 5.0
                    kp_ang = 10.0
                    max_lin = 12.0
                    max_ang = 14.0
                    if abs(dx) < 0.2:
                        max_lin = 14.0

                    if abs(ang_err) > 0.8:
                        lin = 0.0
                    else:
                        lin = min(max_lin, kp_lin * dist)

                    ang = max(-max_ang, min(max_ang, kp_ang * ang_err))

                    if dist < 0.15:
                        reached = True
                        break

                    msg = Twist()
                    msg.linear.x = lin
                    msg.angular.z = ang
                    self.pub_cmd.publish(msg)

                    time.sleep(0.005)

                if self.stop_requested:
                    break

            self.set_pen(off=1)  # pen up at end

        finally:
            self.stop_turtle()
            self.running = False
            self.get_logger().info("FAST draw thread finished.")

    def _move_to_point(self, tx, ty):
        if self.pose is None:
            return
        reached = False
        while not reached and not self.stop_requested:
            dx = tx - self.pose.x
            dy = ty - self.pose.y
            dist = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            ang_err = angle_diff(angle_to_goal - self.pose.theta)

            # 🚀 Faster move
            kp_lin = 5.0
            kp_ang = 10.0
            max_lin = 12.0
            max_ang = 14.0

            if abs(ang_err) > 0.6:
                lin = 0.0
            else:
                lin = min(max_lin, kp_lin * dist)

            ang = max(-max_ang, min(max_ang, kp_ang * ang_err))

            if dist < 0.1:
                reached = True
                break

            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            self.pub_cmd.publish(msg)
            time.sleep(0.005)

    def stop_turtle(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(2):
            self.pub_cmd.publish(msg)
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_turtle()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
