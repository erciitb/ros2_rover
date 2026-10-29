#!/usr/bin/env python3
"""Autonomous exploration: pick random reachable goals from the /map grid, drive
to them with Nav2, and save a camera image at each one (bonus challenges 1 & 2).
"""

import os
import time
import random
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
import tf2_ros

try:
    from cv_bridge import CvBridge
    import cv2
    HAVE_CV = True
except Exception:
    HAVE_CV = False


class Explorer(Node):
    def __init__(self):
        super().__init__("explorer_node")

        self.num_goals = self.declare_parameter("num_goals", 0).value  # 0 = run forever
        self.goal_timeout = self.declare_parameter("goal_timeout", 120.0).value
        self.min_goal_distance = self.declare_parameter("min_goal_distance", 1.5).value
        self.max_goal_distance = self.declare_parameter("max_goal_distance", 5.0).value
        self.obstacle_margin = self.declare_parameter("obstacle_margin_cells", 3).value
        # keep goals inside the finite terrain, or the rover drives off the edge
        self.max_range_from_start = self.declare_parameter("max_range_from_start", 8.0).value
        self.start_xy = None
        self.capture_images = self.declare_parameter("capture_images", True).value
        self.image_dir = self.declare_parameter(
            "image_dir", os.path.expanduser("~/ros2_rover/captured_images")).value

        self.map = None
        self.latest_image = None
        self.bridge = CvBridge() if HAVE_CV else None
        self._stop = False

        map_qos = QoSProfile(  # /map is latched by the SLAM node
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        img_qos = QoSProfile(  # camera publishes BEST_EFFORT
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, "/camera/image_raw", self._image_cb, img_qos)

        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.capture_images and not HAVE_CV:
            self.get_logger().warn("cv_bridge unavailable, image capture disabled")
            self.capture_images = False

    def _map_cb(self, msg):
        self.map = msg

    def _image_cb(self, msg):
        self.latest_image = msg

    def current_xy(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _window_free(self, data, w, h, col, row, margin):
        for r in range(row - margin, row + margin + 1):
            base = r * w
            for c in range(col - margin, col + margin + 1):
                if data[base + c] != 0:
                    return False
        return True

    def sample_goal(self):
        m = self.map
        if m is None:
            return None
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = m.data
        margin = self.obstacle_margin
        if w <= 2 * margin or h <= 2 * margin:
            return None
        cur = self.current_xy()
        for _ in range(300):
            col = random.randint(margin, w - 1 - margin)
            row = random.randint(margin, h - 1 - margin)
            if data[row * w + col] != 0:
                continue
            if not self._window_free(data, w, h, col, row, margin):
                continue
            x = ox + (col + 0.5) * res
            y = oy + (row + 0.5) * res
            if cur is not None:
                d2 = (x - cur[0]) ** 2 + (y - cur[1]) ** 2
                if d2 < self.min_goal_distance ** 2 or d2 > self.max_goal_distance ** 2:
                    continue
            if self.start_xy is not None:
                if (x - self.start_xy[0]) ** 2 + (y - self.start_xy[1]) ** 2 > self.max_range_from_start ** 2:
                    continue
            return (x, y)
        return None

    def _wait_future(self, fut, timeout):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.1)
        return True

    def go_to(self, xy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.w = 1.0

        send_fut = self.nav.send_goal_async(goal)
        if not self._wait_future(send_fut, 10.0):
            self.get_logger().warn("goal not acknowledged in time")
            return False
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().warn("goal rejected by server")
            return False

        result_fut = handle.get_result_async()
        if not self._wait_future(result_fut, self.goal_timeout):
            self.get_logger().warn("goal timed out, cancelling")
            handle.cancel_goal_async()
            return False
        status = result_fut.result().status
        names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                 GoalStatus.STATUS_ABORTED: "ABORTED",
                 GoalStatus.STATUS_CANCELED: "CANCELED"}
        self.get_logger().info(f"navigation result: {names.get(status, status)}")
        return status == GoalStatus.STATUS_SUCCEEDED

    def save_image(self, n):
        if not self.capture_images:
            return
        if self.latest_image is None:
            self.get_logger().warn("no camera image yet, nothing to save")
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            os.makedirs(self.image_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = f"image_{n:03d}_{stamp}.png"
            cv2.imwrite(os.path.join(self.image_dir, fname), cv_img)
            self.get_logger().info(f"captured {fname}")
        except Exception as e:
            self.get_logger().warn(f"image save failed: {e}")

    def run(self):
        self.get_logger().info("waiting for NavigateToPose action server...")
        self.nav.wait_for_server()
        self.get_logger().info("waiting for /map...")
        while self.map is None and rclpy.ok():
            time.sleep(0.5)
        while self.start_xy is None and rclpy.ok():
            self.start_xy = self.current_xy()
            if self.start_xy is None:
                time.sleep(0.3)
        self.get_logger().info(
            f"starting exploration within {self.max_range_from_start:.1f} m of {self.start_xy}")

        reached = 0
        while rclpy.ok() and not self._stop:
            goal = self.sample_goal()
            if goal is None:
                self.get_logger().info("no valid goal found, retrying...")
                time.sleep(1.0)
                continue
            self.get_logger().info(f"goal #{reached + 1} -> map({goal[0]:.2f}, {goal[1]:.2f})")
            if self.go_to(goal):
                reached += 1
                self.get_logger().info(f"goal reached ({reached} total)")
                self.save_image(reached)
            else:
                self.get_logger().info("goal not reached, resampling")
            if self.num_goals and reached >= self.num_goals:
                self.get_logger().info(f"completed {reached} goals")
                break


def main():
    import signal
    rclpy.init()
    node = Explorer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    def _sig(*_):
        node._stop = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        node.run()
    except KeyboardInterrupt:
        node._stop = True
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
