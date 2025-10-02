#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import threading
import time
from collections import deque

# Waypoints (x, y, yaw). yaw unused for now
named_locations = {
    "Home": [-4.21, -5.64, 0.0],
    "Kitchen": [-11.69, -5.91, 0.0],
    "Table 1": [1.09, -3.11, 0.0],
    "Table 2": [1.19, 2.97, 0.0],
    "Table 3": [-1.55, 2.76, 0.0],
}

CONFIRM_TIMEOUT = 10.0  # single order
BATCH_TABLE_CONFIRM_TIMEOUT = 15.0  # batch mode


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')

        # Publishers / Subscribers
        self.status_pub = self.create_publisher(String, 'order_status', 10)
        self.order_sub = self.create_subscription(String, 'new_order', self._cb_new_order, 10)
        self.cancel_sub = self.create_subscription(String, 'cancel_order', self._cb_cancel_order, 10)
        self.kitchen_confirm_sub = self.create_subscription(String, 'confirm_kitchen', self._cb_kitchen_confirm, 10)
        self.table_confirm_sub = self.create_subscription(String, 'confirm_table', self._cb_table_confirm, 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Internal state
        self._queue = deque()
        self._queue_lock = threading.Lock()
        self._processing = False
        self._worker_thread = None

        # Active task
        self._current_table = None
        self._cancel_flag = False
        self._kitchen_confirmed = False
        self._table_confirmed = False
        self._current_goal_handle = None
        self._nav_goal_lock = threading.Lock()
        self._nav_was_cancelled = False
        self._in_batch = False

        self.get_logger().info("OrderManager ready")

    # --------------------------
    # ROS callbacks
    # --------------------------
    def _cb_new_order(self, msg: String):
        table = msg.data.strip()
        if not table:
            return
        with self._queue_lock:
            if table not in self._queue:
                self._queue.append(table)
        self._publish_log(f"New order queued: {table}")

        if not self._processing:
            self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
            self._processing = True
            self._worker_thread.start()

    def _cb_cancel_order(self, msg: String):
        table = msg.data.strip()
        if not table:
            return
        with self._queue_lock:
            if table in self._queue:
                self._queue = deque([t for t in self._queue if t != table])
                self._publish_log(f"Cancelled queued order: {table}")
                return

        if self._current_table == table:
            self._cancel_flag = True
            self._publish_log(f"Cancel requested for active order: {table}")
            with self._nav_goal_lock:
                if self._current_goal_handle is not None:
                    try:
                        self._current_goal_handle.cancel_goal_async()
                        self._publish_log("Sent cancel to Nav2 for current goal")
                    except Exception:
                        pass
        else:
            self._publish_log(f"Cancel request ignored (not in queue/current): {table}")

    def _cb_kitchen_confirm(self, msg: String):
        table = msg.data.strip()
        if self._current_table == table and not self._in_batch:
            self._kitchen_confirmed = True
            self._publish_log(f"Kitchen confirmed for {table}")

    def _cb_table_confirm(self, msg: String):
        table = msg.data.strip()
        if self._current_table == table:
            self._table_confirmed = True
            self._publish_log(f"Table confirmed for {table}")

    # --------------------------
    # Worker thread
    # --------------------------
    def _worker_loop(self):
        try:
            while True:
                with self._queue_lock:
                    if not self._queue:
                        break
                    if len(self._queue) > 1:
                        batch = [self._queue.popleft() for _ in range(len(self._queue))]
                    else:
                        batch = [self._queue.popleft()]

                if len(batch) == 1:
                    self._in_batch = False
                    self._process_single_order(batch[0])
                else:
                    self._in_batch = True
                    self._process_batch_orders(batch)
                    self._in_batch = False

            self._publish_log("All queued orders processed. Returning Home.")
            self._force_goto_home()
        finally:
            self._processing = False

    # --------------------------
    # Single order flow
    # --------------------------
    def _process_single_order(self, table):
        self._publish_log(f"Processing single order: {table}")
        self._current_table = table
        self._reset_flags()

        # Go to kitchen
        if not self._goto_and_wait("Kitchen"):
            self._publish_log("Failed to reach Kitchen -> returning Home")
            self._force_goto_home()
            return

        # Wait kitchen confirmation
        if not self._wait_flag_or_cancel(lambda: self._kitchen_confirmed, CONFIRM_TIMEOUT):
            self._publish_log("Kitchen confirmation TIMEOUT -> returning Home")
            self._force_goto_home()
            return

        # Check cancel
        if self._cancel_flag:
            self._publish_log("Canceled before going to table -> returning Kitchen then Home")
            self._goto_and_wait("Kitchen")
            self._force_goto_home()
            return

        # Go to table
        self._nav_was_cancelled = False
        if not self._goto_and_wait(table):
            if self._nav_was_cancelled:
                self._publish_log("Canceled while going to Table -> returning Kitchen then Home")
                self._goto_and_wait("Kitchen")
            self._force_goto_home()
            return

        # Wait table confirmation
        if not self._wait_flag_or_cancel(lambda: self._table_confirmed, CONFIRM_TIMEOUT):
            self._publish_log("Table confirmation TIMEOUT -> going to Kitchen then Home")
            self._goto_and_wait("Kitchen")
            self._force_goto_home()
            return

        self._publish_log(f"Delivered to {table} -> returning Home")
        self._force_goto_home()

    # --------------------------
    # Batch order flow
    # --------------------------
    def _process_batch_orders(self, batch_tables):
        self._publish_log(f"Processing batch orders: {batch_tables}")

        # Go to kitchen once
        if not self._goto_and_wait("Kitchen"):
            self._publish_log("Failed to reach Kitchen for batch -> returning Home")
            self._force_goto_home()
            return

        for table in batch_tables:
            self._current_table = table
            self._reset_flags()
            self._publish_log(f"Starting delivery to {table}...")

            self._nav_was_cancelled = False
            if not self._goto_and_wait(table):
                if self._nav_was_cancelled:
                    self._publish_log(f"Canceled while going to {table} -> skipping this table")
                    continue
                self._publish_log(f"Navigation failed for {table} -> skipping")
                continue

            if not self._wait_flag_or_cancel(lambda: self._table_confirmed, BATCH_TABLE_CONFIRM_TIMEOUT):
                self._publish_log(f"No confirmation at {table} after 15s -> skipping")
                continue

            self._publish_log(f"Delivered successfully at {table}")

        self._publish_log("Batch deliveries complete -> returning Kitchen then Home")
        self._goto_and_wait("Kitchen")
        self._force_goto_home()
        self._current_table = None

    # --------------------------
    # Wait helper
    # --------------------------
    def _wait_flag_or_cancel(self, flag_fn, timeout_s):
        start = time.time()
        while time.time() - start < timeout_s:
            if self._cancel_flag:
                return False
            if flag_fn():
                return True
            time.sleep(0.05)
        return False

    # --------------------------
    # Navigation helper
    # --------------------------
    def _goto_and_wait(self, location_name, nav_timeout=120.0):
        coords = named_locations.get(location_name)
        if not coords:
            self._publish_log(f"Unknown waypoint: {location_name}")
            return False
        x, y, _ = coords
        self._nav_was_cancelled = False

        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for Nav2 action server...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self.nav_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.01)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._publish_log(f"Nav2 rejected goal to {location_name}")
            return False

        with self._nav_goal_lock:
            self._current_goal_handle = goal_handle

        result_event = threading.Event()
        result_container = {"status": None}

        def _on_result(fut):
            try:
                res = fut.result()
                result_container["status"] = getattr(res, 'status', None)
            finally:
                result_event.set()

        goal_handle.get_result_async().add_done_callback(_on_result)

        waited = 0.0
        poll_dt = 0.05
        while waited < nav_timeout:
            if result_event.is_set():
                break
            if self._cancel_flag:
                try:
                    goal_handle.cancel_goal_async()
                    self._publish_log("Goal cancel requested to Nav2")
                except Exception:
                    pass
                self._nav_was_cancelled = True
            time.sleep(poll_dt)
            waited += poll_dt

        if not result_event.is_set():
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._publish_log(f"Navigation to {location_name} timed out/cancelled")
            with self._nav_goal_lock:
                self._current_goal_handle = None
            return False

        status = result_container["status"]
        with self._nav_goal_lock:
            self._current_goal_handle = None

        success = (status == 4)
        self._publish_log(f"Nav2 result for {location_name}: status={status} success={success}")
        return success

    # --------------------------
    # Force go home
    # --------------------------
    def _force_goto_home(self):
        if self._current_table != "Home":
            self._current_table = "Home"
            self._cancel_flag = False
            self._kitchen_confirmed = False
            self._table_confirmed = False
            self._goto_and_wait("Home")
        self._publish_log("Reached Home")
        self._current_table = None

    # --------------------------
    # Reset flags
    # --------------------------
    def _reset_flags(self):
        self._cancel_flag = False
        self._kitchen_confirmed = False
        self._table_confirmed = False

    # --------------------------
    # Logging
    # --------------------------
    def _publish_log(self, text: str):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = OrderManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
