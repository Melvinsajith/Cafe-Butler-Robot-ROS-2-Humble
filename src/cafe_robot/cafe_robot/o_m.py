#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import threading
import time
from collections import deque

# Waypoints (x,y,yaw). yaw is unused for now (we send w=1.0 quaternion)
named_locations = {
    "Home": [-4.21, -5.64, 0.0],
    "Kitchen": [-11.69, -5.91, 0.0],
    "Table 1": [1.09, -3.11, 0.0],
    "Table 2": [1.19, 2.97, 0.0],
    "Table 3": [-1.55, 2.76, 0.0],
}

# How long to wait for confirmations (seconds)
CONFIRM_TIMEOUT = 10.0


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
        self._queue = deque()           # pending orders
        self._queue_lock = threading.Lock()
        self._processing = False        # is worker thread active?
        self._worker_thread = None

        # Active task variables (accessed from both threads; protect where needed)
        self._current_table = None
        self._cancel_flag = False
        self._kitchen_confirmed = False
        self._table_confirmed = False

        # Nav goal handle (for canceling)
        self._current_goal_handle = None
        self._nav_goal_lock = threading.Lock()

        self.get_logger().info("OrderManager ready")

    # --------------------------
    # ROS callbacks (main thread)
    # --------------------------
    def _cb_new_order(self, msg: String):
        table = msg.data.strip()
        if table == "":
            return
        with self._queue_lock:
            # avoid duplicate queue entries
            if table not in self._queue:
                self._queue.append(table)
        self._publish_log(f"New order queued: {table}")

        # ensure worker started
        if not self._processing:
            self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
            self._processing = True
            self._worker_thread.start()

    def _cb_cancel_order(self, msg: String):
        table = msg.data.strip()
        if table == "":
            return
        # If the table is in queue, remove it; if it's current, set cancel flag
        with self._queue_lock:
            if table in self._queue:
                self._queue = deque([t for t in self._queue if t != table])
                self._publish_log(f"Cancelled queued order: {table}")
                return

        # if currently processing this table, set cancel flag (worker will handle)
        if self._current_table == table:
            self._cancel_flag = True
            self._publish_log(f"Cancel requested for active order: {table}")
            # also cancel nav goal if running
            with self._nav_goal_lock:
                if self._current_goal_handle is not None:
                    try:
                        self._current_goal_handle.cancel_goal_async()
                        self._publish_log("Sent cancel to Nav2 for current goal")
                    except Exception:
                        # ignore if cancel fails
                        pass
        else:
            # not in queue and not current
            self._publish_log(f"Cancel request ignored (not in queue/current): {table}")

    def _cb_kitchen_confirm(self, msg: String):
        table = msg.data.strip()
        # only accept if it's the current order (single-order path) or batch mode's table while active
        if self._current_table == table:
            self._kitchen_confirmed = True
            self._publish_log(f"Kitchen confirmed for {table}")

    def _cb_table_confirm(self, msg: String):
        table = msg.data.strip()
        if self._current_table == table:
            self._table_confirmed = True
            self._publish_log(f"Table confirmed for {table}")

    # --------------------------
    # Worker thread (process orders)
    # --------------------------
    def _worker_loop(self):
        try:
            while True:
                # get a snapshot of queue
                with self._queue_lock:
                    if not self._queue:
                        # no more orders -> done
                        break
                    # decide batch mode: if more than 1 queued at the start, treat them as a batch
                    if len(self._queue) > 1:
                        # pop all current items as batch
                        batch = []
                        while self._queue:
                            batch.append(self._queue.popleft())
                    else:
                        # single order: pop one
                        batch = [self._queue.popleft()]

                # Process batch
                if len(batch) == 1:
                    # single-order behavior (kitchen confirm, table confirm)
                    table = batch[0]
                    self._process_single_order(table)
                else:
                    # multi-order behavior: go to kitchen once, then deliver to all tables
                    self._process_batch_orders(batch)

            # Finished processing
            self._publish_log("All queued orders processed. Returning Home.")
            self._goto_and_wait("Home")
            self._publish_log("Reached Home")
        finally:
            self._processing = False

    # --------------------------
    # Single order flow
    # --------------------------
    def _process_single_order(self, table):
        self._publish_log(f"Processing single order: {table}")
        self._current_table = table
        self._cancel_flag = False
        self._kitchen_confirmed = False
        self._table_confirmed = False

        # 1) Go to Kitchen
        ok = self._goto_and_wait("Kitchen")
        if not ok:
            self._publish_log("Failed to reach Kitchen -> returning Home")
            self._goto_and_wait("Home")
            self._current_table = None
            return

        # Wait for kitchen confirm or timeout
        if not self._wait_flag_or_cancel(lambda: self._kitchen_confirmed, CONFIRM_TIMEOUT):
            # no confirmation -> return home
            self._publish_log("Kitchen confirmation TIMEOUT -> returning Home")
            self._goto_and_wait("Home")
            self._current_table = None
            return

        # 2) Go to Table
        # If cancel was requested while waiting, handle it
        if self._cancel_flag:
            self._publish_log("Canceled before going to table -> returning Home")
            self._goto_and_wait("Home")
            self._current_table = None
            return

        ok = self._goto_and_wait(table)
        if not ok:
            self._publish_log(f"Failed to reach {table} -> returning Home")
            self._goto_and_wait("Home")
            self._current_table = None
            return

        # Wait for table confirm or timeout
        if not self._wait_flag_or_cancel(lambda: self._table_confirmed, CONFIRM_TIMEOUT):
            self._publish_log("Table confirmation TIMEOUT -> going to Kitchen then Home")
            self._goto_and_wait("Kitchen")
            self._goto_and_wait("Home")
            self._current_table = None
            return

        # Completed
        self._publish_log(f"Delivered to {table} -> returning Home")
        self._goto_and_wait("Home")
        self._publish_log("Reached Home")
        self._current_table = None

    # --------------------------
    # Batch flow (multiple orders)
    # --------------------------
    def _process_batch_orders(self, batch_tables):
        # batch_tables: list of table names
        self._publish_log(f"Processing batch orders: {batch_tables}")

        # Go to kitchen once (no kitchen confirm required for batch mode)
        ok = self._goto_and_wait("Kitchen")
        if not ok:
            self._publish_log("Failed to reach Kitchen for batch -> returning Home")
            self._goto_and_wait("Home")
            return

        # Deliver each table in the batch in order. Skip canceled ones.
        for table in list(batch_tables):  # iterate over snapshot
            # check canceled
            with self._queue_lock:
                canceled = (table in self._queue and False)  # irrelevant; canceled tracked via cancel_flag only for active
            # if user canceled before batch started, remove (we removed duplicates earlier)
            # set current table
            self._current_table = table
            self._cancel_flag = False
            self._table_confirmed = False

            # If this table was canceled earlier (not in queue - we don't keep a canceled set separate here),
            # GUI cancel will set cancel_flag if active; if canceled before activate, it was removed from queue earlier.
            # Start nav
            ok = self._goto_and_wait(table)
            if not ok:
                self._publish_log(f"Could not reach {table}, skipping it")
                continue

            # Wait for table confirmation (10s). If none: skip to next table.
            if not self._wait_flag_or_cancel(lambda: self._table_confirmed, CONFIRM_TIMEOUT):
                # No confirmation: skip and go next table
                self._publish_log(f"No confirmation at {table} -> skipping to next table")
                continue

            # Delivered
            self._publish_log(f"Delivered to {table} (batch)")

        # After finishing batch, go to kitchen then home
        self._publish_log("Batch done -> returning to Kitchen then Home")
        self._goto_and_wait("Kitchen")
        self._goto_and_wait("Home")
        self._publish_log("Reached Home (batch)")
        self._current_table = None

    # --------------------------
    # Helper: wait for a boolean function or handle cancel
    # --------------------------
    def _wait_flag_or_cancel(self, flag_fn, timeout_s):
        """Wait up to timeout_s seconds for flag_fn() to become True.
           If self._cancel_flag becomes True, returns False immediately.
           Returns True if flag_fn() became True, False otherwise."""
        start = time.time()
        while time.time() - start < timeout_s:
            if self._cancel_flag:
                return False
            if flag_fn():
                return True
            time.sleep(0.05)
        return False

    # --------------------------
    # Navigation (send goal, wait for result). Safe to call from worker thread.
    # --------------------------
    def _goto_and_wait(self, location_name, nav_timeout=120.0):
        """Send a NavigateToPose goal and wait for result (or cancellation).
           Returns True on success (status == SUCCEEDED), False otherwise.
           Also publishes logs to order_status.
        """
        coords = named_locations.get(location_name)
        if not coords:
            self._publish_log(f"Unknown waypoint: {location_name}")
            return False
        x, y, _yaw = coords

        # Wait for server
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for Nav2 action server...")

        # Build goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        # Send goal asynchronously
        send_future = self.nav_client.send_goal_async(goal_msg)

        # Wait for send_future to complete (it will be set by rclpy executor in main thread)
        while not send_future.done():
            time.sleep(0.01)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._publish_log(f"Nav2 rejected goal to {location_name}")
            return False

        # Store handle for possible cancel
        with self._nav_goal_lock:
            self._current_goal_handle = goal_handle

        # Create a threading.Event and a place to store result status
        result_event = threading.Event()
        result_container = {"status": None}

        # Callback when result comes (executed inside rclpy executor thread)
        def _on_result(fut):
            try:
                res = fut.result()
                # res has .status (GoalStatus) and .result (the action result)
                status = getattr(res, 'status', None)
                result_container["status"] = status
            except Exception as e:
                self.get_logger().warn(f"Exception in _on_result: {e}")
                result_container["status"] = None
            finally:
                result_event.set()

        # Register callback on get_result_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(_on_result)

        # Wait for either result_event set or nav_timeout or cancel flag
        waited = 0.0
        poll_dt = 0.05
        while waited < nav_timeout:
            if result_event.is_set():
                break
            if self._cancel_flag:
                # Cancel requested: cancel goal
                try:
                    goal_handle.cancel_goal_async()
                    self._publish_log("Goal cancel requested to Nav2")
                except Exception:
                    pass
                # wait a short while for cancel to be processed
                # let the loop continue and result callback handle status
            time.sleep(poll_dt)
            waited += poll_dt

        # If result not arrived yet, attempt to cancel and return False
        if not result_event.is_set():
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._publish_log(f"Navigation to {location_name} timed out/cancelled")
            with self._nav_goal_lock:
                self._current_goal_handle = None
            return False

        # result arrived; interpret status
        status = result_container["status"]
        with self._nav_goal_lock:
            self._current_goal_handle = None

        # GoalStatus: 4 = SUCCEEDED (this numeric matches action_msgs/goal_status constants)
        success = (status == 4)
        self._publish_log(f"Nav2 result for {location_name}: status={status} success={success}")
        return success

    # --------------------------
    # Utility: publish log
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
        rclpy.spin(node)  # main thread handles callbacks & action server responses
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
