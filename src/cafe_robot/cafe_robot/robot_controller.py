# robot_controller.py
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

named_locations = {
    "Home": [-4.21, -5.64, 0.0],
    "Kitchen": [-11.69, -5.91, 0.0],
    "Table 1": [1.09, -3.11, 0.0],
    "Table 2": [1.19, 2.97, 0.0],
    "Table 3": [-1.55, 2.76, 0.0],
}

class RobotController:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def go_to(self, location_name, callback=None):
        if location_name not in named_locations:
            self.node.get_logger().error(f"Unknown location: {location_name}")
            if callback:
                callback(False)
            return

        x, y, yaw = named_locations[location_name]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # convert yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.send_goal(pose, callback or (lambda s: None))

    def send_goal(self, pose: PoseStamped, callback):
        self.client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(lambda f: self.goal_response(f, callback))

    def goal_response(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected!")
            callback(False)
            return
        self.node.get_logger().info("Goal accepted!")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda f: callback(f.result().status == 4))  # SUCCEEDED
