import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox, QListWidget
)
import rclpy
from std_msgs.msg import String


class CafeDashboard(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # Publishers
        self.order_pub = node.create_publisher(String, 'new_order', 10)
        self.cancel_pub = node.create_publisher(String, 'cancel_order', 10)
        self.confirm_kitchen_pub = node.create_publisher(String, 'confirm_kitchen', 10)
        self.confirm_table_pub = node.create_publisher(String, 'confirm_table', 10)
        self.home_pub = node.create_publisher(String, 'go_home', 10)  # New publisher

        # Logs & robot state
        self.logs = []
        self.robot_state = QLabel("Robot State: IDLE")

        # GUI setup
        self.init_ui()
        self.setWindowTitle("Cafe Robot Dashboard")
        self.show()

        # Subscribe to logs
        node.create_subscription(String, 'order_status', self.log_callback, 10)

    def init_ui(self):
        layout = QVBoxLayout()

        # Table selection
        self.combo = QComboBox()
        self.combo.addItems(["Table 1", "Table 2", "Table 3"])
        layout.addWidget(QLabel("Select Table"))
        layout.addWidget(self.combo)

        # Order buttons
        self.add_btn = QPushButton("Add Order")
        self.add_btn.clicked.connect(self.add_order)
        layout.addWidget(self.add_btn)

        self.cancel_btn = QPushButton("Cancel Order")
        self.cancel_btn.clicked.connect(self.cancel_order)
        layout.addWidget(self.cancel_btn)

        # New: Global Cancel button
        self.global_cancel_btn = QPushButton("🚨 Global Cancel (All Orders)")
        self.global_cancel_btn.clicked.connect(self.global_cancel)
        layout.addWidget(self.global_cancel_btn)

        # New: Go Home button
        self.home_btn = QPushButton("🏠 Go Home")
        self.home_btn.clicked.connect(self.go_home)
        layout.addWidget(self.home_btn)

        # Confirmation buttons
        self.kitchen_btn = QPushButton("Confirm Kitchen")
        self.kitchen_btn.clicked.connect(self.confirm_kitchen)
        layout.addWidget(self.kitchen_btn)

        self.table_btn = QPushButton("Confirm Table")
        self.table_btn.clicked.connect(self.confirm_table)
        layout.addWidget(self.table_btn)

        # Robot state
        layout.addWidget(self.robot_state)

        # Logs
        self.log_list = QListWidget()
        layout.addWidget(QLabel("Logs"))
        layout.addWidget(self.log_list)

        self.setLayout(layout)

    # -----------------------------
    # GUI callbacks
    # -----------------------------
    def add_order(self):
        table = self.combo.currentText()
        msg = String()
        msg.data = table
        self.order_pub.publish(msg)
        self.append_log(f"📦 Order sent for {table}")

    def cancel_order(self):
        table = self.combo.currentText()
        msg = String()
        msg.data = table
        self.cancel_pub.publish(msg)
        self.append_log(f"❌ Cancel sent for {table}")

    def global_cancel(self):
        msg = String()
        msg.data = "ALL"
        self.cancel_pub.publish(msg)
        self.append_log("🚨 Global Cancel issued (all orders cancelled)")

    def go_home(self):
        msg = String()
        msg.data = "HOME"
        self.home_pub.publish(msg)
        self.append_log("🏠 Go Home command sent")

    def confirm_kitchen(self):
        table = self.combo.currentText()
        msg = String()
        msg.data = table
        self.confirm_kitchen_pub.publish(msg)
        self.append_log(f"✅ Kitchen confirmed for {table}")

    def confirm_table(self):
        table = self.combo.currentText()
        msg = String()
        msg.data = table
        self.confirm_table_pub.publish(msg)
        self.append_log(f"✅ Table confirmed for {table}")

    # -----------------------------
    # Log callback
    # -----------------------------
    def log_callback(self, msg):
        text = msg.data
        self.append_log(text)

        # Update robot state
        if "Heading to Kitchen" in text:
            self.robot_state.setText("Robot State: TO_KITCHEN")
        elif "Heading to Table" in text:
            self.robot_state.setText("Robot State: TO_TABLE")
        elif "Reached Home" in text or "Returning Home" in text:
            self.robot_state.setText("Robot State: IDLE")

    # -----------------------------
    # Append log
    # -----------------------------
    def append_log(self, text):
        self.logs.append(text)
        self.log_list.addItem(text)
        self.log_list.scrollToBottom()


# -----------------------------
# Main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gui_node')
    app = QApplication(sys.argv)
    dashboard = CafeDashboard(node)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
