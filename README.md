# Butler Robot — ROS2 Café Automation

This project implements a butler robot system for café/restaurant automation using ROS 2.
It includes:

🧠 Order Manager Node → Handles order flow, queue management, robot navigation.

🖥️ GUI Dashboard (PyQt5) → Control panel for operators to place/cancel orders and monitor robot state.

🚙 Robot Integration → Works with Nav2, AMCL, and SLAM for autonomous navigation.

✨ Features

Add & cancel table orders (supports batch & global cancel).

Kitchen & table confirmation workflow.

Return robot to home base on demand.

Real-time order status monitoring.

Operator-friendly PyQt5 dashboard.

Logs of all robot actions for traceability.


🚀 Getting Started
1. Install Dependencies
# ROS 2 (Humble recommended)
sudo apt install ros-humble-desktop

# Python deps
pip install pyqt5

2. Build & Source Workspace
cd ~/ros2_ws
colcon build
source install/setup.bash

3. Run Simulation + Order Manager
ros2 launch butler_pkg butler.launch.py

4. Run GUI Dashboard
ros2 run butler_pkg gui_dashboard.py

🖥️ GUI Dashboard
Controls

Add Order → Send order for selected table.

Cancel Order → Cancel the selected table’s order.

🚨 Global Cancel → Cancel all active orders.

🏠 Go Home → Return robot to home position.

Confirm Kitchen → Confirm pickup from kitchen.

Confirm Table → Confirm delivery to table.

Robot State

IDLE → Robot is waiting at home.

TO_KITCHEN → Robot is heading to the kitchen.

TO_TABLE → Robot is delivering to a table.
