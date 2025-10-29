# Human-Following Robot: Environment Setup (ROS Melodic + TurtleBot2)

> This project demonstrates a human-following robot using computer vision and deep learning (YOLOv4) to perform vision-only navigation with a TurtleBot2 running on ROS Melodic. The environment setup is simplified since most of the base installation and hardware configuration are already clearly explained in the repository [Turtlebot2-On-Melodic](https://github.com/gaunthan/Turtlebot2-On-Melodic). This repo greatly explains how to set up ROS Melodic and your TurtleBot2 on Ubuntu 18.04.

---

## 1) Requirements

**Hardware:**

* TurtleBot2 with Kobuki base
* Laptop running Ubuntu 18.04 (desktop version)
* USB camera or integrated laptop camera

**Software:**

* ROS Melodic
* Python 3.6+
* OpenCV (with contrib modules)
* YOLOv4 model files (`.cfg`, `.weights`, `coco.names`)

---

## 2) Set Up ROS Melodic and TurtleBot2

Follow the detailed steps provided in the [Turtlebot2-On-Melodic](https://github.com/gaunthan/Turtlebot2-On-Melodic) repository to install and configure:

* ROS Melodic desktop-full
* Kobuki drivers and TurtleBot2 packages
* Proper udev rules for USB connection
* Teleoperation and navigation packages

Once those steps are complete, verify that you can run:

```bash
roslaunch kobuki_node minimal.launch
```

And confirm that velocity commands are accepted:

```bash
rostopic list
```

Expected topics should include `/mobile_base/commands/velocity` or `/cmd_vel`.

---

## 3) Python 3 Vision Environment

Create and activate a Python 3 virtual environment for the vision-based tracking node:

```bash
sudo apt-get -y install python3-pip python3-venv
python3 -m venv ~/vision_env
source ~/vision_env/bin/activate
pip install numpy opencv-python opencv-contrib-python roslibpy
```

---

## 4) rosbridge Communication

Start the rosbridge server to connect the Python 3 vision script with ROS Melodic:

```bash
sudo apt-get -y install ros-melodic-rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch
```

This opens a WebSocket interface at port 9090 for message exchange.

---

## 5) Project Structure

```
human_follower_robot/
├── models/
│   ├── yolov4.cfg
│   ├── yolov4.weights
│   └── coco.names
├── Camera.py
└── README.md
```

Ensure the `models/` folder matches the paths used in your code.

---

## 6) Running the Project

1. **Launch Kobuki base:**

```bash
roslaunch kobuki_node minimal.launch
```

2. **Run rosbridge:**

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

3. **Start the vision-based follower:**

```bash
source ~/vision_env/bin/activate
python3 follower.py
```

A camera window will open, and the robot will begin tracking and following detected persons.

---

## 7) Communication Topics

* Velocity commands are published to `/mobile_base/commands/velocity`.
* Message type: `geometry_msgs/Twist`
* The node controls the robot’s linear and angular velocity according to detected target position and estimated distance.

---

## 8) YOLOv4 Model Files

Place the following files inside the `models/` directory:

* `yolov4.cfg`
* `yolov4.weights`
* `coco.names`

These are required for the object detection stage.

---

## 9) Camera Test

Verify camera functionality before running:

```bash
v4l2-ctl --list-devices
ls /dev/video*
```

Ensure it appears as `/dev/video0` for `cv2.VideoCapture(0)`.

---

## 10) Notes

* Power on the Kobuki base before launching ROS nodes.
* Confirm ROS network settings if running multiple computers.
* Manually test velocity topics before starting the full follower script.

---

— End —
