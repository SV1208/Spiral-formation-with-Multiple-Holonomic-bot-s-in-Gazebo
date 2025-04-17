# Spiral Formation with Multiple Holonomic Bots in Gazebo

This project demonstrates a **multi-robot holonomic control system** in a Gazebo simulation using ROS 2. Four holonomic mobile robots are launched in a circle and are controlled to **spiral inward and then outward** in a coordinated formation. It showcases the loading and control of multiple robots simultaneously, highlighting the ability to manage and command their movements.

**🎥 Demo Video:** Watch the simulation in action: [YouTube - Spiral Formation Demo][![Demo Thumbnail](https://img.youtube.com/vi/sCC9gmWQZFI/0.jpg)](https://www.youtube.com/watch?v=sCC9gmWQZFI)

---

## 💡 Project Highlights

- **Multi-robot Holonomic Motion Control:** Demonstrates coordinated control of four holonomic robots.
- **Spiral Formation:** Robots  spiral inward towards a center point and then expand outward in a circular formation.
- **Simultaneous Loading and Control:** Shows the process of loading multiple robot models in Gazebo and controlling them concurrently.
- **ROS 2 Implementation:** Utilizes ROS 2 for robot control and communication.

---

---

## ⚙️ Requirements

Tested with
- ROS 2 (Humble)
- Gazebo Classic
- Python 3
- `rclpy` (ROS 2 Python client library)
---

## 🚀 How to Run

### 1. Clone the repository:

```bash
git clone https://github.com/SV1208/Spiral-formation-with-Multiple-Holonomic-bot-s-in-Gazebo.git
````
### 2. Do colcon build:
```bash
colcon build
source install/setup.bash
```
### 3. Launch the multibot.launch.py file:
```bash
ros2 launch robot_bringup multibot.launch.py
```
### 4. Run the  spiral_formation node:
```bash
ros2 run formations spiral_formation
```

## 🙏 Credits
The base code and model for the holonomic drive system used in this project are taken from the repository:
https://github.com/MickySukmana/holonomic
All credit for the design and implementation of the holonomic drive system goes to Micky Sukmana. 

My part was to launch multiple bots and make the spiral node
