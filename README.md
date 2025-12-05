**Assignment 1 - ROS2 Turtle Control**

This project implements a multi-node ROS2 system that controls two turtles in **turtlesim**.  
It includes:
- A Python node that **spawns a second turtle**
- A C++ node (`distance`) that **monitors safety rules**:
  - Prevents turtles from colliding (minimum distance)
  - Stops a turtle when it reaches the map boundary
  - Automatically turns the turtle 180° when it hits a border
- A C++ UI node (`ui`) that lets the user **send velocity commands** to either turtle

---

**Project Structure**

```
assignment1_rt/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   ├── ui.cpp              #user interface node
│   └── distance.cpp        #distance node
└── scripts/
    ├── turtle_spawn.py     #spawns turtle2
```


---

**How to Build**

- In one terminal:

```
cd ~/ros2_ws
colcon build --packages-select assignment1_rt
source install/setup.bash
```
- Terminal 1 - Start turtlesim:
```
ros2 run turtlesim turtlesim_node
```
- Terminal 2 - Spawn turtle2:
```
ros2 run assignment1_rt turtle_spawn
```

- Terminal 3 - Start the distance node:
```
ros2 run assignment1_rt distance
```

- Terminal 4 - Start the user control node:
```
ros2 run assignment1_rt ui
```
---

**Node Descriptions**

TURTLE_SPAWN.PY:

- Creates turtle2 using the /spawn service.
- Allows both turtles to be controlled independently.

DISTANCE.CPP:

- Subscribes to poses of turtle1 and turtle2
- Publishes stop commands when:
  - Distance between turtles becomes too small
  - A turtle reaches the world border
  - Automatically rotates a turtle 180° when it hits the border
  - After rotation, the turtle is allowed to move again

Key parameters:
- dist_limit → minimum safe distance between turtles
- min_x, max_x, min_y, max_y → movement boundaries


UI.CPP:

- Terminal interface that lets the user:
  - Choose which turtle to control (turtle1 or turtle2)
  - Enter linear velocity
  - Enter angular velocity
  - Each command is applied for 1 second, then the turtle stops.

---

**Logic Summary**

- Turtles never leave the map - they rotate away when touching borders.
- Turtles never collide - movement stops if distance < threshold.
- Safety node temporarily blocks new commands only during the correction turn.
- After turning, turtles move normally again.

---

**Requirements**
- Ubuntu 22.04 (WSL or native)
- ROS2 Jazzy
- turtlesim package installed

---

**Author: Endri Gjinaj**
