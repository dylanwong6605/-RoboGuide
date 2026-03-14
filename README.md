# RoboGuide - Autonomous Mobile Robot Simulation

> A Dockerized ROS2 Humble workspace for simulating an Autonomous Mobile Robot (AMR) navigating complex environments with dynamic obstacle avoidance, multi-scenario world support, and automated performance benchmarking.

---

## Summary

Did you know 85% of patients ask for directions at hospitals or public health facilities? How about 30% of first-time visitors get lost? This claim by The Journal of MHealth shows that modern buildings such as hospitals feature irregular, varied floorplans which are hard to navigate. To solve this problem, we introduce ##RoboGuide!## An AMR (Autonomous Mobile Robot) Tour Guide to help navigate and waypoint users to their desired destination. Our solution is to enhance autonomous behaviors of the AMR in simulation and conduct multi-scenario testing to prepare for the physical AMR. Due to time constraints and limited availability, we pivoted from working with the AMR physically, to simulation and testing. In simulation, we tested multiple scenarios including, but not limited to crowded hallways, hallways with obstacles, and varied layouts. During the testing phase, by logging performance metrics such as collisions, time/success rate, and path efficiency, we can analyze the performance of the AMR in simulation to fit our end goal of deployment in the real world.

---

### Core Stack

| Component | Role |
|---|---|
| **ROS2 Humble** | Robot middleware and communication framework |
| **Gazebo** | Physics-based 3D simulation environment |
| **Nav2** | Navigation stack with **A\* path planning** |
| **SLAM Toolbox** | Simultaneous Localization and Mapping |
| **YOLO26** | Real-time object detection |

### Features

- **14 scenario worlds** with varying obstacle layouts and complexity
- **Dynamic obstacle avoidance** — simulated pedestrians and moving objects
- **AMR navigation** through cluttered environments using A\* path planning
- **Automated test runner** — executes scenarios and exports performance metrics to CSV

---

## Documentation

| Document | Description |
|---|---|
| [`INITIAL_SETUP.md`](./INITIAL_SETUP.md) | First-time environment setup and prerequisites |
| [`WORKFLOW.md`](./WORKFLOW.md) | Development workflow |

---

## Getting Started

### 1. Install Dependencies

```bash
sudo apt-get update
sudo apt-get install -y curl wget git
```

### 2. Install Docker

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
rm get-docker.sh
sudo apt-get install -y docker-compose-plugin
```

Verify installation:

```bash
docker --version
docker compose version
```

### 3. Clone the Repository

```bash
git clone https://github.com/dylanwong6605/RoboGuide.git
cd RoboGuide
```

### 4. Allow Docker to Access the Display

```bash
xhost +local:docker
echo "xhost +local:docker" >> ~/.bashrc
```

### 5. Build and Launch the Container

```bash
docker compose build
docker compose up -d
```

### 6. Enter the Container and Build the Workspace
(Make sure to colcon build and source setup after making code changes)

```bash
docker exec -it amr_dev bash
cd /ros2_ws
colcon build
source install/setup.bash
```

---

## Running a Simulation

### Single Scenario Launch

Navigate to the launch file directory:

```
/ros2_ws/src/amr_gazebo/launch/
```

Then launch your chosen scenario:

```bash
ros2 launch amr_gazebo <your_launch_file>.launch.py
# Example:
ros2 launch amr_gazebo example.launch.py
```

### Moving Dynamic Obstacles (Pedestrians)

1. Add or select a person config file from:
   ```
   ros2_ws/src/amr_gazebo/config/
   ```

2. Edit the mover script to reference your config file:
   ```
   ros2_ws/src/amr_gazebo/mover.py
   ```

3. In a **new terminal**, run:
   ```bash
   python3 ros2_ws/src/amr_gazebo/mover.py
   ```

---

## Test Runner

The test runner automatically cycles through scenarios and logs performance metrics (e.g., path length, success rate, navigation time) to a `.csv` file.

```bash
python3 test_runner.py                        # Interactive — prompts for number of runs
python3 test_runner.py --runs 10              # Run all scenarios 10 times each
python3 test_runner.py --runs 50 --headless   # Overnight: 50 runs, no GUI
```

---

## Exiting the Container

```bash
exit
```

---
