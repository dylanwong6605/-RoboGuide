# Development Workflow

## 1. Start Container
```bash
cd -RoboGuide
docker compose up -d
```

---

## 2. Write Code

Write your code in any editor.

**Your code goes in:** `-RoboGuide/ros2_ws/src/`

---

## 3. Build Code

**Enter container:**
```bash
docker exec -it amr_dev bash
```

**Build:**
```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

---

## 4. Run/Test Code

**Inside container:**
```bash
ros2 run <package_name> <your_node_name>
```

Example: `ros2 run amr_perception my_detector`

---

## 5. Push to GitHub

**Exit container:**
```bash
exit
```

**Push:**
```bash
cd -RoboGuide
git add .
git commit -m "Your message"
git push origin main
```

---

## Stop Container
```bash
docker compose down
```
