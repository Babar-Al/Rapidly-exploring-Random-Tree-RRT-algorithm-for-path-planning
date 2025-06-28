# Rapidly exploring Random Tree(RRT)algorithm for path planning
This project implements the Rapidly-exploring Random Tree (RRT) algorithm for path planning in a 2D space with obstacles. Given a grayscale map (e.g., an aerial or occupancy grid), the algorithm computes a path from a start point to a goal point, avoiding obstacles.
#  RRT Path Planning in 2D Obstacle Map

This project implements the **RRT (Rapidly-exploring Random Tree)** algorithm to find a path between a start and a goal point while avoiding obstacles in a 2D environment. The input map is a grayscale image where obstacles are shown in black and free space in white.


---

## 📌 Features

- 📸 Loads a map image and converts it into an occupancy grid
- 🧠 Implements a basic RRT algorithm
- 🔄 Avoids obstacles during tree expansion
- 🗺️ Visualizes the growing tree and final path using `matplotlib`
- 📍 Supports dynamic visualization of iterations

---

## 🛠️ How It Works

1. The grayscale map image is loaded and converted to a binary occupancy grid.
2. A tree is initialized at the start location.
3. Random points are sampled.
4. The tree grows toward the sampled point if the path is obstacle-free.
5. The process repeats until the tree reaches the goal region.
6. The final path is traced back from the goal to the start.

---




