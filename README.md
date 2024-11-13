# Project Name: F1tenth Autonomous Systems Labs

## Overview
This repository contains the lab assignments for the F1tenth Autonomous Systems course. Each lab is organized into its own folder, and you can build and run the code for each lab using the `colcon` build system.

---

## Prerequisites
Ensure you have the following installed on your system before building the project:
- **ROS 2 (Humble, Foxy, etc.)**
- **Colcon Build Tools**
- Python dependencies (if any, specify them here)

---

## Build Instructions

1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/UpLiftL1f3/F1tenth.git
   cd F1tenth
   ```

2. Navigate to the specific lab folder you want to build:
   ```bash
   cd lab1_ws  # Replace 'lab1_ws' with the desired lab folder
   ```

3. Run the `colcon` build command:
   ```bash
   colcon build --symlink-install
   ```

   - The `--symlink-install` flag creates symbolic links, allowing changes to your source code to reflect without rebuilding.

---

## Running the Code
Once the build process completes successfully, you can run the nodes or launch files for the specific lab. For example:

1. Source the workspace:
   ```bash
   source install/setup.bash
   ```

2. Launch or execute the desired node:
   ```bash
   ros2 launch <package_name> <launch_file>.launch.py
   ```

---

## Troubleshooting
- **Build Fails**: Ensure all dependencies are installed. You can install dependencies using `rosdep`:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
- **Environment Not Sourced**: Make sure you've sourced your ROS 2 installation and the `install/setup.bash` file.

---

## Directory Structure
```plaintext
.
├── lab1_ws/
│   ├── src/
│   ├── build/
│   ├── install/
│   ├── log/
├── lab2_ws/
│   ├── src/
│   ├── build/
│   ├── install/
│   ├── log/
└── README.md
```
- Each lab folder (`lab1_ws`, `lab2_ws`, etc.) contains its own workspace and follows the ROS 2 workspace structure.

---

## License
This project is licensed under the [MIT License](LICENSE).

---

## Acknowledgments
- F1tenth Community
- [ROS 2 Documentation](https://docs.ros.org/)