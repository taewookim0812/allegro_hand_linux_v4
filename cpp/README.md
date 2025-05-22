# C++ API(Grasp Demo)

This guide provides step-by-step instructions to run the motion demo for the Allegro Hand V4 using **C++ API**. Note that this API does not need ROS1 or ROS2.

> [!IMPORTANT]
> Make sure to choose the correct Ubuntu version for your setup. The **Setup Manual** and **Source Code** differ depending on the project you are working on.
> And Choose the correct version of the `grasp` program for your system:
> - **Ubuntu 20.04**: `grasp_2004`
> - **Ubuntu 22.04**: `grasp_2204`

## File Structure


```
.
└── cpp
    ├── grasp_[ubuntu_version]
    │   ├── CMakeLists.txt
    │   ├── RockScissorsPaper.cpp
    │   ├── RockScissorsPaper.h
    │   ├── canAPI.cpp
    │   ├── canAPI.h
    │   ├── canDef.h
    │   ├── main.cpp
    │   ├── rDeviceAllegroHandCANDef.h
    │   ├── build.sh
    │   └── run.sh
    └── build_and_run.sh
```

## Build the `grasp` Program

We use an **out-of-source** build style to keep source files clean.

1. **Configure the Allegro Hand V4**

- Before building the grasping program, ensure that the hand configuration in [`main.cpp`](./grasp/main.cpp) matches your hardware settings.:
  ```cpp
  // USER HAND CONFIGURATION
  const bool RIGHT_HAND = true;
  const int HAND_VERSION = 4;
  const double tau_cov_const_v4 = 1200.0; // 1200.0 for SAH040xxxxx
  ```

2. **Build with CMake**

  - Run the build script. The `build.sh` file is in the `grasp` directory: `./build.sh`
  - The `build/` directory contains all compiled files. If needed, you can delete it and rebuild without affecting the source code.

3. **Run the Motion Demo**

   - Power on the Allegro Hand V4 and **Connect PCAN-USB

   - Start the Grasping Program. The `run.sh` file is in the `grasp` directory: `./run.sh`

   - Use Keyboard Commands to Control the Hand

     | Command | Action |
     |---------|--------|
     | `H` | Home Position (PD control) |
     | `R` | Ready Position (used before grasping) |
     | `G` | Three-Finger Grasp |
     | `K` | Four-Finger Grasp |
     | `P` | Two-finger pinch (index-thumb) |
     | `M` | Two-finger pinch (middle-thumb) |
     | `E` | Envelop Grasp (all fingers) |
     | `A` | Gravity Compensation |
     | `F` | Turn Servos OFF (any grasp command turns them back on) |
     | `1` | Rock Motion (from `RockScissorsPaper.cpp`) |
     | `2` | Scissors Motion (from `RockScissorsPaper.cpp`) |
     | `3` | Paper Motion (from `RockScissorsPaper.cpp`) |
     | `S` | Show Keyboard Commands List |
     | `Q` | Quit program |

4. Tips: Build and Run

   - To build and run the program in one step, use the `build_and_run.sh` script in the `cpp` directory: `./build_and_run.sh [folder_name]`
     - Replace `[folder_name]` with the name of the folder containing the `CMakeLists.txt` file (e.g., `grasp_2004` or `grasp_2204`).
   - This script will build the program and then run it automatically.

---

## Notes

- If the hand does not move, double-check the **PCAN interface** and **Configuration of the Allegro Hand V4**.
- If you need custom grasping motions, modify `RockScissorsPaper.cpp`.

### 🎯 You're all set! Now you can experiment with different grasping techniques and develop new motion strategies! 🚀


## Reference
- [Allegro Hand V4 CAN Protocol](https://www.allegrohand.com/v4-main/can-protocol-1)


