# allegro_hand_linux_v4
> Official Code Repository for controlling the Allegro Hand V4 on Linux System

This repository is the official release for controlling the **Allegro Hand V4**.

There are [ROS1 API](./ros_source/) and [C++ API](./cpp/) for [Allegro Hand V4](https://www.allegrohand.com/ah-v4-main)

# Setup

In this section, we will introduce how to setup PC setting and PCAN driver which is for connecting Allegro Hand V4 and PC.

1. **Install Ubuntu 20.04 and ROS1 Noetic**
   Follow the official [ROS Noetic installation guide](https://wiki.ros.org/noetic/Installation/Ubuntu).

2. **Install the Grasping Library (`libBHand`)**

- **Download `libBHand`**
  - Get the `libBHand` from the [Grasping Library for Linux](https://www.allegrohand.com/ah-v4-grasping-library-for-linux)
  - Download either `LibBHand_64.zip` or `LibBHand_32.zip` (depending on your system architecture).
    - To check your system architecture: `getconf LONG_BIT`

- **Extract and Install `libBHand`**
  ```bash
  unzip LibBHand_{32|64}.zip
  cd libBHand_{32|64}
  sudo make install
  sudo ldconfig
  ```

3. **Install the PCAN driver**
  > Before using the hand, you must install the **PCAN drivers**. These instructions assume you are using a **Peak Systems PCAN-USB adapter**.

  - **Install dependencies**

      ```bash
      sudo apt-get install libpopt-dev ros-noetic-libpcan
      ```

  - **Download the latest drivers**: [Peak-System Linux Driver](http://www.peak-system.com/fileadmin/media/linux/index.htm#download)

  - **Install the drivers**

      ```bash
      make clean; make NET=NO_NETDEV_SUPPORT
      sudo make install
      sudo /sbin/modprobe pcan
      ```

  - **Verify installation**: Run the following command to check if the interface is installed correctly. You should see streaming output.

      ```bash
      cat /proc/pcan
      ```

  - **Check for available interfaces**: When the hand is connected, you should see `pcanusb0` or `pcanusbN` (`N` can be any number).

      ```bash
      ls -l /dev/pcan*
      ```

     - If no files are listed, try running the following command from the **downloaded PCAN driver folder**. It manually creates the device files if the system has not done so automatically.
       ```bash
       sudo ./driver/pcan_make_devices 2
       ```

  - **(Optional) CAN Communication Tutorial**
     - If you are not familiar with PCAN, please refer to the [CAN Communication Tutorial](https://www.allegrohand.com/ah-v4-can-communication).


# File Structure

```
.
├── cpp
│   ├── 📃README.md: Instructions for building the C++ library.
│   └── grasp
├── ros_source
│   ├── 📃README.md: Instructions for building the ROS package.
│   └── src
│       ├── allegro_hand_controllers
│       │   ├── launch
│       │   │   ├── allegro_hand.launch
│       │   │   └── allegro_viz.launch
│       │   ├── package.xml
│       │   └── src
│       ├── allegro_hand_description
│       ├── allegro_hand_driver
│       ├── allegro_hand_keyboard
│       ├── allegro_hand_parameters
│       ├── allegro_hand_python
│       └── bhand
└── 📃README.md: Instructions for setup the Allegro Hand V4
```

# ROS1 API

We offer a ROS1 API to control the Allegro Hand V4 using **ROS1 Noetic** on **Ubuntu 20.04**. For more details about the ROS1 API, please refer to 📃[ros_source/README.md](./ros_source/README.md).


# C++ API

We’ve provided step-by-step instructions for running the motion demo for the Allegro Hand V4 using the **C++ API**—which operates independently of ROS. For additional details about the C++ API, please see 📃[cpp/README.md](./cpp/README.md).


# Discussion & Issues

If you encounter any issues or would like to discuss improvements, you can use the following channels:

- [**GitHub Issues**](https://github.com/Wonikrobotics-git/allegro_hand_linux_v4/issues)
- [**Allegro Hand Forum**](https://allegrohand.com/forum)


# Contribution

We welcome contributions! 🚀
If you'd like to contribute, feel free to submit a **pull request** with a **clear and concise explanation** of its purpose.

### **Special Thanks to Contributors**

We sincerely appreciate the contributions that have helped improve this repository:

- **Interfaces and controllers** by [@felixduvallet](https://github.com/felixduvallet)
- **New torque controller** by [@nisommer](https://github.com/nisommer)

Thank you for your valuable contributions! 🙌
