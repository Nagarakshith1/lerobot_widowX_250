# WidowX 250S with LeRobot

This fork extends [LeRobot](https://github.com/huggingface/lerobot) to support the **WidowX 250S robotic arm** with **SpaceMouse teleoperation**.  

📌 For environment setup instructions, please refer to the original [LeRobot README](https://github.com/huggingface/lerobot/blob/main/README.md).  

🔗 Robot specifications can be found here: [WidowX 250S Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html)

---

## Teleoperation System

You can teleoperate the WidowX 250S robotic arm using either:  
- **Keyboard controls**, or  
- **[3Dconnexion SpaceMouse](https://3dconnexion.com/dk/product/spacemouse-compact/)**  

The robot is controlled via **end-effector position control**.  

The **URDF model** for the WidowX 250S is taken from the [Interbotix ROS packages](https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/wx250s.urdf.xacro).

---

### SpaceMouse Setup

#### Install Dependencies
```bash
sudo apt-get update
sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
mamba install -c conda-forge hidapi
pip install pyspacemouse easyhid
```
#### Set Up UDEV Rules
1. Identify the device using
    ```bash
    lsusb
    ```
    Example output:
    ```
    Bus 001 Device 005: ID 256f:c62e 3Dconnexion SpaceMouse Compact
    ```
2. Replace `idVendor` and `idProduct` values in the command below if they differ from your device:
    ```bash
    sudo sh -c 'echo "KERNEL==\"hidraw*\", ATTRS{idVendor}==\"256f\", ATTRS{idProduct}==\"c635\", MODE=\"0666\"" > /etc/udev/rules.d/99-spacemouse.rules'
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
3. Replug the SpaceMouse for changes to take effect.

### Enable Low Latency Mode for USB (Robot Arm)
1. Identify the FTDI/USB2Dynamixel adapter using:
    ```bash
    lsusb
    ```
    Example output:
    ```
    Bus 001 Device 038: ID 0403:6014 Future Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC
    ```
2. Replace `idVendor` and `idProduct` values in the command below if they differ from your device:
   ```bash
   sudo sh -c 'echo "ACTION==\"add\", SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", RUN+=\"/bin/setserial /dev/%k low_latency\"" > /etc/udev/rules.d/99-dynamixel.rules'
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
3. Replug the robot arm for changes to take effect.

### Running the Teleoperation Script
Example command to run the teleoperation script with SpaceMouse
```bash
python teleoperate.py --robot.type=widowx_250s_end_effector --robot.id 1 --robot.port=/dev/ttyUSB0 --robot.cameras={ top: {type: opencv, index_or_path: /dev/video4, width: 640, height: 480, fps: 30}} --teleop.type=space_mouse --display_data=true 

```
For keyboard teleoperation use `teleop.type=keyboard_ee`.

For smooth space mouse control, some keyboard hotkeys are available:
- 'r': Toggles rotation lock so that only translation is applied
- 't': Toggles translation lock so that only rotation is applied
- 'g': Toggles gripper open/close