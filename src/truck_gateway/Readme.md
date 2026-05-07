# Truck Gateway Startup Guide

This document provides the standard operating procedure for connecting to the truck via CAN bus and using an Xbox controller for remote operation via ROS 2.

---

## 1. Connect to Truck (CAN Interface)

### Physical Setup
1. Follow the standard vehicle startup procedure.
2. Configure the truck into **CAN-mode** using the **HAN-Tune** utility.
3. Connect the **PCAN-USB** adapter to the device.

### Network Configuration
The software is configured to look for the interface named `can1`. If your adapter is detected as `can0`, follow these steps to rename and initialize it:

**Verify the current interface name:**
```bash
ip link show | grep can
```

**Rename and bring the interface UP:**
*If the output shows `can0`, run the following sequence:*
```bash
sudo ip link set can0 down
sudo ip link set can0 name can1
sudo ip link set can1 up type can bitrate 500000
```

---

## 2. Connect Xbox Controller

1. **Bluetooth Pairing:** Connect the Xbox controller via the system Bluetooth settings.
2. **Verification:** Once connected, the `truck_gateway` software will automatically map inputs. You can verify input flow later by checking the `/joy` topic:
   ```bash
   ros2 topic echo /joy
   ```

---

## 3. Startup truck_gateway

### Initialize Workspace
Open a terminal and navigate to your ROS 2 workspace:
```bash
cd ~/ros2_ws
```

Build the specific package and source the environment:
```bash
colcon build --packages-select truck_gateway
source install/setup.bash
```

### Launch the Gateway
Run the launch file to start the `joy_node`, `xbox_translator`, and `truck_can_gateway`:
```bash
ros2 launch truck_gateway truck_gateway_launch.py
```

---

## Troubleshooting
- **Device Not Found:** If you receive a `[Errno 19] No such device` error, ensure that `can1` is visible in `ip link show` and that the state is `UP`.
- **Controller Latency:** If the joystick response is sluggish, check the Bluetooth signal strength or consider using a wired USB connection.
- **Permission Denied:** Ensure you have the necessary permissions to access the CAN interface. You may need to add your user to the `dialout` group.
```

### Key Improvements made:
1.  **Clear Hierarchy:** Used H1, H2, and H3 headers for better navigation.
2.  **Standardized Syntax:** Formatted code blocks for better readability.
3.  **Instructional Clarity:** Explained *why* the user is running specific commands (e.g., explaining the `can0` to `can1` rename logic).
4.  **Troubleshooting Section:** Added a section for the most common errors you encountered today.