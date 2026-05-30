# Truck Gateway Startup Guide

This document provides the standard operating procedure for connecting to the truck via CAN bus and using an Xbox controller for remote operation via ROS 2.

---

## 1. Connect to Truck (CAN Interface)

### Physical Setup
1. Follow the standard vehicle startup procedure.
2. Configure the truck into **CAN-mode** using the **HAN-Tune** utility.

   2.1 Open the HANtune directory (this case build 102)
   ```bash
      cd HANtune_build_102
   ```
   2.2 Open HANtune from the terminal
   ```bash
      bash HANtune.sh
   ```
   2.3 Open file HAN_Tune_SCALED.hml in HANtune
   2.4 Calibrate wheels
3. Connect the **PCAN-USB** adapter to the device.

### CAN network Configuration


**Verify that the PCAN-USB driver is available:**
```bash
ip link show | grep can
```

**Bring the interface UP:**
```bash
sudo ip link set can1 up type can bitrate 500000
```
**Note change can... to relevant CAN interface*

**(Change CAN interface name)**

By default the launch file of the gateway listens to ```can0```.
This parameter can be changed in the launch file, or the interface can be renamed;
```bash
sudo ip link set can0 down
sudo ip link set can0 name can1
<<<<<<< HEAD
=======
sudo ip link set can0 up type can bitrate 500000
>>>>>>> ad8707203eab5494828eb7775b435d385ab0a344
```

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
- **Device Not Found:** If you receive a `[Errno 19] No such device` error, ensure that a `can` inteface is visible in `ip link show` and that the state is `UP`. Also ensure the launch file is configured to listen to this `can` interface.
- **Permission Denied:** Ensure you have the necessary permissions to access the CAN interface. You may need to add your user to the `dialout` group.

