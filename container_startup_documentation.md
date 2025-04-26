# Yahboom ROS 2 Robot Car Container Startup Documentation

This document explains how the yahboomtechnology/ros-humble:3.6 Docker container is configured, started, and interfaces with the Raspberry Pi 5 host system for the MicroROS-Pi5 ROS2 Robot Car.

## 1. Container Configuration

### 1.1 Docker Image Structure

The Docker image `yahboomtechnology/ros-humble:3.6` is structured as follows:

- **Base OS**: Ubuntu 22.04 (arm64 architecture)
- **ROS 2 Distribution**: ROS 2 Humble (full desktop installation)
- **Additional Components**:
  - ROS 2 camera packages
  - Image transport plugins
  - Yahboom robot-specific packages 

### 1.2 Container Entry Point

The container has the following default configuration:

```json
{
  "Cmd": ["/bin/bash"],
  "Entrypoint": null,
  "WorkingDir": "",
  "Env": [
    "DISPLAY=:0",
    "QT_X11_NO_MITSHM=1",
    "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  ]
}
```

Key points:
- **Default Command**: `/bin/bash` (launches an interactive bash shell)
- **No Custom Entrypoint**: Uses the default Docker behavior
- **Display Environment Variables**: Set up for GUI applications

### 1.3 Workspace Structure

The container includes a pre-built ROS 2 workspace:

```
/root/yahboomcar_ws/
├── build/        # Build artifacts 
├── install/      # Installed packages
│   └── setup.bash  # Workspace setup script
├── log/          # ROS logs
└── src/          # Source code
    ├── bringup/  # Launch files
    ├── yahboom_bringup/  # Yahboom-specific launch files
    ├── yahboom_base/  # Base controller
    ├── yahboom_msgs/  # Custom message definitions
    ├── (various other packages)
```

This workspace contains all the custom packages developed by Yahboom for their robot car platform.

## 2. Container Startup Process

### 2.1 ROS Environment Setup

When the container starts, the following environment is automatically set up through `.bashrc`:

```bash
# From /root/.bashrc
export ROS_DOMAIN_ID=12  # Default domain ID for ROS communication
export LANG=en_US.UTF-8
source /opt/ros/humble/setup.bash  # Source the ROS 2 installation
source ~/yahboomcar_ws/install/setup.bash  # Source the Yahboom workspace
```

This means that:
1. ROS 2 is configured with domain ID 12 by default
2. Both the standard ROS 2 packages and custom Yahboom packages are available

### 2.2 Launch File Structure

The container includes several launch files for different robot functionalities:

1. **Generic Sensor Bringup**:
   - `~/yahboomcar_ws/src/bringup/launch/bringup.launch.py`
   - Starts IMU and LiDAR sensors

2. **Color Detection Demo**:
   - `~/yahboomcar_ws/src/yahboom_bringup/launch/DogFindColor.launch.py`
   - Starts voice control, image publishing, and Redis status services

3. **Mapping and Navigation**:
   - Various launch files for SLAM and navigation functionalities

### 2.3 Startup Command for the Container

The typical command to run this container on the Raspberry Pi would be:

```bash
docker run -it \
  --network=host \
  --privileged \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  yahboomtechnology/ros-humble:3.6
```

Or when starting a specific application:

```bash
docker run -it \
  --network=host \
  --privileged \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  yahboomtechnology/ros-humble:3.6 \
  bash -c "source /opt/ros/humble/setup.bash && \
           source ~/yahboomcar_ws/install/setup.bash && \
           ros2 launch yahboom_bringup DogFindColor.launch.py"
```

## 3. Host-Container Integration

### 3.1 Shared Resources

The Docker container and Raspberry Pi host share several resources:

1. **Device Access** (`-v /dev:/dev`):
   - USB devices (cameras, LiDAR, IMU)
   - Serial ports for microcontroller communication
   - GPIO access (through device files)

2. **Display** (`-v /tmp/.X11-unix:/tmp/.X11-unix` and `-e DISPLAY=$DISPLAY`):
   - X11 socket for GUI applications
   - Allows visualization tools like RViz to display on the host

3. **Network** (`--network=host`):
   - Container shares the host's network stack
   - Enables proper DDS discovery and communication
   - Access to WiFi/Ethernet for remote operations

4. **Full Privileges** (`--privileged`):
   - Grants the container broad hardware access
   - Needed for direct hardware interaction

### 3.2 Communication with micro-ROS

While direct micro-ROS components were not found in the container, the typical communication path would be:

1. **Serial Communication**:
   - Microcontroller connected via USB/Serial to Raspberry Pi
   - Device available as e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`
   - Mounted into container via `/dev` volume

2. **micro-ROS Agent**:
   - Would need to be installed separately or run from a different container
   - Likely started with a command like:
     ```bash
     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
     ```

3. **Topic Bridging**:
   - micro-ROS agent translates between the microcontroller and ROS 2
   - Topics from the microcontroller appear in the ROS domain
   - Container nodes can publish/subscribe to these topics

## 4. Customizing Container Startup

### 4.1 Environment Variables

Key environment variables that can be set when starting the container:

| Variable | Default | Purpose |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | 12 | ROS 2 domain for communication isolation |
| `DISPLAY` | :0 | X11 display for GUI applications |
| `ROS_LOCALHOST_ONLY` | (not set) | Restrict ROS to localhost if set |
| `CYCLONEDDS_URI` | (not set) | Custom DDS configuration |

### 4.2 Custom Entry Points

To create a custom startup script that runs on the container's launch:

1. Create a shell script on the Raspberry Pi:
   ```bash
   #!/bin/bash
   # startup.sh
   source /opt/ros/humble/setup.bash
   source ~/yahboomcar_ws/install/setup.bash
   
   # Start micro-ROS agent
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &
   
   # Launch robot applications
   ros2 launch yahboom_bringup DogFindColor.launch.py
   ```

2. Mount and use the script when starting the container:
   ```bash
   docker run -it \
     --network=host \
     --privileged \
     -v /dev:/dev \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v /path/to/startup.sh:/startup.sh \
     -e DISPLAY=$DISPLAY \
     yahboomtechnology/ros-humble:3.6 \
     bash /startup.sh
   ```

### 4.3 Docker Compose Configuration

For reproducible deployment, a `docker-compose.yml` file could be used:

```yaml
version: '3'
services:
  ros2_robot:
    image: yahboomtechnology/ros-humble:3.6
    network_mode: host
    privileged: true
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=12
    volumes:
      - /dev:/dev
      - /tmp/.X11-unix:/tmp/.X11-unix
    command: bash -c "source /opt/ros/humble/setup.bash && source ~/yahboomcar_ws/install/setup.bash && ros2 launch yahboom_bringup DogFindColor.launch.py"
```

## 5. Startup Best Practices

### 5.1 System Initialization Sequence

For optimal robot startup, follow this sequence:

1. **Hardware Check**:
   - Ensure all USB devices are connected
   - Verify power to sensors and actuators

2. **Start Container with Base Services**:
   - Launch container with sensor drivers
   - Start micro-ROS agent if using microcontrollers

3. **Verify Communication**:
   - Check that topics are published/received
   - Ensure transform tree is properly set up

4. **Launch Application-Specific Nodes**:
   - Navigation, mapping, or color detection as needed

### 5.2 Debugging Startup Issues

Common startup issues and solutions:

1. **Device Permission Problems**:
   - Error: "Permission denied" for /dev/ devices
   - Solution: Ensure the container runs with `--privileged` or adjust udev rules

2. **Missing Serial Connections**:
   - Error: "No such file or directory" for /dev/ttyXXX
   - Solution: Check USB connections or adjust device names

3. **Display Issues**:
   - Error: "Cannot connect to X server"
   - Solution: Verify X11 settings and that DISPLAY variable is properly set

4. **ROS Communication Problems**:
   - Symptom: Nodes don't see each other's topics
   - Solution: Check ROS_DOMAIN_ID consistency and network configuration