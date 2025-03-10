#Install dependencies
Run the script file install_dependencies.sh
1. Online installation
```shell
./install_dependencies.sh online
```
2. Offline installation
```shell
./install_dependencies.sh offline
```

# Compile
Run the script file bash build.sh


# Load the environment

```shell
. install/setup.bash
```

# Run
It supports two communication methods: serial port and network port.
1. Serial port

```shell
ros2 launch bynav_ros_driver connect_port.launch.py
```
2. Network port

```shell
ros2 launch bynav_ros_driver connect_net.launch.py
```

You can modify the serial port and network port connection information in the launch file.

You can enable NTRIP and configure NTRIP-related parameters in the ntrip_parameters.yaml file under the config folder.

**Note:**
1. After modifying the configuration files in the source code, you need to re-run the script file build.sh.
2. If you need to use the relevant data of the IMU, you need to configure the IMU frequency and the scales of gyro and acc in the launch file according to the specific IMU model before use.
3. If the script file cannot be executed, you can add executable permissions through the following command:

```shell
chmod +x build.sh
chmod +x install_dependencies.sh
```
