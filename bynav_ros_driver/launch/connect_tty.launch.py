
import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


PKG = "bynav_ros_driver"

def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory(PKG), "config", cfg )
    

def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)

def get_params(cfg):
    return load_yaml(get_cfg_path(cfg))

def arg(name, default_value, description):
    return DeclareLaunchArgument(name = name, description = description, default_value = default_value)

def generate_launch_description():

    node = Node(
        package=PKG,
        namespace='bynav',
        name=PKG,
        executable=PKG+'_exe',
        
        parameters=[
                    get_params("std_msg_handlers.yaml"),
                    get_params("std_oem7_raw_msgs.yaml"),
                    get_params("std_msg_topics.yaml"),
                    get_params("supported_imus.yaml"),
                    get_params("std_init_commands.yaml"),
                    get_params("ntrip_parameters.yaml"),
                    get_params("nmea_parameters.yaml"),
                    {
                    'oem7_msg_decoder'   : 'Oem7MessageDecoder',
                    'oem7_max_io_errors' : 3,
                    # Data storage file path. Leave it empty if you don't want to save the data.
                    'oem7_receiver_log'  : '',
                    # The maximum size of the data file. When the file size exceeds this value, a new file will be used for saving. The unit is MB. Set it to 0 to keep saving to the same file.
                    'max_log_size'      : 100,
                    'oem7_if'                       : 'Oem7ReceiverPort',
                    'oem7_port_name'                : LaunchConfiguration('oem7_port_name'),
                    'oem7_port_baud'                : LaunchConfiguration('oem7_port_baud'),
                    'imu_rate'                      : LaunchConfiguration('imu_rate'),
                    'imu_gyro_scale_factor'         : LaunchConfiguration('imu_gyro_scale_factor'),
                    'imu_accel_scale_factor'        : LaunchConfiguration('imu_accel_scale_factor')
                    }
        ],
    
        output='screen'
    )

    imu_rate_arg = arg('imu_rate', '0', 'IMU raw data frequency')
    imu_gyro_scale_factor_arg = arg('imu_gyro_scale_factor', '0.0', 'Scale factor for raw IMU gyroscope data, unit: °/s/LSB')
    imu_accel_scale_factor_arg = arg('imu_accel_scale_factor', '0.0', 'Scale factor for raw IMU accelerometer data, unit: m/s²/LSB')
    
    return LaunchDescription([
                             arg('oem7_port_name', '/dev/ttyUSB0',  'Integrated navigation serial port name, e.g., /dev/ttyUSB0'), 
                             arg('oem7_port_baud', '115200',  'Integrated navigation serial port baud rate, e.g., 115200'), 
                             imu_rate_arg,
                             imu_gyro_scale_factor_arg,
                             imu_accel_scale_factor_arg,
                             node
                             ])
