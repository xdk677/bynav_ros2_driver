
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
                    'oem7_max_io_errors' : 10,
                    # 数据保存文件路径, 为空时不保存
                    'oem7_receiver_log'  : '',
                    # 数据文件的最大大小， 超过该值时切换新文件保存, 单位为 MB, 设置为 0 则一直存放到一个文件中
                    'max_log_size'      : 100,
                    'oem7_if'                       : LaunchConfiguration('oem7_if'),
                    'oem7_ip_addr'                  : LaunchConfiguration('oem7_ip_addr'),
                    'oem7_port'                     : LaunchConfiguration('oem7_port'),
                    'imu_rate'                      : LaunchConfiguration('imu_rate'),
                    'imu_gyro_scale_factor'         : LaunchConfiguration('imu_gyro_scale_factor'),
                    'imu_accel_scale_factor'        : LaunchConfiguration('imu_accel_scale_factor')
                    }
                    ],
    
        output='screen',
    )
    
    ip_arg   = arg('oem7_ip_addr', '127.0.0.1',  '组合导航设备的 ipv4 地址, 例如, 127.0.0.1')
    port_arg = arg('oem7_port', '8888', '组合导航设备的端口, 比如 8888')
    if_arg   = arg('oem7_if', 'Oem7ReceiverTcp', '选择连接方式, tcp 连接方式填写 Oem7ReceiverTcp, \
                                                 udp 连接方式填写 Oem7ReceiverUdp, 默认是 tcp 连接方式')
    imu_rate_arg = arg('imu_rate', '0', 'imu 原始数据频率')
    imu_gyro_scale_factor_arg = arg('imu_gyro_scale_factor', '0.0', 'imu 陀螺仪原始数据比例因子, 单位为 °/s/LSB')
    imu_accel_scale_factor_arg = arg('imu_accel_scale_factor', '0.0', 'imu 加速度计原始数据比例因子, 单位为 m/s2/LSB')
    
    return LaunchDescription([ip_arg,
                              port_arg,
                              if_arg,
                              imu_rate_arg,
                              imu_gyro_scale_factor_arg,
                              imu_accel_scale_factor_arg,
                              node])