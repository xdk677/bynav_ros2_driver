# 安装依赖
运行脚本文件 install_dependencies.sh
1. 在线安装
```shell
./install_dependencies.sh online
```
2. 离线安装
```shell
./install_dependencies.sh offline
```

# 编译
运行脚本文件
```bash build.sh

# 加载环境
```shell
. install/setup.bash
```

# 运行
支持串口和网口两种通信方式
1. 串口
```shell
ros2 launch bynav_ros_driver connect_port.launch.py
```
2. 网口
```shell
ros2 launch bynav_ros_driver connect_net.launch.py
```

在 launch 文件中可以修改串口和网口连接信息

在config文件夹下的ntrip_parameters.yaml文件中可以启用ntrip和配置ntrip相关参数。

**注意：**
1. 修改完源码中的配置文件后需要重新运行下脚本文件 build.sh
2. 如果需要使用IMU的相关数据，那么使用前需要根据具体的IMU型号在 launch 文件中配置 IMU 频率以及 gyro 和 acc 的 scale
3. 如果脚本文件无法执行，可以通过下面的指令增加可执行权限
```shell
chmod +x build.sh
chmod +x install_dependencies.sh
```
