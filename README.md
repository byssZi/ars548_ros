***
# 说明
这是一个功能完善，能够读取ARS548毫米波雷达UDP数据，并配置雷达状态，发送自车信息给雷达的ros驱动程序，适配于V5.48.4固件版本。部分源码参考自 https://github.com/robotics-upo/ars548_ros 工程
***
# Radar准备
* 通过网线与车载以太网转换器与radar连接
* 终端进入`cfg`路径下，运行下述指令配置VLAN，parent interface可以通过终端运行`ifconfig`查看
```bash
sh configure.sh
```
* 配置完成后可以尝试ping 10.13.1.113观察能否连通
***
# 驱动运行
clone这个工程到ros工作空间下,执行下述指令编译运行ros驱动
```bash
catkin_make
source devel/setup.bash
roslaunch ars548_ros run.launch
```
***
# Radar输出信息
Radar驱动的输出话题如下：
|话题名称|功能描述|备注|
|---|---|---|
|/Status|雷达自身状态信息| - |
|/DetectionList|雷达Detection探测信息| - |
|/ObjectList|雷达Object探测信息| - |
|/DirectionVelocity|雷达Object目标速度方向可视化信息| - |
|/PointCloudDetection|雷达Detection点云可视化信息| - |
|/PointCloudObject|雷达Object目标可视化信息| - |
***
# Radar输入信息
Radar驱动的输入话题如下：
|话题名称|功能描述|备注|
|---|---|---|
|/ars548_process/acc_lateral_cog|本车横向加速度信息| - |
|/ars548_process/acc_longitudinal_cog|本车纵向加速度信息| - |
|/ars548_process/characteristic_speed|本车特征速度信息| - |
|/ars548_process/driving_direction|本车车辆行驶方向信息| - |
|/ars548_process/steering_angle|本车方向盘转角信息| - |
|/ars548_process/velocity_vehicle|本车速度信息| - |
|/ars548_process/yaw_rate|本车偏航率信息| - |
***
# 配置Radar状态
配置Radar状态，通过rosservice实现，另起终端，输入下述指令观察服务名称：
```bash
source devel/setup.bash
rosservice list
```
|服务名称|功能描述|备注|
|---|---|---|
|/set_sensor_mounting|设置radar安装参数|包括Longitudinal、Lateral、Vertical、Yaw、Pitch、PlugOrientation、NewSensorMounting|
|/set_vehicle_parameters|设置车辆参数|包括Length、Width、Height、Wheelbase、NewVehicleParameters|
|/set_radar_parameters|设置radar参数|包括MaximumDistance、FrequencySlot、CycleTime、TimeSlot、HCC、Powersave_Standstill、NewRadarParameters|
|/set_network_configuration|设置radar网络ip|包括SensorIPAddress_0、SensorIPAddress_1（前两个参数配置在`cfg/params.yaml`下）、NewNetworkConfiguration|
***
举例说明配置参数指令如下：（source devel/setup.bash后输入rosservice call `服务名称`后按Tab可以补全参数）
```bash
source devel/setup.bash
rosservice call /set_vehicle_parameters "{Length: 4.4, Width: 2.0, Height: 2.0, Wheelbase: 2.7, NewVehicleParameters: 1}"
```
***
