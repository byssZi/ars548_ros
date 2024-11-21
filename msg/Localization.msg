# s
float64 timestamp

# 定位模块最终的输出 为UTM坐标，四元数定位变换坐标格式为Z Y X ,坐标系为UTM坐标
nav_msgs/Odometry location

# 当前定位状态是否有效
bool is_valid

#定位定向设备原始数据
ars548_ros/Ins original_ins
