# 时间戳

# utc_time
string utc_time

# gps_time
uint16 gps_week
float64 gps_time

# unix ros linux timestamp
float64 timestamp

float64 latitude

float64 longitude

# 海拔高
float64 altitude

#椭球高 m
float64 ellip_height

# 东向速度，单位m/s
float64 east_speed

# 北向速度，单位m/s
float64 north_speed

# 天向速度，单位m/s
float64 sky_speed

# 水平地面速度，单位m/s
float64 ground_speed

# 航向 0 - 2pi 单位rad
float64 yaw

# 俯仰，单位 rad
float64 pitch

# 横滚，单位 rad
float64 roll

# X轴加速度，单位m/s²
float64 acc_x

# Y轴加速度，单位m/s²
float64 acc_y

# Z轴加速度，单位m/s²
float64 acc_z

# X轴角速度，rad/s
float64 angular_x

# Y轴角速度，单位 rad/s
float64 angular_y

#Z轴角速度，顺时针为正，单位rad/s
float64 angular_z

#星数 主天线使用星数
uint8 satellite_num
uint8 satellite_num_1 # 辅助天线星数

# 主/辅天线搜星数
uint8 satellite_num_sats
uint8 satellite_num_sats_1 # 辅助天线星数

#卫星状态 0 不定位不定向 1 单点定位定向 2伪距差分定位定向  3 组合推算 4 RTK固定解 5 RTK浮点解 6 单点定位不定向 7 伪距差分定位不定向 8 RTK稳定解定位不定向 9 RTK浮点解定位不定向 10 pps 11 手动输入模式
uint8 satellite_status

# 系统状态 0 初始化 1 卫星导航 2 组合导航 3 纯惯性 4 里程计组合导航
uint8 system_state

#导航信息不确定度
uint8 nav_uncertainty

uint8 front_star_num
uint8 back_star_num 
uint8 ins_state 
uint8 gps_state

#0x02:定向设备正在启动；
#0x03:启动完毕；
#0x04:静态寻北；
#0x05:导航；
#0x09:程序烧写；
uint8 work_status
