Header header

# 驾驶模式反馈 0:人工, 1:无人
uint8 driving_mode

# 无人模式下的具体细分:  0 无人 1远程遥控 2反向人工驾驶
uint8 mode_flag

uint16 adas_mode # bit 0 acc bit 1 aeb bit 2 lka bit3 assist

uint8 acc_active
uint8 aeb_active
uint8 lka_active
uint8 assist_active

#   0:人工, 1:无人,
uint8 adu_mode
uint8 eps_mode 
uint8 epb_mode
uint8 bcm_mode
# allow external cmd
uint8 xbr_mode 
#添加挡位线控，20230808
uint8 gear_mode 

uint8 xbr_active_mode # allow external cmd
uint8 xbr_system_state # allow external cm
uint8 auto_to_manual_tips

# 转向模式反馈  0:前轮转向, 1:楔形转向, 2:原地转向, 3:无效
uint8 steer_mode

float64 steering_wheel_angle #方向盘转角 左正右负
float64 steering_wheel_angle_speed #方向盘角速度 
float64 steering_wheel_hand_moment_signal

# 当前前轮转角 单位: °
float64 front_wheel_angle
# 当前后轮转角 单位: °
float64 rear_wheel_angle
# 当前油门值 0-100% 
float64 throttle_pedal
float64 throttle_pedal_manual
bool is_throttle_pedal_manual


# 当前刹车值 0-100% 
float64 brake_pedal
float64 brake_pedal_manual
bool is_brake_pedal_manual


# 当前档位  空挡：0 前进档：1-6 倒档：7 无效：8  if has gear_p  2 
uint8 gear_location

float64 gear_ratio

# 驻车状态  true:驻车, false:解驻车
# 0 Parking brake not applied
# 1 1 Parking brake applied
# 2 Parking brake releasing
# 3 Parking brake applying
# 4 error
# 5 Secondary braking active

uint8 parking_brake
uint8 epb_state
uint8 parking_brake_switch


# 急停状态  true:急停, false:急停关闭
bool emergency_stop

# 低压上电信号反馈 0:断电, 1:加电, 2:无效
uint8 low_voltage_signal
# 高压上电信号反馈 0:断电, 1:加电, 2:无效
uint8 high_voltage_signal

# 当前车速 单位：m/s
float64 current_velocity
# 当前加速度 单位：m/s^2
float64 current_acceleration
float64 current_torque # 行车实际扭矩 Nm

# 左前轮轮速 单位：m/s 
float64 wheel_speed_fl
# 右前轮轮速 单位：m/s 
float64 wheel_speed_fr
# 左后轮轮速 单位：m/s 
float64 wheel_speed_rl
# 右后轮轮速 单位：m/s 
float64 wheel_speed_rr

# 发动机当前转速 单位：rpm
float64 current_engine_speed
# 发动机当前扭矩 单位：N*m
float64 current_engine_torque 
# 发送机温度 
float64 current_engine_temperature

# 行驶里程 
float64 total_kilometres
# 剩余电量 
float64 remaining_electricity
# 剩余油量
float64 remaining_oil
# 左转向灯状态  true: 开, false: 关
bool left_light
# 右转向灯状态  true: 开, false: 关
bool right_light
# 远光灯状态  true: 开, false: 关
bool high_light
# 近光灯状态  true: 开, false: 关
bool low_light
# 刹车灯状态  true: 开, false: 关
bool brake_light
# 危险报警灯状态  true: 开, false: 关
bool emergency_light
# 前雾灯状态  true: 开, false: 关
bool front_foggy_light
# 后雾灯状态  true: 开, false: 关
bool rear_foggy_light
# 喇叭  true: 开, false: 关
bool horn
bool position_light
bool wiper
bool reverse_light

# 大灯=前照灯=近光灯+远光灯，小灯＝示宽光
bool head_light

# 自动驾驶开关 true: 开, false: 关
bool auto_switch 
# 制动压力：什么物理量呢？
float64 brake_pressure


# 0：未介入；1：介入； 2：错误； 3：无效
uint8 brake_intervene
uint8 estop_intervene
uint8 steer_intervene
uint8 aeb_intervene #for x6000
uint8 throttle_intervene #for x6000

bool timeout_status
uint8 remote_button_status

bool ready

uint8 auto_max_speed_button  #自动驾驶调速按钮  0 不变  1 up 2 down
bool electromagnetic_silence
bool temporary_parking
uint8 fault_level #故障等级 0 无故障 1 一级故障 2 二级故障 3 三级故障

uint8 brake_work_status 
uint8 parking_work_status 

# 底盘状态   （xb项目添加 2024.3.1 yqx）
uint32 chassis_status
# 底盘控制模式（xb项目添加 2024.3.1 yqx）
uint8 tcu_ctr_mode
# 发动机点火信号 true 已启动 false未启动（xb项目添加 2024.3.1 yqx）
bool engine_started
# 分动器当前档位  00：高档 01：低档 10：空档 0606ADD （xb项目添加 2024.3.1 yqx）
uint8 tran_gear_feedback
# 前桥转向模式（xb项目添加 2024.3.1 yqx）
uint8 front_axle_turn_mode
# 后桥转向模式（xb项目添加 2024.3.1 yqx）
uint8 rear_axle_turn_mode
# 防空灯状态  true: 开, false: 关（xb项目添加 2024.3.11 yqx）
bool air_lamp_fbk
# 进气加热（xb项目添加 2024.3.11 yqx）
bool m_GasHeatFlag
# 发动机启动或停止（xb项目添加 2024.3.11 yqx）
uint8 engine_on_off