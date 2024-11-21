#include "ars548_ros/radar_input_node.h"

RadarInputPublisher::RadarInputPublisher(ros::NodeHandle& nh) : gear_location(0), current_velocity(0), current_acceleration(0), yaw_rate(0), steering_wheel_angle(0)
{
    acc_lateral_cog_pub_ = nh.advertise<ars548_ros::AccelerationLateralCog>("/ars548_process/acc_lateral_cog", 10);
    acc_longitudinal_cog_pub_ = nh.advertise<ars548_ros::AccelerationLongitudinalCog>("/ars548_process/acc_longitudinal_cog", 10);
    characteristic_speed_pub_ = nh.advertise<ars548_ros::CharacteristicSpeed>("/ars548_process/characteristic_speed", 10);
    driving_direction_pub_ = nh.advertise<ars548_ros::DrivingDirection>("/ars548_process/driving_direction", 10);
    steering_angle_pub_ = nh.advertise<ars548_ros::SteeringAngleFrontAxle>("/ars548_process/steering_angle", 10);
    velocity_vehicle_pub_ = nh.advertise<ars548_ros::VelocityVehicle>("/ars548_process/velocity_vehicle", 10);
    yaw_rate_pub_ = nh.advertise<ars548_ros::YawRate>("/ars548_process/yaw_rate", 10);

    odom_sub = nh.subscribe("/odomData",1,&RadarInputPublisher::odom_callback, this);//接收自车惯导信息，输入前向毫米波雷达
    chassis_sub = nh.subscribe("/chassis",1,&RadarInputPublisher::chassis_callback, this);//接收自车底盘信息，输入前向毫米波雷达
}

void RadarInputPublisher::odom_callback(const ars548_ros::Localization::ConstPtr &odom){
    yaw_rate = -odom->original_ins.angular_z * 180.0 / M_PI;//定义为顺时针为正，要取负数
}

void RadarInputPublisher::chassis_callback(const ars548_ros::ChassisReport::ConstPtr &chassis){
    gear_location = chassis->gear_location; //空挡：0 前进档：1-6 倒档：7 无效：8 
    current_velocity = chassis->current_velocity * 3.6;//m/s转换为km/h
    current_acceleration = chassis->current_acceleration; //单位m/s^2
    steering_wheel_angle = chassis->steering_wheel_angle;//方向盘转角 左正右负
}

void RadarInputPublisher::publishAccLateralCog()
{
    ars548_ros::AccelerationLateralCog msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.AccelerationLateralErrAmp = 0;
    msg.AccelerationLateralErrAmp_InvalidFlag = 0;
    msg.QualifierAccelerationLateral = 0;
    msg.AccelerationLateral = 0;//横向加速度 -65～65m/s²
    msg.AccelerationLateral_InvalidFlag = 0;
    msg.AccelerationLateralEventDataQualifier = 0;
    acc_lateral_cog_pub_.publish(msg);
}

void RadarInputPublisher::publishAccLongitudinalCog()
{
    ars548_ros::AccelerationLongitudinalCog msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.AccelerationLongitudinalErrAmp = 0;
    msg.AccelerationLongitudinalErrAmp_InvalidFlag = 0;
    msg.QualifierAccelerationLongitudinal = 0;
    msg.AccelerationLongitudinal = current_acceleration;//纵向加速度 -65～65m/s²
    msg.AccelerationLongitudinal_InvalidFlag = 0;
    msg.AccelerationLongitudinalEventDataQualifier = 0;
    acc_longitudinal_cog_pub_.publish(msg);
}

void RadarInputPublisher::publishCharacteristicSpeed()
{
    ars548_ros::CharacteristicSpeed msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.CharacteristicSpeedErrAmp = 0;
    msg.QualifierCharacteristicSpeed = 0;
    msg.CharacteristicSpeed = 60;//0～255 分辨率为1	km/h
    characteristic_speed_pub_.publish(msg);
}

void RadarInputPublisher::publishDrivingDirection()
{
    ars548_ros::DrivingDirection msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.DrivingDirectionUnconfirmed = 0;
    if (gear_location == 7) {//倒挡
        msg.DrivingDirectionConfirmed = 2; 
    } else if (gear_location == 0) {//空挡
        msg.DrivingDirectionConfirmed = 0; 
    } else {//前进档
        msg.DrivingDirectionConfirmed = 1;
    }
    driving_direction_pub_.publish(msg);
}

void RadarInputPublisher::publishSteeringAngle()
{
    ars548_ros::SteeringAngleFrontAxle msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.QualifierSteeringAngleFrontAxle = 0;
    msg.SteeringAngleFrontAxleErrAmp = 0;
    msg.SteeringAngleFrontAxleErrAmp_InvalidFlag = 0;
    msg.SteeringAngleFrontAxle = steering_wheel_angle;//方向盘转角 -90～90
    msg.SteeringAngleFrontAxle_InvalidFlag = 0;
    msg.SteeringAngleFrontAxleEventDataQualifier = 0;
    steering_angle_pub_.publish(msg);
}

void RadarInputPublisher::publishVelocityVehicle()
{
    ars548_ros::VelocityVehicle msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.StatusVelocityNearStandstill = 0;
    msg.QualifierVelocityVehicle = 0;
    msg.VelocityVehicleEventDataQualifier = 0;
    msg.VelocityVehicle = current_velocity;//车辆速度 0～350 km/h
    msg.VelocityVehicle_InvalidFlag = 0;
    velocity_vehicle_pub_.publish(msg);
}

void RadarInputPublisher::publishYawRate()
{
    ars548_ros::YawRate msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.YawRateErrAmp = 0;
    msg.YawRateErrAmp_InvalidFlag = 0;
    msg.QualifierYawRate = 0;
    msg.YawRate = yaw_rate;//车辆偏航率-163.84～163.83 deg/s
    msg.YawRate_InvalidFlag = 0;
    msg.YawRateEventDataQualifier = 0;
    yaw_rate_pub_.publish(msg);
}

void RadarInputPublisher::publishAll()
{
    publishAccLateralCog();
    publishAccLongitudinalCog();
    publishCharacteristicSpeed();
    publishDrivingDirection();
    publishSteeringAngle();
    publishVelocityVehicle();
    publishYawRate();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_input_node");
    ros::NodeHandle nh;

    RadarInputPublisher radar_publisher(nh);

    ros::Rate rate(10);

    while (ros::ok())
    {
        radar_publisher.publishAll();
        rate.sleep();
    }

    return 0;
}
