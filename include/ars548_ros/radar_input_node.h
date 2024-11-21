#ifndef RADAR_INPUT_PUBLISHER_H
#define RADAR_INPUT_PUBLISHER_H

#include <ros/ros.h>
#include "ars548_ros/AccelerationLateralCog.h"
#include "ars548_ros/AccelerationLongitudinalCog.h"
#include "ars548_ros/CharacteristicSpeed.h"
#include "ars548_ros/DrivingDirection.h"
#include "ars548_ros/SteeringAngleFrontAxle.h"
#include "ars548_ros/VelocityVehicle.h"
#include "ars548_ros/YawRate.h"
#include "ars548_ros/ChassisReport.h"
#include "ars548_ros/Localization.h"

class RadarInputPublisher
{
public:
    RadarInputPublisher(ros::NodeHandle& nh);
    void publishAll();

private:
    void publishAccLateralCog();
    void publishAccLongitudinalCog();
    void publishCharacteristicSpeed();
    void publishDrivingDirection();
    void publishSteeringAngle();
    void publishVelocityVehicle();
    void publishYawRate();

    ros::Publisher acc_lateral_cog_pub_;
    ros::Publisher acc_longitudinal_cog_pub_;
    ros::Publisher characteristic_speed_pub_;
    ros::Publisher driving_direction_pub_;
    ros::Publisher steering_angle_pub_;
    ros::Publisher velocity_vehicle_pub_;
    ros::Publisher yaw_rate_pub_;

    ros::Subscriber odom_sub;
    ros::Subscriber chassis_sub;

    void odom_callback(const ars548_ros::Localization::ConstPtr &odom);
    void chassis_callback(const ars548_ros::ChassisReport::ConstPtr &chassis);

    int gear_location;
    double current_velocity;
    double current_acceleration;
    double yaw_rate;
    double steering_wheel_angle;


};

#endif // RADAR_INPUT_PUBLISHER_H
