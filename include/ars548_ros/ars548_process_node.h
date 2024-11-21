#ifndef ARS548_PROCESS_NODE_H
#define ARS548_PROCESS_NODE_H

#include <ros/ros.h>

#include "ars548_ros/AccelerationLateralCog.h"
#include "ars548_ros/AccelerationLongitudinalCog.h"
#include "ars548_ros/CharacteristicSpeed.h"
#include "ars548_ros/DrivingDirection.h"
#include "ars548_ros/SteeringAngleFrontAxle.h"
#include "ars548_ros/VelocityVehicle.h"
#include "ars548_ros/YawRate.h"
#include "ars548_ros.h"
#include "ars548_ros/RadarParameters.h"
#include "ars548_ros/SensorMounting.h"
#include "ars548_ros/VehicleParameters.h"
#include "ars548_ros/NetworkConfiguration.h"

#define RADAR_INTERFACE "10.13.1.166"
#define CONFIGURATION_SOURCE_PORT 42401
#define CONFIGURATION_DESTINATION_PORT 42101
#define CONFIGURATION_METHOD_ID 390
#define CONFIGURATION_MESSAGE_ID 390
#define CONFIGURATION_PDU_LENGTH 56
#define CONFIGURATION_UDP_PAYLOAD 64
#define CONFIGURATION_UDP_LENGTH 72
#define NEW_IP "0.0.0.0"
#define RADAR_IP "10.13.1.113"

class ars548_process
{
private:

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    struct SensorConfiguration radar_cfg;
    struct AccelerationLateralCoG radar_accelerationlateral;
    struct AccelerationLongitudinalCoG radar_accelerationlongtitudinal;
    struct CharacteristicSpeed radar_characteristicspeed;
    struct DrivingDirection radar_drivingdirection;
    struct SteeringAngleFrontAxle radar_steeringanglefrontaxle;
    struct VelocityVehicle radar_velocityvehicle;
    struct Yaw_Rate radar_yawrate;

    float Param_Longitudinal;
    float Param_Lateral;
    float Param_Vertical;
    float Param_Yaw;
    float Param_Pitch;
    int Param_PlugOrientation;
    float Param_Length;
    float Param_Width;
    float Param_Height;
    float Param_Wheelbase;
    int Param_MaximumDistance;
    int Param_FrequencySlot;
    int Param_CycleTime;
    int Param_TimeSlot;
    int Param_HCC;
    int Param_Powersave_Standstill;
    std::string Param_SensorIPAddress_0;
    std::string Param_SensorIPAddress_1;

    ros::Subscriber acc_lateral_sub;
    ros::Subscriber acc_longitudinal_sub;
    ros::Subscriber characteristic_speed_sub;
    ros::Subscriber driving_direction_sub;
    ros::Subscriber steering_angle_sub;
    ros::Subscriber velocity_vehicle_sub;
    ros::Subscriber yaw_rate_sub;
    ros::ServiceServer set_network_configuration;
    ros::ServiceServer set_radar_parameters;
    ros::ServiceServer set_sensor_mounting;
    ros::ServiceServer set_vehicle_parameters;

    int fds;//One to send the data to it
    struct sockaddr_in addrS;

    void InitUDP(void);
    void InitRadarCfg(void);
    bool RadarParametersConfiguration(ars548_ros::RadarParameters::Request& req, ars548_ros::RadarParameters::Response& /*res*/);
    bool SensorMountingConfiguration(ars548_ros::SensorMounting::Request& req, ars548_ros::SensorMounting::Response& /*res*/);
    bool VehicleParametersConfiguration(ars548_ros::VehicleParameters::Request& req, ars548_ros::VehicleParameters::Response& /*res*/);
    bool NetworkConfiguration(ars548_ros::NetworkConfiguration::Request& req, ars548_ros::NetworkConfiguration::Response& /*res*/);
    void AccLateralCogCallBack(const ars548_ros::AccelerationLateralCog& msg);
    void AccLongitudinalCogCallBack(const ars548_ros::AccelerationLongitudinalCog& msg);
    void CharacteristicSpeedCallBack(const ars548_ros::CharacteristicSpeed& msg);
    void DrivingDirectionCallBack(const ars548_ros::DrivingDirection& msg);
    void SteeringAngleCallBack(const ars548_ros::SteeringAngleFrontAxle& msg);
    void VelocityVehicleCallBack(const ars548_ros::VelocityVehicle& msg);
    void YawRateCallBack(const ars548_ros::YawRate& msg);

        /**
     * @brief Changes the endianness of the object received 
     * @tparam v The object to be modified.
     * @return T. The object modified.
     */
    template<typename T>
    T ChangeEndianness(T v){
        T r;
        uint8_t *pv = (uint8_t *)&v, *pr = (uint8_t *)&r;
        for (int i = 0;i <int(sizeof(T)); i++){
            pr[i]=pv[sizeof(T)-1-i];
        }
        return r;
    }

public:
    ars548_process();
    ~ars548_process();
    void run();
};
#endif