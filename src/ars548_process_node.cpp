#include "ars548_ros/ars548_process_node.h"

ars548_process::ars548_process(): private_nh("~"){  
    private_nh.param<std::string>("SensorIPAddress_0", Param_SensorIPAddress_0, "10.13.1.113");
    private_nh.param<std::string>("SensorIPAddress_1", Param_SensorIPAddress_1, "169.254.116.113");
    InitUDP();
    InitRadarCfg();
}

ars548_process::~ars548_process(){
    close(fds);
}

void ars548_process::run(){
    acc_lateral_sub = nh.subscribe("/ars548_process/acc_lateral_cog", 10, &ars548_process::AccLateralCogCallBack,this);
    acc_longitudinal_sub = nh.subscribe("/ars548_process/acc_longitudinal_cog", 10, &ars548_process::AccLongitudinalCogCallBack,this);
    characteristic_speed_sub = nh.subscribe("/ars548_process/characteristic_speed", 10, &ars548_process::CharacteristicSpeedCallBack,this);
    driving_direction_sub = nh.subscribe("/ars548_process/driving_direction", 10, &ars548_process::DrivingDirectionCallBack,this);
    steering_angle_sub = nh.subscribe("/ars548_process/steering_angle", 10, &ars548_process::SteeringAngleCallBack,this);
    velocity_vehicle_sub = nh.subscribe("/ars548_process/velocity_vehicle", 10, &ars548_process::VelocityVehicleCallBack,this);
    yaw_rate_sub = nh.subscribe("/ars548_process/yaw_rate", 10, &ars548_process::YawRateCallBack,this);
    set_radar_parameters = nh.advertiseService("set_radar_parameters", &ars548_process::RadarParametersConfiguration,this);
    set_sensor_mounting = nh.advertiseService("set_sensor_mounting", &ars548_process::SensorMountingConfiguration,this);
    set_vehicle_parameters = nh.advertiseService("set_vehicle_parameters", &ars548_process::VehicleParametersConfiguration,this);
    set_network_configuration = nh.advertiseService("set_network_configuration", &ars548_process::NetworkConfiguration,this);
}

void ars548_process::InitUDP(void)
{
    struct sockaddr_in addr;

    bzero(&addr, sizeof(struct sockaddr_in));
	addr.sin_family=AF_INET;
	addr.sin_addr.s_addr=htonl(INADDR_ANY);
	addr.sin_port=htons(CONFIGURATION_SOURCE_PORT); //42401

    fds = socket(AF_INET, SOCK_DGRAM, 0);
    if (fds < 0) {
        perror("socket");
        return ;
    }

    if(bind(fds,(struct sockaddr *)(&addr),sizeof(struct sockaddr_in))<0)
	{
		perror("Binding the socket!");
		return;
	}
    	
    std::memset(&addrS, 0, sizeof(addrS));
    addrS.sin_family = AF_INET;
    addrS.sin_port = htons(CONFIGURATION_DESTINATION_PORT);//42101
    if (inet_pton(AF_INET, RADAR_IP, &addrS.sin_addr) <= 0) {
        perror("inet_pton");
        return;
    }
}

void ars548_process::InitRadarCfg(void)
{
    radar_cfg.ServiceID = 0;
    radar_cfg.MethodID = 390;
    radar_cfg.PayloadLength = 56;
    radar_cfg.SensorIPAddress_0 = static_cast<uint32_t>(inet_addr(Param_SensorIPAddress_0.c_str()));
    radar_cfg.SensorIPAddress_1 = static_cast<uint32_t>(inet_addr(Param_SensorIPAddress_1.c_str()));
}
bool ars548_process::NetworkConfiguration(ars548_ros::NetworkConfiguration::Request& req, ars548_ros::NetworkConfiguration::Response& /*res*/)
{
    struct SensorConfiguration sc;
    sc.ServiceID = ChangeEndianness(radar_cfg.ServiceID);
    sc.MethodID = ChangeEndianness(radar_cfg.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_cfg.PayloadLength);
    if(radar_cfg.SensorIPAddress_0 >= 0 && radar_cfg.SensorIPAddress_0 <= 4294967295){
        sc.SensorIPAddress_0 = ChangeEndianness(radar_cfg.SensorIPAddress_0);
    }
    else{
        std::cout<<"Error SensorIPAddress_0"<<std::endl;
        return false;
    }
    if(radar_cfg.SensorIPAddress_1 >= 0 && radar_cfg.SensorIPAddress_1 <= 4294967295){
        sc.SensorIPAddress_1 = ChangeEndianness(radar_cfg.SensorIPAddress_1);
    }
    else{
        std::cout<<"Error SensorIPAddress_1"<<std::endl;
        return false;
    }
    if(req.NewNetworkConfiguration >= 0 && req.NewNetworkConfiguration <= 1){
        sc.NewNetworkConfiguration = ChangeEndianness(req.NewNetworkConfiguration);
    }
    else{
        std::cout<<"Error NewNetworkConfiguration"<<std::endl;
        return false;
    }
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the SensorConfiguration message");
        return false;
    }
    return true;
}

bool ars548_process::RadarParametersConfiguration(ars548_ros::RadarParameters::Request& req, ars548_ros::RadarParameters::Response& /*res*/)
{
    struct SensorConfiguration sc;
    sc.ServiceID = ChangeEndianness(radar_cfg.ServiceID);
    sc.MethodID = ChangeEndianness(radar_cfg.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_cfg.PayloadLength);
    if(req.MaximumDistance >= 93 && req.MaximumDistance <= 1514){
        sc.MaximumDistance = ChangeEndianness(req.MaximumDistance);
    }
    else{
        std::cout<<"Error MaximumDistance"<<std::endl;
        return false;
    }
    if(req.FrequencySlot >= 0 && req.FrequencySlot <= 2){
        sc.FrequencySlot = ChangeEndianness(req.FrequencySlot);
    }
    else{
        std::cout<<"Error FrequencySlot"<<std::endl;
        return false;
    }
    if(req.CycleTime >= 50 && req.CycleTime <= 100){
        sc.CycleTime = ChangeEndianness(req.CycleTime);
    }
    else{
        std::cout<<"Error CycleTime"<<std::endl;
        return false;
    }
    if(req.TimeSlot >= 10 && req.TimeSlot <= 90){
        sc.TimeSlot = ChangeEndianness(req.TimeSlot);
    }
    else{
        std::cout<<"Error TimeSlot"<<std::endl;
        return false;
    }
    if(req.HCC >= 1 && req.HCC <= 2){
        sc.HCC = ChangeEndianness(req.HCC);
    }
    else{
        std::cout<<"Error HCC"<<std::endl;
        return false;
    }
    if(req.Powersave_Standstill >= 0 && req.Powersave_Standstill <= 1){
        sc.Powersave_Standstill = ChangeEndianness(req.Powersave_Standstill);
    }
    else{
        std::cout<<"Error Powersave_Standstill"<<std::endl;
        return false;
    }
    if(req.NewRadarParameters >= 0 && req.NewRadarParameters <= 1){
        sc.NewRadarParameters = ChangeEndianness(req.NewRadarParameters);
    }
    else{
        std::cout<<"Error NewRadarParameters"<<std::endl;
        return false;
    }
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the SensorConfiguration message");
        return false;
    }
    return true;
}

bool ars548_process::SensorMountingConfiguration(ars548_ros::SensorMounting::Request& req, ars548_ros::SensorMounting::Response& /*res*/)
{
    struct SensorConfiguration sc;
    sc.ServiceID = ChangeEndianness(radar_cfg.ServiceID);
    sc.MethodID = ChangeEndianness(radar_cfg.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_cfg.PayloadLength);
    if(req.Longitudinal >= -100 && req.Longitudinal <= 100){
        sc.Longitudinal = ChangeEndianness(req.Longitudinal);
    }
    else{
        std::cout<<"Error Longitudinal"<<std::endl;
        return false;
    }
    if(req.Lateral >= -100 && req.Lateral <= 100){
        sc.Lateral = ChangeEndianness(req.Lateral);
    }
    else{
        std::cout<<"Error Lateral"<<std::endl;
        return false;
    }
    if(req.Vertical >= 0.01 && req.Vertical <= 10){
        sc.Vertical = ChangeEndianness(req.Vertical);
    }
    else{
        std::cout<<"Error Vertical"<<std::endl;
        return false;
    }
    if(req.Yaw >= -3.14159 && req.Yaw <= 3.14159){
        sc.Yaw = ChangeEndianness(req.Yaw);
    }
    else{
        std::cout<<"Error Yaw"<<std::endl;
        return false;
    }
    if(req.Pitch >= -1.5707 && req.Pitch <= 1.5707){
        sc.Pitch = ChangeEndianness(req.Pitch);
    }
    else{
        std::cout<<"Error Pitch"<<std::endl;
        return false;
    }
    if(req.PlugOrientation >= 0 && req.PlugOrientation <= 1){
        sc.PlugOrientation = ChangeEndianness(req.PlugOrientation);
    }
    else{
        std::cout<<"Error PlugOrientation"<<std::endl;
        return false;
    }
    if(req.NewSensorMounting >= 0 && req.NewSensorMounting <= 1){
        sc.NewSensorMounting = ChangeEndianness(req.NewSensorMounting);
    }
    else{
        std::cout<<"Error NewSensorMounting"<<std::endl;
        return false;
    }
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the SensorConfiguration message");
        return false;
    }
    return true;
}

bool ars548_process::VehicleParametersConfiguration(ars548_ros::VehicleParameters::Request& req, ars548_ros::VehicleParameters::Response& /*res*/)
{
    struct SensorConfiguration sc;
    sc.ServiceID = ChangeEndianness(radar_cfg.ServiceID);
    sc.MethodID = ChangeEndianness(radar_cfg.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_cfg.PayloadLength);
    if(req.Length >= 0.01 && req.Length <= 100){
        sc.Length = ChangeEndianness(req.Length);
    }
    else{
        std::cout<<"Error Length"<<std::endl;
        return false;
    }
    if(req.Width >= 0.01 && req.Width <= 100){
        sc.Width = ChangeEndianness(req.Width);
    }
    else{
        std::cout<<"Error Width"<<std::endl;
        return false;
    }
    if(req.Height >= 0.01 && req.Height <= 100){
        sc.Height = ChangeEndianness(req.Height);
    }
    else{
        std::cout<<"Error Height"<<std::endl;
        return false;
    }
    if(req.Wheelbase >= 0.01 && req.Wheelbase <= 100){
        sc.Wheelbase = ChangeEndianness(req.Wheelbase);
    }
    else{
        std::cout<<"Error Wheelbase"<<std::endl;
        return false;
    }
    if(req.NewVehicleParameters >= 0 && req.NewVehicleParameters <= 1){
        sc.NewVehicleParameters = ChangeEndianness(req.NewVehicleParameters);
    }
    else{
        std::cout<<"Error NewVehicleParameters"<<std::endl;
        return false;
    }
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the SensorConfiguration message");
        return false;
    }
    return true;
}

void ars548_process::AccLateralCogCallBack(const ars548_ros::AccelerationLateralCog& msg)
{   
    radar_accelerationlateral.ServiceID = 0;
    radar_accelerationlateral.MethodID = 321;
    radar_accelerationlateral.PayloadLength = 32;
    radar_accelerationlateral.AccelerationLateralErrAmp = msg.AccelerationLateralErrAmp;
    radar_accelerationlateral.AccelerationLateralErrAmp_InvalidFlag = msg.AccelerationLateralErrAmp_InvalidFlag;
    radar_accelerationlateral.QualifierAccelerationLateral = msg.QualifierAccelerationLateral;
    radar_accelerationlateral.AccelerationLateral = msg.AccelerationLateral;
    radar_accelerationlateral.AccelerationLateral_InvalidFlag = msg.AccelerationLateral_InvalidFlag;
    radar_accelerationlateral.AccelerationLateralEventDataQualifier = msg.AccelerationLateralEventDataQualifier;
    struct  AccelerationLateralCoG sc;
    sc.ServiceID = ChangeEndianness(radar_accelerationlateral.ServiceID);
    sc.MethodID = ChangeEndianness(radar_accelerationlateral.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_accelerationlateral.PayloadLength);
    sc.AccelerationLateralErrAmp = ChangeEndianness(radar_accelerationlateral.AccelerationLateralErrAmp);
    sc.AccelerationLateralErrAmp_InvalidFlag = ChangeEndianness(radar_accelerationlateral.AccelerationLateralErrAmp_InvalidFlag);
    sc.QualifierAccelerationLateral = ChangeEndianness(radar_accelerationlateral.QualifierAccelerationLateral);
    sc.AccelerationLateral = ChangeEndianness(radar_accelerationlateral.AccelerationLateral);
    sc.AccelerationLateral_InvalidFlag = ChangeEndianness(radar_accelerationlateral.AccelerationLateral_InvalidFlag);
    sc.AccelerationLateralEventDataQualifier = ChangeEndianness(radar_accelerationlateral.AccelerationLateralEventDataQualifier);
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the AccelerationLateralCog message");
        return ;
    }
}

void ars548_process::AccLongitudinalCogCallBack(const ars548_ros::AccelerationLongitudinalCog& msg)
{ 
    radar_accelerationlongtitudinal.ServiceID = 0;
    radar_accelerationlongtitudinal.MethodID = 322;
    radar_accelerationlongtitudinal.PayloadLength = 32;
    radar_accelerationlongtitudinal.AccelerationLongitudinalErrAmp = msg.AccelerationLongitudinalErrAmp;
    radar_accelerationlongtitudinal.AccelerationLongitudinalErrAmp_InvalidFlag = msg.AccelerationLongitudinalErrAmp_InvalidFlag;
    radar_accelerationlongtitudinal.QualifierAccelerationLongitudinal = msg.QualifierAccelerationLongitudinal;
    radar_accelerationlongtitudinal.AccelerationLongitudinal = msg.AccelerationLongitudinal;
    radar_accelerationlongtitudinal.AccelerationLongitudinal_InvalidFlag = msg.AccelerationLongitudinal_InvalidFlag;
    radar_accelerationlongtitudinal.AccelerationLongitudinalEventDataQualifier = msg.AccelerationLongitudinalEventDataQualifier;
    struct AccelerationLongitudinalCoG sc;
    sc.ServiceID = ChangeEndianness(radar_accelerationlongtitudinal.ServiceID);
    sc.MethodID = ChangeEndianness(radar_accelerationlongtitudinal.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_accelerationlongtitudinal.PayloadLength);
    sc.AccelerationLongitudinalErrAmp = ChangeEndianness(radar_accelerationlongtitudinal.AccelerationLongitudinalErrAmp);
    sc.AccelerationLongitudinalErrAmp_InvalidFlag = ChangeEndianness(radar_accelerationlongtitudinal.AccelerationLongitudinalErrAmp_InvalidFlag);
    sc.QualifierAccelerationLongitudinal = ChangeEndianness(radar_accelerationlongtitudinal.QualifierAccelerationLongitudinal);
    sc.AccelerationLongitudinal = ChangeEndianness(radar_accelerationlongtitudinal.AccelerationLongitudinal);
    sc.AccelerationLongitudinal_InvalidFlag = ChangeEndianness(radar_accelerationlongtitudinal.AccelerationLongitudinal_InvalidFlag);
    sc.AccelerationLongitudinalEventDataQualifier = ChangeEndianness(radar_accelerationlongtitudinal.AccelerationLongitudinalEventDataQualifier);
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the AccelerationLongitudinalCog message");
        return ;
    }
}

void ars548_process::CharacteristicSpeedCallBack(const ars548_ros::CharacteristicSpeed& msg)
{ 
    radar_characteristicspeed.ServiceID = 0;
    radar_characteristicspeed.MethodID = 328;
    radar_characteristicspeed.PayloadLength = 11;
    radar_characteristicspeed.CharacteristicSpeedErrAmp = msg.CharacteristicSpeedErrAmp;
    radar_characteristicspeed.QualifierCharacteristicSpeed = msg.QualifierCharacteristicSpeed;
    radar_characteristicspeed.CharacteristicSpeed = msg.CharacteristicSpeed;
    struct CharacteristicSpeed sc;
    sc.ServiceID = ChangeEndianness(radar_characteristicspeed.ServiceID);
    sc.MethodID = ChangeEndianness(radar_characteristicspeed.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_characteristicspeed.PayloadLength);
    sc.CharacteristicSpeedErrAmp = ChangeEndianness(radar_characteristicspeed.CharacteristicSpeedErrAmp);
    sc.QualifierCharacteristicSpeed = ChangeEndianness(radar_characteristicspeed.QualifierCharacteristicSpeed);
    sc.CharacteristicSpeed = ChangeEndianness(radar_characteristicspeed.CharacteristicSpeed);
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the CharacteristicSpeed message");
        return ;
    }
}

void ars548_process::DrivingDirectionCallBack(const ars548_ros::DrivingDirection& msg)
{  
    radar_drivingdirection.ServiceID = 0;
    radar_drivingdirection.MethodID = 325;
    radar_drivingdirection.PayloadLength = 22;
    radar_drivingdirection.DrivingDirectionUnconfirmed = msg.DrivingDirectionUnconfirmed;
    radar_drivingdirection.DrivingDirectionConfirmed = msg.DrivingDirectionConfirmed;
    struct DrivingDirection sc;
    sc.ServiceID = ChangeEndianness(radar_drivingdirection.ServiceID);
    sc.MethodID = ChangeEndianness(radar_drivingdirection.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_drivingdirection.PayloadLength);
    sc.DrivingDirectionUnconfirmed = ChangeEndianness(radar_drivingdirection.DrivingDirectionUnconfirmed);
    sc.DrivingDirectionConfirmed = ChangeEndianness(radar_drivingdirection.DrivingDirectionConfirmed);
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the DrivingDirection message");
        return ;
    }
}

void ars548_process::SteeringAngleCallBack(const ars548_ros::SteeringAngleFrontAxle& msg)
{
    radar_steeringanglefrontaxle.ServiceID = 0;
    radar_steeringanglefrontaxle.MethodID = 327;
    radar_steeringanglefrontaxle.PayloadLength = 32;
    radar_steeringanglefrontaxle.QualifierSteeringAngleFrontAxle = msg.QualifierSteeringAngleFrontAxle;
    radar_steeringanglefrontaxle.SteeringAngleFrontAxleErrAmp = msg.SteeringAngleFrontAxleErrAmp;
    radar_steeringanglefrontaxle.SteeringAngleFrontAxleErrAmp_InvalidFlag = msg.SteeringAngleFrontAxleErrAmp_InvalidFlag;
    radar_steeringanglefrontaxle.SteeringAngleFrontAxle = msg.SteeringAngleFrontAxle;
    radar_steeringanglefrontaxle.SteeringAngleFrontAxle_InvalidFlag = msg.SteeringAngleFrontAxle_InvalidFlag;
    radar_steeringanglefrontaxle.SteeringAngleFrontAxleEventDataQualifier = msg.SteeringAngleFrontAxleEventDataQualifier;
    struct  SteeringAngleFrontAxle sc;
    sc.ServiceID = ChangeEndianness(radar_steeringanglefrontaxle.ServiceID);
    sc.MethodID = ChangeEndianness(radar_steeringanglefrontaxle.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_steeringanglefrontaxle.PayloadLength);
    sc.QualifierSteeringAngleFrontAxle = ChangeEndianness(radar_steeringanglefrontaxle.QualifierSteeringAngleFrontAxle);
    sc.SteeringAngleFrontAxleErrAmp = ChangeEndianness(radar_steeringanglefrontaxle.SteeringAngleFrontAxleErrAmp);
    sc.SteeringAngleFrontAxleErrAmp_InvalidFlag = ChangeEndianness(radar_steeringanglefrontaxle.SteeringAngleFrontAxleErrAmp_InvalidFlag);
    sc.SteeringAngleFrontAxle = ChangeEndianness(radar_steeringanglefrontaxle.SteeringAngleFrontAxle);
    sc.SteeringAngleFrontAxle_InvalidFlag = ChangeEndianness(radar_steeringanglefrontaxle.SteeringAngleFrontAxle_InvalidFlag);
    sc.SteeringAngleFrontAxleEventDataQualifier = ChangeEndianness(radar_steeringanglefrontaxle.SteeringAngleFrontAxleEventDataQualifier);
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the SteeringAngleFrontAxle message");
        return ;
    }
}

void ars548_process::VelocityVehicleCallBack(const ars548_ros::VelocityVehicle& msg)
{
    radar_velocityvehicle.ServiceID = 0;
    radar_velocityvehicle.MethodID = 323;
    radar_velocityvehicle.PayloadLength = 28;
    radar_velocityvehicle.StatusVelocityNearStandstill = msg.StatusVelocityNearStandstill;
    radar_velocityvehicle.QualifierVelocityVehicle = msg.QualifierVelocityVehicle;
    radar_velocityvehicle.VelocityVehicleEventDataQualifier = msg.VelocityVehicleEventDataQualifier;
    radar_velocityvehicle.VelocityVehicle = msg.VelocityVehicle;
    radar_velocityvehicle.VelocityVehicle_InvalidFlag = msg.VelocityVehicle_InvalidFlag;
    struct VelocityVehicle sc;
    sc.ServiceID = ChangeEndianness(radar_velocityvehicle.ServiceID);
    sc.MethodID = ChangeEndianness(radar_velocityvehicle.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_velocityvehicle.PayloadLength);
    sc.StatusVelocityNearStandstill = ChangeEndianness(radar_velocityvehicle.StatusVelocityNearStandstill);
    sc.QualifierVelocityVehicle = ChangeEndianness(radar_velocityvehicle.QualifierVelocityVehicle);
    sc.VelocityVehicleEventDataQualifier = ChangeEndianness(radar_velocityvehicle.VelocityVehicleEventDataQualifier);
    sc.VelocityVehicle = ChangeEndianness(radar_velocityvehicle.VelocityVehicle);
    sc.VelocityVehicle_InvalidFlag = ChangeEndianness(radar_velocityvehicle.VelocityVehicle_InvalidFlag);
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the VelocityVehicle message");
        return ;
    }
}

void ars548_process::YawRateCallBack(const ars548_ros::YawRate& msg)
{
    radar_yawrate.ServiceID = 0;
    radar_yawrate.MethodID = 326;
    radar_yawrate.PayloadLength = 32;
    radar_yawrate.YawRateErrAmp = msg.YawRateErrAmp;
    radar_yawrate.YawRateErrAmp_InvalidFlag = msg.YawRateErrAmp_InvalidFlag;
    radar_yawrate.QualifierYawRate = msg.QualifierYawRate;
    radar_yawrate.YawRate = msg.YawRate;
    radar_yawrate.YawRate_InvalidFlag = msg.YawRate_InvalidFlag;
    radar_yawrate.YawRateEventDataQualifier = msg.YawRateEventDataQualifier; 
    struct Yaw_Rate sc;
    sc.ServiceID = ChangeEndianness(radar_yawrate.ServiceID);
    sc.MethodID = ChangeEndianness(radar_yawrate.MethodID);
    sc.PayloadLength = ChangeEndianness(radar_yawrate.PayloadLength);
    sc.YawRateErrAmp = ChangeEndianness(radar_yawrate.YawRateErrAmp);
    sc.YawRateErrAmp_InvalidFlag = ChangeEndianness(radar_yawrate.YawRateErrAmp_InvalidFlag);
    sc.QualifierYawRate = ChangeEndianness(radar_yawrate.QualifierYawRate);
    sc.YawRate = ChangeEndianness(radar_yawrate.YawRate);
    sc.YawRate_InvalidFlag = ChangeEndianness(radar_yawrate.YawRate_InvalidFlag);
    sc.YawRateEventDataQualifier = ChangeEndianness(radar_yawrate.YawRateEventDataQualifier);
    int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,sizeof(addrS));
    if (sent_bytes < 0) {
        perror("Failed sending the YawRate message");
        return ;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ars548_process_node");
    ars548_process ars548;
    ars548.run();
    ros::spin();
    return 0;
}


