#include "ars548_ros/ars548_ros.h"

ars548_driver_class::ars548_driver_class():modifierDetection(cloud_msgDetect){
    // 参数初始化
    nh_.param("radarIP", ars548_IP, std::string(DEFAULT_RADAR_IP));
    nh_.param("radarPort", ars548_Port, DEFAULT_RADAR_PORT);
    nh_.param("frameID", frame_ID, std::string(DEFAULT_FRAME_ID));


    //Creation of their modifiers

    //Detection Cloud
    modifierDetection.setPointCloud2Fields(8,
                                        "x",1,sensor_msgs::PointField::FLOAT32,
                                        "y",1,sensor_msgs::PointField::FLOAT32,
                                        "z",1,sensor_msgs::PointField::FLOAT32,
                                        "r",1,sensor_msgs::PointField::FLOAT32,
                                        "v",1,sensor_msgs::PointField::FLOAT32,
                                        "RCS",1,sensor_msgs::PointField::INT8,
                                        "azimuth",1,sensor_msgs::PointField::FLOAT32,
                                        "elevation",1,sensor_msgs::PointField::FLOAT32
                                        );
    modifierDetection.reserve(SIZE);
    modifierDetection.clear();

    cloud_Direction.poses.reserve(SIZE);
}

ars548_driver_class::~ars548_driver_class() {
    if (fd >= 0) {
        close(fd);
    }
}

void ars548_driver_class::run() {
    // 初始化发布者
    statusPublisher = nh_.advertise<ars548_ros::Status>("Status", 10);
    objectPublisher = nh_.advertise<ars548_ros::ObjectList4D>("ObjectList", 10);
    detectionsPublisher = nh_.advertise<ars548_ros::DetectionList>("DetectionList", 10);
    directionPublisher = nh_.advertise<geometry_msgs::PoseArray>("DirectionVelocity", 10);
    pubObj = nh_.advertise<visualization_msgs::MarkerArray>("PointCloudObject", 10);
    pubDetect = nh_.advertise<sensor_msgs::PointCloud2>("PointCloudDetection", 10);
    start_read();
}

void ars548_driver_class::start_read() {
    // Initialize socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return;
    }
    struct sockaddr_in addr;
    u_int yes = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes)) < 0) {
        perror("Reusing ADDR failed");
        return;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(ars548_Port); //42102

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return;
    }

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(ars548_IP.c_str()); //224.0.2.2
    mreq.imr_interface.s_addr = inet_addr(RADAR_INTERFACE); //10.13.1.166

    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0) {
        perror("setsockopt");
        return;
    }
    socklen_t addrlen = sizeof(addr);
    // Main loop
    while (ros::ok()) {
        memset(msgbuf, 0, MSGBUFSIZE);
        nbytes = recvfrom(fd, msgbuf, MSGBUFSIZE, 0, (struct sockaddr*)&addr, &addrlen);
        if (nbytes < 0) {
            perror("recvfrom error");
            return;
        }
        //Creation of the Point Cloud iterators.
        //Detection Iterators
        sensor_msgs::PointCloud2Iterator<float> iter_xD(cloud_msgDetect,"x");
        sensor_msgs::PointCloud2Iterator<float> iter_yD(cloud_msgDetect,"y");
        sensor_msgs::PointCloud2Iterator<float> iter_zD(cloud_msgDetect,"z");
        sensor_msgs::PointCloud2Iterator<float> iter_rD(cloud_msgDetect,"r");
        sensor_msgs::PointCloud2Iterator<float> iter_vD(cloud_msgDetect,"v");
        sensor_msgs::PointCloud2Iterator<int8_t> iter_RCSD(cloud_msgDetect,"RCS");
        sensor_msgs::PointCloud2Iterator<float> iter_azimuthD(cloud_msgDetect,"azimuth");
        sensor_msgs::PointCloud2Iterator<float> iter_elevationD(cloud_msgDetect,"elevation");
        switch (nbytes) {
            case STATUS_MESSAGE_PAYLOAD: {
                UDPStatus status;
                memcpy(&status, msgbuf, sizeof(UDPStatus));
                status.ServiceID = ChangeEndianness(status.ServiceID);
                status.MethodID = ChangeEndianness(status.MethodID);
                status.PayloadLength = ChangeEndianness(status.PayloadLength);
                if (status.MethodID == STATUS_MESSAGE_METHOD_ID && status.PayloadLength == STATUS_MESSAGE_PDU_LENGTH) {
                    ars548_ros::Status statusMessage;
                    status = modifyStatus(status);
                    fillStatusMessage(statusMessage, status);
                    statusPublisher.publish(statusMessage);
                }
                break;
            }
            case OBJECT_MESSAGE_PAYLOAD: {
                Object_List objectList;
                visualization_msgs::MarkerArray objectMarkers;
                memcpy(&objectList, msgbuf, sizeof(Object_List));
                objectList.ServiceID = ChangeEndianness(objectList.ServiceID);
                objectList.MethodID = ChangeEndianness(objectList.MethodID);
                objectList.PayloadLength = ChangeEndianness(objectList.PayloadLength);
                if (objectList.MethodID == OBJECT_MESSAGE_METHOD_ID && objectList.PayloadLength == OBJECT_MESSAGE_PDU_LENGTH) {
                    ars548_ros::ObjectList4D objectMessage;
                    objectList = modifyObjectList(objectList);
                    cloud_Direction.poses.resize(objectList.ObjectList_NumOfObjects);
                    fillObjectMessage(objectMessage, objectList);   
                    for (uint32_t i = 0; i < objectList.ObjectList_NumOfObjects; ++i) {
                        visualization_msgs::Marker objectMarker;
                        fillMarkerMessage(objectMarker, objectList, i);
                        objectMarkers.markers.push_back(objectMarker);
                        fillDirectionMessage(cloud_Direction, objectList, i);
                    }
                    pubObj.publish(objectMarkers);
                    directionPublisher.publish(cloud_Direction);
                    objectPublisher.publish(objectMessage);
                }
                break;
            }
            case DETECTION_MESSAGE_PAYLOAD: {
                DetectionList detectionList;
                memcpy(&detectionList, msgbuf, sizeof(DetectionList));
                detectionList.ServiceID = ChangeEndianness(detectionList.ServiceID);
                detectionList.MethodID = ChangeEndianness(detectionList.MethodID);
                detectionList.PayloadLength = ChangeEndianness(detectionList.PayloadLength);
                if (detectionList.MethodID == DETECTION_MESSAGE_METHOD_ID && detectionList.PayloadLength == DETECTION_MESSAGE_PDU_LENGTH) {
                    ars548_ros::DetectionList detectionMessage;
                    detectionList = modifyDetectionList(detectionList);
                    fillDetectionMessage(detectionMessage, detectionList);
                    fillCloudMessage(cloud_msgDetect, detectionList.List_NumOfDetections);           
                    for (uint64_t i = 0; i < detectionList.List_NumOfDetections; ++i,++iter_xD,++iter_yD,++iter_zD,
                        ++iter_vD, ++iter_rD, ++iter_RCSD,++iter_azimuthD, ++iter_elevationD) {
                        float posX = detectionList.List_Detections[i].f_Range * std::cos(detectionList.List_Detections[i].f_ElevationAngle) * std::cos(detectionList.List_Detections[i].f_AzimuthAngle);
                        float posY = detectionList.List_Detections[i].f_Range * std::cos(detectionList.List_Detections[i].f_ElevationAngle) * std::sin(detectionList.List_Detections[i].f_AzimuthAngle);
                        float posZ = detectionList.List_Detections[i].f_Range * std::sin(detectionList.List_Detections[i].f_ElevationAngle);
                        *iter_xD = posX;
                        *iter_yD = posY;
                        *iter_zD = posZ;
                        *iter_rD = detectionList.List_Detections[i].f_Range;
                        *iter_vD = detectionList.List_Detections[i].f_RangeRate;
                        *iter_RCSD = detectionList.List_Detections[i].s_RCS;
                        *iter_azimuthD = detectionList.List_Detections[i].f_AzimuthAngle;
                        *iter_elevationD = detectionList.List_Detections[i].f_ElevationAngle;
                    }
                    pubDetect.publish(cloud_msgDetect);
                    detectionsPublisher.publish(detectionMessage);
                }
                break;
            }
        }
    }
    return;
}

UDPStatus ars548_driver_class::modifyStatus(UDPStatus status){
    status.Timestamp_Nanoseconds=ChangeEndianness(status.Timestamp_Nanoseconds);
    status.Timestamp_Seconds=ChangeEndianness(status.Timestamp_Seconds);
    status.Timestamp_SyncStatus=ChangeEndianness(status.Timestamp_SyncStatus);
    status.SWVersion_Major=ChangeEndianness(status.SWVersion_Major);
    status.SWVersion_Minor=ChangeEndianness(status.SWVersion_Minor);
    status.SWVersion_Patch=ChangeEndianness(status.SWVersion_Patch);
    status.Longitudinal=ChangeEndianness(status.Longitudinal);
    status.Lateral=ChangeEndianness(status.Lateral);
    status.Vertical=ChangeEndianness(status.Vertical);
    status.Yaw=ChangeEndianness(status.Yaw);
    status.Pitch=ChangeEndianness(status.Pitch);
    status.PlugOrientation=ChangeEndianness(status.PlugOrientation);
    status.Length=ChangeEndianness(status.Length);
    status.Width=ChangeEndianness(status.Width);
    status.Height=ChangeEndianness(status.Height);
    status.Wheelbase=ChangeEndianness(status.Wheelbase);
    status.MaximunDistance=ChangeEndianness(status.MaximunDistance);
    status.FrequencySlot=ChangeEndianness(status.FrequencySlot);
    status.CycleTime=ChangeEndianness(status.CycleTime);
    status.TimeSlot=ChangeEndianness(status.TimeSlot);
    status.HCC=ChangeEndianness(status.HCC);
    status.Powersave_Standstill=ChangeEndianness(status.Powersave_Standstill);
    status.SensorIPAddress_0=ChangeEndianness(status.SensorIPAddress_0);
    status.SensorIPAddress_1=ChangeEndianness(status.SensorIPAddress_1);
    status.ConfigurationCounter=ChangeEndianness(status.ConfigurationCounter);
    status.Status_LongitudinalVelocity=ChangeEndianness(status.Status_LongitudinalVelocity);
    status.Status_LongitudinalAcceleration=ChangeEndianness(status.Status_LongitudinalAcceleration);
    status.Status_LateralAcceleration=ChangeEndianness(status.Status_LateralAcceleration);
    status.Status_YawRate=ChangeEndianness(status.Status_YawRate);
    status.Status_SteeringAngle=ChangeEndianness(status.Status_SteeringAngle);
    status.Status_DrivingDirection=ChangeEndianness(status.Status_DrivingDirection);
    status.Status_CharacteristicSpeed=ChangeEndianness(status.Status_CharacteristicSpeed);
    status.Status_RadarStatus=ChangeEndianness(status.Status_RadarStatus);
    status.Status_VoltageStatus=ChangeEndianness(status.Status_VoltageStatus);
    status.Status_TemperatureStatus=ChangeEndianness(status.Status_TemperatureStatus);
    status.Status_BlockageStatus=ChangeEndianness(status.Status_BlockageStatus);
    return status;
}

Object_List ars548_driver_class::modifyObjectList(Object_List object_List){
    object_List.CRC=ChangeEndianness(object_List.CRC);
    object_List.Length=ChangeEndianness(object_List.Length);
    object_List.SQC=ChangeEndianness(object_List.SQC);
    object_List.DataID=ChangeEndianness(object_List.DataID);
    object_List.Timestamp_Nanoseconds=ChangeEndianness(object_List.Timestamp_Nanoseconds);
    object_List.Timestamp_Seconds=ChangeEndianness(object_List.Timestamp_Seconds);
    object_List.Timestamp_SyncStatus=ChangeEndianness(object_List.Timestamp_SyncStatus);
    object_List.EventDataQualifier=ChangeEndianness(object_List.EventDataQualifier);
    object_List.ObjectList_NumOfObjects=ChangeEndianness(object_List.ObjectList_NumOfObjects);
    if (object_List.ObjectList_NumOfObjects>MAX_OBJECTS){
        object_List.ObjectList_NumOfObjects=MAX_OBJECTS;
    }
    for(u_int32_t i =0; i<object_List.ObjectList_NumOfObjects;++i){
        object_List.ObjectList_Objects[i].u_StatusSensor=ChangeEndianness(object_List.ObjectList_Objects[i].u_StatusSensor);
        object_List.ObjectList_Objects[i].u_ID=ChangeEndianness(object_List.ObjectList_Objects[i].u_ID);
        object_List.ObjectList_Objects[i].u_Age=ChangeEndianness(object_List.ObjectList_Objects[i].u_Age);
        object_List.ObjectList_Objects[i].u_StatusMeasurement=ChangeEndianness(object_List.ObjectList_Objects[i].u_StatusMeasurement);
        object_List.ObjectList_Objects[i].u_StatusMovement=ChangeEndianness(object_List.ObjectList_Objects[i].u_StatusMovement);
        object_List.ObjectList_Objects[i].u_Position_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_InvalidFlags);
        object_List.ObjectList_Objects[i].u_Position_Reference=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Reference);
        object_List.ObjectList_Objects[i].u_Position_X=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_X);
        object_List.ObjectList_Objects[i].u_Position_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_X_STD);
        object_List.ObjectList_Objects[i].u_Position_Y=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Y);
        object_List.ObjectList_Objects[i].u_Position_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Y_STD);
        object_List.ObjectList_Objects[i].u_Position_Z=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Z);
        object_List.ObjectList_Objects[i].u_Position_Z_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Z_STD);
        object_List.ObjectList_Objects[i].u_Position_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_CovarianceXY);
        object_List.ObjectList_Objects[i].u_Position_Orientation=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Orientation);
        object_List.ObjectList_Objects[i].u_Position_Orientation_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Orientation_STD);
        object_List.ObjectList_Objects[i].u_Existence_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Existence_InvalidFlags);
        object_List.ObjectList_Objects[i].u_Existence_Probability=ChangeEndianness(object_List.ObjectList_Objects[i].u_Existence_Probability);
        object_List.ObjectList_Objects[i].u_Existence_PPV=ChangeEndianness(object_List.ObjectList_Objects[i].u_Existence_PPV);
        object_List.ObjectList_Objects[i].u_Classification_Car=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Car);
        object_List.ObjectList_Objects[i].u_Classification_Truck=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Truck);
        object_List.ObjectList_Objects[i].u_Classification_Motorcycle=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Motorcycle);
        object_List.ObjectList_Objects[i].u_Classification_Bicycle=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Bicycle);
        object_List.ObjectList_Objects[i].u_Classification_Pedestrian=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Pedestrian);
        object_List.ObjectList_Objects[i].u_Classification_Animal=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Animal);
        object_List.ObjectList_Objects[i].u_Classification_Hazard=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Hazard);
        object_List.ObjectList_Objects[i].u_Classification_Unknown=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Unknown);
        object_List.ObjectList_Objects[i].u_Classification_Overdrivable=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Overdrivable);
        object_List.ObjectList_Objects[i].u_Classification_Underdrivable=ChangeEndianness(object_List.ObjectList_Objects[i].u_Classification_Underdrivable);
        object_List.ObjectList_Objects[i].u_Dynamics_AbsVel_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_AbsVel_InvalidFlags);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY);
        object_List.ObjectList_Objects[i].u_Dynamics_RelVel_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_RelVel_InvalidFlags);
        object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X);
        object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y);
        object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY);
        object_List.ObjectList_Objects[i].u_Dynamics_AbsAccel_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_AbsAccel_InvalidFlags);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY);
        object_List.ObjectList_Objects[i].u_Dynamics_RelAccel_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_RelAccel_InvalidFlags);
        object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X);
        object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y);
        object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD);
        object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY);
        object_List.ObjectList_Objects[i].u_Dynamics_Orientation_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_Orientation_InvalidFlags);
        object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean);
        object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD);
        object_List.ObjectList_Objects[i].u_Shape_Length_Status=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Length_Status);
        object_List.ObjectList_Objects[i].u_Shape_Length_Edge_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Length_Edge_InvalidFlags);
        object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean);
        object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD);
        object_List.ObjectList_Objects[i].u_Shape_Width_Status=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Width_Status);
        object_List.ObjectList_Objects[i].u_Shape_Width_Edge_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Width_Edge_InvalidFlags);
        object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean);
        object_List.ObjectList_Objects[i].u_Shape_Width_Edge_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Width_Edge_STD);
    }
    return object_List;
}

DetectionList ars548_driver_class::modifyDetectionList(DetectionList detectionList){
    detectionList.CRC=ChangeEndianness(detectionList.CRC);
    detectionList.Length=ChangeEndianness(detectionList.Length);
    detectionList.SQC=ChangeEndianness(detectionList.SQC);
    detectionList.DataID=ChangeEndianness(detectionList.DataID);
    detectionList.Timestamp_Nanoseconds=ChangeEndianness(detectionList.Timestamp_Nanoseconds);
    detectionList.Timestamp_Seconds=ChangeEndianness(detectionList.Timestamp_Seconds);
    detectionList.Timestamp_SyncStatus=ChangeEndianness(detectionList.Timestamp_SyncStatus);
    detectionList.EventDataQualifier=ChangeEndianness(detectionList.EventDataQualifier);
    detectionList.ExtendedQualifier=ChangeEndianness(detectionList.ExtendedQualifier);
    detectionList.Origin_InvalidFlags=ChangeEndianness(detectionList.Origin_InvalidFlags);
    detectionList.Origin_Xpos=ChangeEndianness(detectionList.Origin_Xpos);
    detectionList.Origin_Xstd=ChangeEndianness(detectionList.Origin_Xstd);
    detectionList.Origin_Ypos=ChangeEndianness(detectionList.Origin_Ypos);
    detectionList.Origin_Ystd=ChangeEndianness(detectionList.Origin_Ystd);
    detectionList.Origin_Zpos=ChangeEndianness(detectionList.Origin_Zpos);
    detectionList.Origin_Zstd=ChangeEndianness(detectionList.Origin_Zstd);
    detectionList.Origin_Roll=ChangeEndianness(detectionList.Origin_Roll);
    detectionList.Origin_Rollstd=ChangeEndianness(detectionList.Origin_Rollstd);
    detectionList.Origin_Pitch=ChangeEndianness(detectionList.Origin_Pitch);
    detectionList.Origin_Pitchstd=ChangeEndianness(detectionList.Origin_Pitchstd);
    detectionList.Origin_Yaw=ChangeEndianness(detectionList.Origin_Yaw);
    detectionList.Origin_Yawstd=ChangeEndianness(detectionList.Origin_Yawstd);
    detectionList.List_InvalidFlags=ChangeEndianness(detectionList.List_InvalidFlags);
    detectionList.List_NumOfDetections=ChangeEndianness(detectionList.List_NumOfDetections);
    detectionList.List_RadVelDomain_Min=ChangeEndianness(detectionList.List_RadVelDomain_Min);
    detectionList.List_RadVelDomain_Max=ChangeEndianness(detectionList.List_RadVelDomain_Max);
    detectionList.Aln_AzimuthCorrection=ChangeEndianness(detectionList.Aln_AzimuthCorrection);
    detectionList.Aln_ElevationCorrection=ChangeEndianness(detectionList.Aln_ElevationCorrection);
    detectionList.Aln_Status=ChangeEndianness(detectionList.Aln_Status);
    if (detectionList.List_NumOfDetections>MAX_DETECTIONS){
        detectionList.List_NumOfDetections=MAX_DETECTIONS;
    }
    for(uint64_t i=0; i<detectionList.List_NumOfDetections;i++){
        //Setting the detection data to littleEndian
        detectionList.List_Detections[i].f_AzimuthAngle=ChangeEndianness(detectionList.List_Detections[i].f_AzimuthAngle);
        detectionList.List_Detections[i].f_AzimuthAngleSTD=ChangeEndianness(detectionList.List_Detections[i].f_AzimuthAngleSTD);
        detectionList.List_Detections[i].u_InvalidFlags=ChangeEndianness(detectionList.List_Detections[i].u_InvalidFlags);
        detectionList.List_Detections[i].f_ElevationAngle=ChangeEndianness(detectionList.List_Detections[i].f_ElevationAngle);
        detectionList.List_Detections[i].f_ElevationAngleSTD=ChangeEndianness(detectionList.List_Detections[i].f_ElevationAngleSTD);
        detectionList.List_Detections[i].f_Range=ChangeEndianness(detectionList.List_Detections[i].f_Range);
        detectionList.List_Detections[i].f_RangeSTD=ChangeEndianness(detectionList.List_Detections[i].f_RangeSTD);
        detectionList.List_Detections[i].f_RangeRate=ChangeEndianness(detectionList.List_Detections[i].f_RangeRate);
        detectionList.List_Detections[i].f_RangeRateSTD=ChangeEndianness(detectionList.List_Detections[i].f_RangeRateSTD);
        detectionList.List_Detections[i].s_RCS=ChangeEndianness(detectionList.List_Detections[i].s_RCS);
        detectionList.List_Detections[i].u_MeasurementID=ChangeEndianness(detectionList.List_Detections[i].u_MeasurementID);
        detectionList.List_Detections[i].u_PositivePredictiveValue=ChangeEndianness(detectionList.List_Detections[i].u_PositivePredictiveValue);
        detectionList.List_Detections[i].u_Classification=ChangeEndianness(detectionList.List_Detections[i].u_Classification);
        detectionList.List_Detections[i].u_MultiTargetProbability=ChangeEndianness(detectionList.List_Detections[i].u_MultiTargetProbability);
        detectionList.List_Detections[i].u_ObjectID=ChangeEndianness(detectionList.List_Detections[i].u_ObjectID);
        detectionList.List_Detections[i].u_AmbiguityFlag=ChangeEndianness(detectionList.List_Detections[i].u_AmbiguityFlag);
        detectionList.List_Detections[i].u_SortIndex=ChangeEndianness(detectionList.List_Detections[i].u_SortIndex);
    }
    return detectionList;
}

void ars548_driver_class::fillStatusMessage(ars548_ros::Status &statusMessage, UDPStatus status){
    statusMessage.cycletime=status.CycleTime;
    statusMessage.configurationcounter=status.ConfigurationCounter;
    statusMessage.frequencyslot=status.FrequencySlot;
    statusMessage.hcc=status.HCC;
    statusMessage.height=status.Height;
    statusMessage.lateral=status.Lateral;
    statusMessage.length=status.Length;
    statusMessage.longitudinal=status.Longitudinal;
    statusMessage.maximundistance=status.MaximunDistance;
    statusMessage.pitch=status.Pitch;
    statusMessage.plugorientation=status.PlugOrientation;
    statusMessage.powersave_standstill=status.Powersave_Standstill;
    statusMessage.sensoripaddress_0=status.SensorIPAddress_0;
    statusMessage.sensoripaddress_1=status.SensorIPAddress_1;
    statusMessage.status_blockagestatus=status.Status_BlockageStatus;
    statusMessage.status_characteristicspeed=status.Status_CharacteristicSpeed;
    statusMessage.status_drivingdirection=status.Status_DrivingDirection;
    statusMessage.status_lateralacceleration=status.Status_LateralAcceleration;
    statusMessage.status_longitudinalacceleration=status.Status_LongitudinalAcceleration;
    statusMessage.status_longitudinalvelocity=status.Status_LongitudinalVelocity;
    statusMessage.status_radarstatus=status.Status_RadarStatus;
    statusMessage.status_steeringangle=status.Status_SteeringAngle;
    statusMessage.status_temperaturestatus=status.Status_TemperatureStatus;
    statusMessage.status_voltagestatus=status.Status_VoltageStatus;
    statusMessage.status_yawrate=status.Status_YawRate;
    statusMessage.swversion_major=status.SWVersion_Major;
    statusMessage.swversion_minor=status.SWVersion_Minor;
    statusMessage.swversion_patch=status.SWVersion_Patch;
    statusMessage.timeslot=status.TimeSlot;
    statusMessage.timestamp_nanoseconds=status.Timestamp_Nanoseconds;
    statusMessage.timestamp_seconds=status.Timestamp_Seconds;
    statusMessage.timestamp_syncstatus=status.Timestamp_SyncStatus;
    statusMessage.vertical=status.Vertical;
    statusMessage.wheelbase=status.Wheelbase;
    statusMessage.width=status.Width;
    statusMessage.yaw=status.Yaw;
}

void ars548_driver_class::fillObjectMessage(ars548_ros::ObjectList4D &objectMessage,Object_List object_List){
    objectMessage.crc=object_List.CRC;
    objectMessage.length=object_List.Length;
    objectMessage.sqc=object_List.SQC;
    objectMessage.timestamp_nanoseconds=object_List.Timestamp_Nanoseconds;
    objectMessage.timestamp_seconds=object_List.Timestamp_Seconds;
    objectMessage.eventdataqualifier=object_List.EventDataQualifier;
    objectMessage.extendedqualifier=object_List.ExtendedQualifier;
    objectMessage.dataid=object_List.DataID;
    objectMessage.objectlist_numofobjects=object_List.ObjectList_NumOfObjects;
    objectMessage.timestamp_syncstatus=object_List.Timestamp_SyncStatus;
    
    objectMessage.header.frame_id=this->frame_ID;
    objectMessage.header.stamp=ros::Time::now();
    for(u_int32_t i =0; i<object_List.ObjectList_NumOfObjects;++i){
        objectMessage.objectlist_objects[i].u_statussensor=object_List.ObjectList_Objects[i].u_StatusSensor;
        objectMessage.objectlist_objects[i].u_id=object_List.ObjectList_Objects[i].u_ID;
        objectMessage.objectlist_objects[i].u_age=object_List.ObjectList_Objects[i].u_Age;
        objectMessage.objectlist_objects[i].u_position_invalidflags=object_List.ObjectList_Objects[i].u_Position_InvalidFlags;
        objectMessage.objectlist_objects[i].u_position_x=object_List.ObjectList_Objects[i].u_Position_X;
        objectMessage.objectlist_objects[i].u_position_x_std=object_List.ObjectList_Objects[i].u_Position_X_STD;
        objectMessage.objectlist_objects[i].u_position_y=object_List.ObjectList_Objects[i].u_Position_Y;
        objectMessage.objectlist_objects[i].u_position_y_std=object_List.ObjectList_Objects[i].u_Position_Y_STD;
        objectMessage.objectlist_objects[i].u_position_z=object_List.ObjectList_Objects[i].u_Position_Z;
        objectMessage.objectlist_objects[i].u_position_z_std=object_List.ObjectList_Objects[i].u_Position_Z_STD;
        objectMessage.objectlist_objects[i].u_position_covariancexy=object_List.ObjectList_Objects[i].u_Position_CovarianceXY;
        objectMessage.objectlist_objects[i].u_position_orientation=object_List.ObjectList_Objects[i].u_Position_Orientation;
        objectMessage.objectlist_objects[i].u_position_orientation_std=object_List.ObjectList_Objects[i].u_Position_Orientation_STD;
        objectMessage.objectlist_objects[i].u_position_reference=object_List.ObjectList_Objects[i].u_Position_Reference;
        objectMessage.objectlist_objects[i].u_existence_invalidflags=object_List.ObjectList_Objects[i].u_Existence_InvalidFlags;
        objectMessage.objectlist_objects[i].u_existence_ppv=object_List.ObjectList_Objects[i].u_Existence_PPV;
        objectMessage.objectlist_objects[i].u_existence_probability=object_List.ObjectList_Objects[i].u_Existence_Probability;
        objectMessage.objectlist_objects[i].u_dynamics_absaccel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_AbsAccel_InvalidFlags;
        objectMessage.objectlist_objects[i].u_dynamics_absvel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_AbsVel_InvalidFlags;
        objectMessage.objectlist_objects[i].u_dynamics_orientation_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_Orientation_InvalidFlags;
        objectMessage.objectlist_objects[i].u_dynamics_orientation_rate_mean=object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean;
        objectMessage.objectlist_objects[i].u_dynamics_orientation_rate_std=object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD;
        objectMessage.objectlist_objects[i].u_dynamics_relaccel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_RelAccel_InvalidFlags;
        objectMessage.objectlist_objects[i].u_dynamics_relvel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_RelVel_InvalidFlags;
        objectMessage.objectlist_objects[i].u_shape_length_edge_invalidflags=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_InvalidFlags;
        objectMessage.objectlist_objects[i].u_shape_length_edge_mean=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean;
        objectMessage.objectlist_objects[i].u_shape_length_edge_std=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD;
        objectMessage.objectlist_objects[i].u_shape_length_status=object_List.ObjectList_Objects[i].u_Shape_Length_Status;
        objectMessage.objectlist_objects[i].u_shape_width_edge_invalidflags=object_List.ObjectList_Objects[i].u_Shape_Width_Edge_InvalidFlags;
        objectMessage.objectlist_objects[i].u_shape_width_edge_mean=object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean;
        objectMessage.objectlist_objects[i].u_shape_width_edge_std=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD;
        objectMessage.objectlist_objects[i].u_shape_width_status=object_List.ObjectList_Objects[i].u_Shape_Width_Status;
        objectMessage.objectlist_objects[i].u_statusmeasurement=object_List.ObjectList_Objects[i].u_StatusMeasurement;
        objectMessage.objectlist_objects[i].u_statusmovement=object_List.ObjectList_Objects[i].u_StatusMovement;
        objectMessage.objectlist_objects[i].u_statussensor=object_List.ObjectList_Objects[i].u_StatusSensor;
        objectMessage.objectlist_objects[i].u_classification_animal=object_List.ObjectList_Objects[i].u_Classification_Animal;
        objectMessage.objectlist_objects[i].u_classification_bicycle=object_List.ObjectList_Objects[i].u_Classification_Bicycle;
        objectMessage.objectlist_objects[i].u_classification_car=object_List.ObjectList_Objects[i].u_Classification_Car;
        objectMessage.objectlist_objects[i].u_classification_hazard=object_List.ObjectList_Objects[i].u_Classification_Hazard;
        objectMessage.objectlist_objects[i].u_classification_motorcycle=object_List.ObjectList_Objects[i].u_Classification_Motorcycle;
        objectMessage.objectlist_objects[i].u_classification_overdrivable=object_List.ObjectList_Objects[i].u_Classification_Overdrivable;
        objectMessage.objectlist_objects[i].u_classification_pedestrian=object_List.ObjectList_Objects[i].u_Classification_Pedestrian;
        objectMessage.objectlist_objects[i].u_classification_truck=object_List.ObjectList_Objects[i].u_Classification_Truck;
        objectMessage.objectlist_objects[i].u_classification_underdrivable=object_List.ObjectList_Objects[i].u_Classification_Underdrivable;
        objectMessage.objectlist_objects[i].u_classification_unknown=object_List.ObjectList_Objects[i].u_Position_CovarianceXY;
        objectMessage.objectlist_objects[i].f_dynamics_absaccel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY;
        objectMessage.objectlist_objects[i].f_dynamics_absaccel_x=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X;
        objectMessage.objectlist_objects[i].f_dynamics_absaccel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD;
        objectMessage.objectlist_objects[i].f_dynamics_absaccel_y=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y;
        objectMessage.objectlist_objects[i].f_dynamics_absaccel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD;
        objectMessage.objectlist_objects[i].f_dynamics_absvel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY;
        objectMessage.objectlist_objects[i].f_dynamics_absvel_x=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X;
        objectMessage.objectlist_objects[i].f_dynamics_absvel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD;
        objectMessage.objectlist_objects[i].f_dynamics_absvel_y=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y;
        objectMessage.objectlist_objects[i].f_dynamics_absvel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD;
        objectMessage.objectlist_objects[i].f_dynamics_relaccel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY;
        objectMessage.objectlist_objects[i].f_dynamics_relaccel_x=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X;
        objectMessage.objectlist_objects[i].f_dynamics_relaccel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X_STD;
        objectMessage.objectlist_objects[i].f_dynamics_relaccel_y=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y;
        objectMessage.objectlist_objects[i].f_dynamics_relaccel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD;
        objectMessage.objectlist_objects[i].f_dynamics_relvel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY;
        objectMessage.objectlist_objects[i].f_dynamics_relvel_x=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X;
        objectMessage.objectlist_objects[i].f_dynamics_relvel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD;
        objectMessage.objectlist_objects[i].f_dynamics_relvel_y=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y;
        objectMessage.objectlist_objects[i].f_dynamics_relvel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD;  
    }
    
}

void ars548_driver_class::fillDetectionMessage(ars548_ros::DetectionList &detectionMessage,DetectionList detectionList){
    detectionMessage.header.frame_id=this->frame_ID;
    detectionMessage.header.stamp=ros::Time::now();
    detectionMessage.aln_status=detectionList.Aln_Status;
    detectionMessage.crc=detectionList.CRC;
    detectionMessage.dataid=detectionList.DataID;
    detectionMessage.eventdataqualifier=detectionList.EventDataQualifier;
    detectionMessage.extendedqualifier=detectionList.ExtendedQualifier;
    detectionMessage.length=detectionList.Length;
    detectionMessage.origin_invalidflags=detectionList.Origin_InvalidFlags;
    detectionMessage.origin_pitch=detectionList.Origin_Pitch;
    detectionMessage.origin_pitchstd=detectionList.Origin_Pitchstd;
    detectionMessage.origin_roll=detectionList.Origin_Roll;
    detectionMessage.origin_rollstd=detectionList.Origin_Rollstd;
    detectionMessage.origin_xpos=detectionList.Origin_Xpos;
    detectionMessage.origin_xstd=detectionList.Origin_Xstd;
    detectionMessage.origin_yaw=detectionList.Origin_Yaw;
    detectionMessage.origin_yawstd=detectionList.Origin_Yawstd;
    detectionMessage.origin_ypos=detectionList.Origin_Ypos;
    detectionMessage.origin_ystd=detectionList.Origin_Ystd;
    detectionMessage.origin_zpos=detectionList.Origin_Zpos;
    detectionMessage.origin_zstd=detectionList.Origin_Zstd;
    detectionMessage.sqc=detectionList.SQC;
    detectionMessage.timestamp_nanoseconds=detectionList.Timestamp_Nanoseconds;
    detectionMessage.timestamp_seconds=detectionList.Timestamp_Seconds;
    detectionMessage.timestamp_syncstatus=detectionList.Timestamp_SyncStatus;
    for(uint64_t i=0; i<detectionList.List_NumOfDetections;i++){            
        detectionMessage.list_detections[i].f_azimuthangle=detectionList.List_Detections[i].f_AzimuthAngle;
        detectionMessage.list_detections[i].f_azimuthanglestd=detectionList.List_Detections[i].f_AzimuthAngleSTD;
        detectionMessage.list_detections[i].f_elevationangle=detectionList.List_Detections[i].f_ElevationAngle;
        detectionMessage.list_detections[i].f_elevationanglestd=detectionList.List_Detections[i].f_ElevationAngleSTD;
        detectionMessage.list_detections[i].f_range=detectionList.List_Detections[i].f_Range;
        detectionMessage.list_detections[i].f_rangerate=detectionList.List_Detections[i].f_RangeRate;
        detectionMessage.list_detections[i].f_rangeratestd=detectionList.List_Detections[i].f_RangeRateSTD;
        detectionMessage.list_detections[i].f_rangestd=detectionList.List_Detections[i].f_RangeSTD;
        detectionMessage.list_detections[i].s_rcs=detectionList.List_Detections[i].s_RCS;
        detectionMessage.list_detections[i].u_ambiguityflag=detectionList.List_Detections[i].u_AmbiguityFlag;
        detectionMessage.list_detections[i].u_classification=detectionList.List_Detections[i].u_Classification;
        detectionMessage.list_detections[i].u_invalidflags=detectionList.List_Detections[i].u_InvalidFlags;
        detectionMessage.list_detections[i].u_measurementid=detectionList.List_Detections[i].u_MeasurementID;
        detectionMessage.list_detections[i].u_multitargetprobability=detectionList.List_Detections[i].u_MultiTargetProbability;
        detectionMessage.list_detections[i].u_objectid=detectionList.List_Detections[i].u_ObjectID;
        detectionMessage.list_detections[i].u_positivepredictivevalue=detectionList.List_Detections[i].u_PositivePredictiveValue;
        detectionMessage.list_detections[i].u_sortindex=detectionList.List_Detections[i].u_SortIndex;   
    }
    detectionMessage.list_invalidflags=detectionList.List_InvalidFlags;
    detectionMessage.list_numofdetections=detectionList.List_NumOfDetections;
    detectionMessage.list_radveldomain_max=detectionList.List_RadVelDomain_Max;
    detectionMessage.list_radveldomain_min=detectionList.List_RadVelDomain_Min;
    detectionMessage.aln_azimuthcorrection=detectionList.Aln_AzimuthCorrection;
    detectionMessage.aln_elevationcorrection=detectionList.Aln_ElevationCorrection;
}

void ars548_driver_class::fillCloudMessage(sensor_msgs::PointCloud2 &cloud_msg, uint32_t num){
    cloud_msg.header=std_msgs::Header();
    cloud_msg.header.frame_id=this->frame_ID;
    cloud_msg.header.stamp=ros::Time::now();
    cloud_msg.is_dense=false;
    cloud_msg.is_bigendian=false;
    cloud_msg.height=POINTCLOUD_HEIGHT;
    cloud_msg.width=num;
    cloud_msg.point_step = sizeof(float) * 7 + sizeof(int8_t);
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.width * cloud_msg.height * cloud_msg.point_step);
}

void ars548_driver_class::fillDirectionMessage(geometry_msgs::PoseArray &cloud_Direction,Object_List object_List,u_int32_t i){
    tf2::Quaternion q;
    float yaw;
    cloud_Direction.header = std_msgs::Header();
    cloud_Direction.header.frame_id=this->frame_ID;
    cloud_Direction.header.stamp = ros::Time::now();
    cloud_Direction.poses[i].position.x = double(object_List.ObjectList_Objects[i].u_Position_X);
    cloud_Direction.poses[i].position.y = double(object_List.ObjectList_Objects[i].u_Position_Y);
    cloud_Direction.poses[i].position.z = double(object_List.ObjectList_Objects[i].u_Position_Z);
    yaw = atan2(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y,object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X);   
    q.setRPY(0,0,yaw);
    cloud_Direction.poses[i].orientation.x=q.x();
    cloud_Direction.poses[i].orientation.y=q.y();
    cloud_Direction.poses[i].orientation.z=q.z();
    cloud_Direction.poses[i].orientation.w=q.w();
}

void ars548_driver_class::fillMarkerMessage(visualization_msgs::Marker &marker_object, Object_List object_List,u_int32_t i){
    marker_object.header = std_msgs::Header();
    marker_object.header.frame_id=this->frame_ID;
    marker_object.header.stamp = ros::Time::now();
    marker_object.pose.position.x = object_List.ObjectList_Objects[i].u_Position_X;
    marker_object.pose.position.y = object_List.ObjectList_Objects[i].u_Position_Y;
    marker_object.pose.position.z = object_List.ObjectList_Objects[i].u_Position_Z;
    tf2::Quaternion q;
    q.setRPY(0,0,object_List.ObjectList_Objects[i].u_Position_Orientation);
    marker_object.pose.orientation.w = q.w();
    marker_object.pose.orientation.x = q.x();
    marker_object.pose.orientation.y = q.y();
    marker_object.pose.orientation.z = q.z();
    marker_object.type = visualization_msgs::Marker::LINE_STRIP;
    marker_object.id = object_List.ObjectList_Objects[i].u_ID;
    geometry_msgs::Point pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
    pos1.x = object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos1.y = object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos1.z = ((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    pos2.x = object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos2.y = object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos2.z = -((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    pos3.x = object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos3.y = -object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos3.z = -((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    pos4.x = object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos4.y = -object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos4.z = ((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    pos5.x = -object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos5.y = -object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos5.z = ((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    pos6.x = -object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos6.y = -object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos6.z = -((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    pos7.x = -object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos7.y = object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos7.z = -((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    pos8.x = -object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean / 2;
    pos8.y = object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean / 2;
    pos8.z = ((object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean + object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean) / 2) / 2;

    marker_object.points.push_back(pos1);
    marker_object.points.push_back(pos2);
    marker_object.points.push_back(pos3);
    marker_object.points.push_back(pos4);
    marker_object.points.push_back(pos5);
    marker_object.points.push_back(pos6);
    marker_object.points.push_back(pos7);
    marker_object.points.push_back(pos8);
    marker_object.points.push_back(pos1);
    marker_object.points.push_back(pos4);
    marker_object.points.push_back(pos3);
    marker_object.points.push_back(pos6);
    marker_object.points.push_back(pos5);
    marker_object.points.push_back(pos8);
    marker_object.points.push_back(pos7);
    marker_object.points.push_back(pos2);

    uint8_t Car = object_List.ObjectList_Objects[i].u_Classification_Car;
    uint8_t Truck = object_List.ObjectList_Objects[i].u_Classification_Truck;
    uint8_t Motorcycle = object_List.ObjectList_Objects[i].u_Classification_Motorcycle;
    uint8_t Bicycle = object_List.ObjectList_Objects[i].u_Classification_Bicycle;
    uint8_t Pedestrian = object_List.ObjectList_Objects[i].u_Classification_Pedestrian;
    uint8_t Animal = object_List.ObjectList_Objects[i].u_Classification_Animal;
    uint8_t Hazard = object_List.ObjectList_Objects[i].u_Classification_Hazard;
    uint8_t Unknown = object_List.ObjectList_Objects[i].u_Classification_Unknown;
    uint8_t Overdrivable = object_List.ObjectList_Objects[i].u_Classification_Overdrivable;
    uint8_t Underdrivable = object_List.ObjectList_Objects[i].u_Classification_Underdrivable;

    uint8_t values[] = {Car, Truck, Motorcycle, Bicycle, Pedestrian, Animal, Hazard, Unknown, Overdrivable, Underdrivable};
    const char* names[] = {"Car", "Truck", "Motorcycle", "Bicycle", "Pedestrian", "Animal", "Hazard", "Unknown", "Overdrivable", "Underdrivable"};
    uint8_t max_val = values[0];
    int max_index = 0;

    // 找出最大值对应的索引
    for (int i = 1; i < 10; ++i) {
        if (values[i] > max_val) {
            max_val = values[i];
            max_index = i;
        }
    }
    if(names[max_index] == "Car") {
        marker_object.color.r = 1.0;
        marker_object.color.g = 0.0;
        marker_object.color.b = 0.0; 
    }
    else if(names[max_index] == "Truck") {
        marker_object.color.r = 0.0;
        marker_object.color.g = 1.0;
        marker_object.color.b = 0.0; 
    }
    else if(names[max_index] == "Motorcycle") {
        marker_object.color.r = 0.0;
        marker_object.color.g = 0.0;
        marker_object.color.b = 1.0; 
    }
    else if(names[max_index] == "Bicycle") {
        marker_object.color.r = 1.0;
        marker_object.color.g = 1.0;
        marker_object.color.b = 0.0; 
    }
    else if(names[max_index] == "Pedestrian") {
        marker_object.color.r = 1.0;
        marker_object.color.g = 0.0;
        marker_object.color.b = 1.0; 
    }
    else if(names[max_index] == "Animal") {
        marker_object.color.r = 0.0;
        marker_object.color.g = 1.0;
        marker_object.color.b = 1.0; 
    }
    else if(names[max_index] == "Hazard") {
        marker_object.color.r = 0.5;
        marker_object.color.g = 1.0;
        marker_object.color.b = 1.0;
    }
    else if(names[max_index] == "Unknown") {
        marker_object.color.r = 1.0;
        marker_object.color.g = 0.5;
        marker_object.color.b = 1.0;
    }
    else if(names[max_index] == "Overdrivable") {
        marker_object.color.r = 1.0;
        marker_object.color.g = 1.0;
        marker_object.color.b = 0.5;
    }
    else if(names[max_index] == "Underdrivable") {
        marker_object.color.r = 0.5;
        marker_object.color.g = 0.5;
        marker_object.color.b = 0.5;
    }
    else {
        marker_object.color.r = 1.0;
        marker_object.color.g = 1.0;
        marker_object.color.b = 1.0;
    }
    marker_object.scale.x = 0.1;
    marker_object.color.a = 1.0;
    marker_object.lifetime.fromSec(0.1);
}