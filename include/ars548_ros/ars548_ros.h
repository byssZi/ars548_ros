/** @file ars548_driver.hpp
 * 
 * @brief ars548_driver是一个类，用于从传感器获取所有数据，翻译并发送给用户以供后续使用。
 * 它还会复制接收到的部分数据，并将其发送到Rviz以便可视化结果。
 */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "ars548_ros/Status.h"
#include "ars548_ros/DetectionList.h"
#include "ars548_ros/ObjectList4D.h"
#include "ars548_data.h"

/**
 * @brief 从RadarSensors_Annex_AES548_IO SW 05.48.04.pdf中获得的数据 
 */
#define DEFAULT_RADAR_IP "224.0.2.2"
#define RADAR_INTERFACE "10.13.1.166"
#define DEFAULT_RADAR_PORT 42102
#define DEFAULT_FRAME_ID "ARS_548" 
#define MSGBUFSIZE 102400
#define MAX_OBJECTS 50
#define MAX_DETECTIONS 800
#define STATUS_MESSAGE_METHOD_ID 380
#define OBJECT_MESSAGE_METHOD_ID 329
#define DETECTION_MESSAGE_METHOD_ID 336
#define STATUS_MESSAGE_PDU_LENGTH 76
#define OBJECT_MESSAGE_PDU_LENGTH 9393
#define DETECTION_MESSAGE_PDU_LENGTH 35328
#define STATUS_MESSAGE_PAYLOAD 84
#define OBJECT_MESSAGE_PAYLOAD 9401
#define DETECTION_MESSAGE_PAYLOAD 35336
/**
 * @brief POINTCLOUD_HEIGHT = 1，因为点云是无序的。
 */
#define POINTCLOUD_HEIGHT 1
/**
 * @brief 这些字段可以更改
 */
#define SIZE 1000

class ars548_driver_class {
private:
    int methodID;
    char msgbuf[MSGBUFSIZE];
    int fd;
    int nbytes;
    float AbsVel;
    std::string answer;
    ros::NodeHandle nh_;
    ros::Publisher statusPublisher;
    ros::Publisher objectPublisher;
    ros::Publisher detectionsPublisher;
    ros::Publisher directionPublisher;
    ros::Publisher pubObj;
    ros::Publisher pubDetect;
    sensor_msgs::PointCloud2 cloud_msgDetect;
    geometry_msgs::PoseArray cloud_Direction;

    sensor_msgs::PointCloud2Modifier modifierDetection;


    /**
     * @brief  Sends the data on socket fd to the address addr.
     * 
     * @tparam data The buffer with all of the data.
     * @param fd The socket where you are going to send the data.
     * @param addr The address where the data is going to be sent.
     * @return nbytes. The number of bytes sent. If its value is -1 there has been an error.
     */

    template<typename T>
    int SendMessage(int fd, T& data, sockaddr_in addr){
        int nbytes=sendto(fd,&data,8+sizeof(data),0,(struct sockaddr *) &addr,sizeof(addr));
        return nbytes;
    }
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

    /**
     * @brief Changes the Endiannes of the status struct. 
     * 
     * @param status The UDPStatus struct that is going to be modified.
     * @return UDPStatus The modified struct. 
     */
    UDPStatus modifyStatus(UDPStatus status);
    /**
     * @brief Changes the endiannes of the Object_List struct
     *  
     * @param object_List The Object_List struct that is going to be modified.
     * @return Object_List The modified Struct.
     */
    Object_List modifyObjectList(Object_List object_List);
    /**
     * @brief Changes the endianness of the DetectionList struct.
     * 
     * @param detectionList The DetectionList struct that is going to be modified.
     * @return DetectionList The modified struct.
     */
    DetectionList modifyDetectionList(DetectionList detectionList);
    /**
     * @brief Fills the Status Messsage.
     * 
     * @param statusMessage The Status message to be filled.
     * @param status The Status struct used to fill the message.
     * 
     */
    void fillStatusMessage(ars548_ros::Status &statusMessage, UDPStatus status);
    /**
     * @brief Fills the ObjectList message.
     * 
     * @param objectMessage The object message to be filled.
     * @param object_List The Object_List struct used to fill the message.
     * @param clock The clock used to fill the timestamp of the message.
     * 
     */
    void fillObjectMessage(ars548_ros::ObjectList4D &objectMessage,Object_List object_List);
    /**
     * @brief Fills the DetectionList message.
     * 
     * @param detectionMessage The DetectionList message to be filled.
     * @param detectionList The DetectionList struct used to fill the message.
     * @param clock The clock used to fill the timestamp of the message.
     */
    void fillDetectionMessage(ars548_ros::DetectionList &detectionMessage,DetectionList detectionList);
    /**
     * @brief Fills the PointCloud2 message. Used for visualization in Rviz.
     * 
     * @param cloud_msg The PointCloud2 message that is going to be filled.
     */
    void fillCloudMessage(sensor_msgs::PointCloud2 &cloud_msg, uint32_t num);
    /**
     * @brief Fills the PoseArray message. Used for visualization in Rviz.
     *
     * @param cloud_Direction The PoseArray message to be filled.
     * @param object_List The Object_List struct used to fill the message.
     * @param i The iterator used to fill the array of poses with the values of the points obtained from the struct.
     * @return PoseArray.msg. The message filled. 
     */
    void fillDirectionMessage(geometry_msgs::PoseArray &cloud_Direction,Object_List object_List,u_int32_t i);
    /**
     * @brief Fills the Marker message. Used for visualization in Rviz.
     *
     * @param cloud_Direction The Marker message to be filled.
     * @param object_List The Object_List struct used to fill the message.
     * @param i The iterator used to fill the array of poses with the values of the points obtained from the struct.
     * @return Marker.msg. The message filled. 
     */
    void fillMarkerMessage(visualization_msgs::Marker &marker_object, Object_List object_List,u_int32_t i);

public:
    std::string ars548_IP;
    int ars548_Port;
    std::string frame_ID;

    ars548_driver_class();

    ~ars548_driver_class();

    void run();

    void start_read();
};
