#ifndef _USRTOPIC_H
#define _USRTOPIC_H

#include <ros/ros.h>
#include <string>

namespace UsrLib{
    //assister topic

    //master topic: master order
    const std::string TOPIC_MASTER_MASTERORDER              ="msg_master_masterorder";

    //driver topic: gnss
    const std::string TOPIC_DRIVER_GNSS                     ="msg_driver_gnss";
    //driver topic: imu
    const std::string TOPIC_DRIVER_IMU                      ="msg_driver_imu";
    //driver topic: camera 1-6
    const std::string TOPIC_DRIVER_CAMERA1                  ="msg_driver_camera_1";
    const std::string TOPIC_DRIVER_CAMERA2                  ="msg_driver_camera_2";
    const std::string TOPIC_DRIVER_CAMERA3                  ="msg_driver_camera_3";
    const std::string TOPIC_DRIVER_CAMERA4                  ="msg_driver_camera_4";
    const std::string TOPIC_DRIVER_CAMERA5                  ="msg_driver_camera_5";
    const std::string TOPIC_DRIVER_CAMERA6                  ="msg_driver_camera_6";
    //driver topic: lidar 1-6
    const std::string TOPIC_DRIVER_LIDAR1                   ="msg_driver_lidar_1";
    const std::string TOPIC_DRIVER_LIDAR2                   ="msg_driver_lidar_2";
    const std::string TOPIC_DRIVER_LIDAR3                   ="msg_driver_lidar_3";
    const std::string TOPIC_DRIVER_LIDAR4                   ="msg_driver_lidar_4";
    const std::string TOPIC_DRIVER_LIDAR5                   ="msg_driver_lidar_5";
    const std::string TOPIC_DRIVER_LIDAR6                   ="msg_driver_lidar_6";

    //perception topic: motion information of the ego car
    const std::string TOPIC_PERCEPTION_MOTIONINFO           ="msg_perception_selfpose";
    //perception topic: actuator information of the ego car
    const std::string TOPIC_PERCEPTION_CARINFO              ="msg_perception_carstate";
    //perception topic: detection/tracking/prediction results, static objects
    const std::string TOPIC_PERCEPTION_STATICOBJ            ="msg_perception_staticobjects";
    //perception topic: detection/tracking/prediction results, dynamic objects
    const std::string TOPIC_PERCEPTION_DYNAMICOBJ           ="msg_perception_dynamicobjects";
    //perception topic: detection results, lane lines
    const std::string TOPIC_PERCEPTION_LANELINE             ="msg_perception_lanelines";
    //perception topic: detection results, traffic lights
    const std::string TOPIC_PERCEPTION_TRAFFICLIGHT         ="msg_perception_trafficlights";

    //planning topic: decision behavior 
    const std::string TOPIC_PLAN_BEHAVIOR                   ="msg_plan_behavior";
    //planning topic: planned global route
    const std::string TOPIC_PLAN_GLOBALROUTE                ="msg_plan_global";
    //planning topic: planned local route
    const std::string TOPIC_PLAN_LOCALROUTE                 ="yhs1/msg_plan_local";

    //control topic: control mode
    const std::string TOPIC_CONTROL_MODE                    ="msg_control_mode";
    //control topic: lateral and longitudinal motion control
    const std::string TOPIC_CONTROL_MOTION                  ="msg_control_motion";
    //control topic: car lamp control
    const std::string TOPIC_CONTROL_LAMP                    ="msg_control_lamp";

}

#endif //_USRTOPIC_H