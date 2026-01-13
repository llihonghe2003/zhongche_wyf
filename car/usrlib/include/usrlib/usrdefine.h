/**
 * @file    usrdefine.h
 * @author  李峥
 * @date    2020-12-08
 *
 * @brief   自定义公用库usrdefine，头文件
*/

#ifndef _USRDEFINE_H
#define _USRDEFINE_H

#include <ros/ros.h>

namespace UsrLib{

    //坐标系类型
    const uint8_t               FRAME_FRD                       =0;  //前-右-下坐标系
    const uint8_t               FRAME_RFU                       =1;  //右-前-上坐标系

    //图像编码方式
    const uint8_t               IMAGE_ENCODE_RGB                =0;  //RGB编码
    const uint8_t               IMAGE_ENCODE_BGR                =1;  //BGR编码
    const uint8_t               IMAGE_ENCODE_MONO               =2;  //灰度

    //检测目标标签
    const uint8_t               ObJ_LABEL_UNKNOWN               =0;  //未知
    const uint8_t               ObJ_LABEL_HUMAN                 =1;  //人
    const uint8_t               ObJ_LABEL_BICYCLE               =2;  //自行车
    const uint8_t               ObJ_LABEL_MOTOR                 =3;  //电动车/摩托车
    const uint8_t               ObJ_LABEL_CAR                   =4;  //小型汽车
    const uint8_t               ObJ_LABEL_BUS                   =5;  //公交车
    const uint8_t               ObJ_LABEL_TRUCK                 =6;  //卡车
    const uint8_t               ObJ_LABEL_OTHERMOVING           =7;  //其他动态目标


    const uint8_t               ObJ_LABEL_BUMP                  =8;  //减速带
    const uint8_t               ObJ_LABEL_BLOCK                 =9;  //路障
    const uint8_t               ObJ_LABEL_PIT                   =10; //陷坑
    const uint8_t               OBJ_LABEL_GATE                  =11; //闸杆
    const uint8_t               ObJ_LABEL_ROADEDGE              =12; //路沿

    const uint8_t               ObJ_LABEL_TRAFFICSIGN           =13; //交通标识
    const uint8_t               ObJ_LABEL_TRAFFICLAMP           =14; //交通灯
    const uint8_t               ObJ_LABEL_LANELINE              =15; //车道线
    const uint8_t               OBJ_LABEL_STOPLINE              =16; //停止线
    const uint8_t               OBJ_LABEL_SIDEWALK              =17; //人行道
    const uint8_t               OBJ_LABEL_PARKING               =18; //停车位
    const uint8_t               OBJ_LABEL_TURNWAIT              =19; //待转区
    const uint8_t               OBJ_LABEL_NOSTOP                =20; //禁停区
    const uint8_t               ObJ_LABEL_OTHERSTATIC           =21; //其他静态目标

    //检测目标行为
    const uint8_t               OBJ_BEHAVIOR_UNKNOWN            =0;  //未知
    const uint8_t               OBJ_BEHAVIOR_STOP               =1;  //停止
    const uint8_t               OBJ_BEHAVIOR_FORWARD            =2;  //前进
    const uint8_t               OBJ_BEHAVIOR_BACKOFF            =3;  //后退
    const uint8_t               OBJ_BEHAVIOR_BRANCHLEFT         =4;  //向左换道
    const uint8_t               OBJ_BEHAVIOR_BRANCHRIGHT        =5;  //向右换道
    const uint8_t               OBJ_BEHAVIOR_TURNLEFT           =6;  //向左转弯
    const uint8_t               OBJ_BEHAVIOR_TURNRIGHT          =7;  //向右转弯
    const uint8_t               OBJ_BEHAVIOR_UTURN              =8;  //掉头（U型弯）  

    //车道线类型
    const uint8_t               LANELINE_TYPE_UNKNOWN           =0;  //未知
    const uint8_t               LANELINE_TYPE_SOLID             =1;  //单实线
    const uint8_t               LANELINE_TYPE_DASHED            =2;  //单虚线
    const uint8_t               LANELINE_TYPE_DOUBLE            =3;  //双实线
    const uint8_t               LANELINE_TYPE_DASHEDTOSOLID     =4;  //虚-实线，靠近当前车道一边为虚线，允许变道
    const uint8_t               LANELINE_TYPE_SOLIDTODASHED     =5;  //实-虚线，靠近当前车道一边为实线，禁止变道

    //车道线颜色
    const uint8_t               LANELINE_COLOR_UNKNOWN          =0;  //未知
    const uint8_t               LANELINE_COLOR_WHITE            =1;  //白色
    const uint8_t               LANELINE_COLOR_YELLOW           =2;  //黄色

    //交通灯颜色
    const uint8_t               LAMP_COLOR_UNKNOWN              =0;  //未知
    const uint8_t               LAMP_COLOR_RED                  =1;  //红灯
    const uint8_t               LAMP_COLOR_YELLOW               =2;  //黄灯
    const uint8_t               LAMP_COLOR_GREEN                =3;  //绿灯

    //控制模式
    const uint8_t               CTRLMODE_MANU                   =0; //手动
    const uint8_t               CTRLMODE_AUTO                   =1; //自动
    const uint8_t               CTRLMODE_AUTOSPEED              =2; //速度自动
    const uint8_t               CTRLMODE_AUTOSTEER              =3; //转向自动

    //预定义场景
    const uint8_t               SCENARIO_FREE                   =0; //自由场景
    const uint8_t               SCENARIO_LANE                   =1; //道路场景
    const uint8_t               SCENARIO_PARKING                =2; //泊车场景

    //预定义行为
    const uint8_t               BEHAVIOR_STOP                   =0; //停车
    const uint8_t               BEHAVIOR_FOWARDRUN              =1; //前进
    const uint8_t               BEHAVIOR_BACKOFF                =2; //倒车
    const uint8_t               BEHAVIOR_FOLLOW                 =3; //跟车行驶
    const uint8_t               BEHAVIOR_LEFTTURN               =4; //左转（U-turn）
    const uint8_t               BEHAVIOR_RIGHTTURN              =5; //右转（U-turn）
    const uint8_t               BEHAVIOR_LEFTMERGE              =6; //向左并道
    const uint8_t               BEHAVIOR_RIGHTMERGE             =7; //向右并道
    const uint8_t               BEHAVIOR_LEFTPARK               =8; //左侧停车
    const uint8_t               BEHAVIOR_RIGHTPARK              =9; //右侧停车

}

#endif //_USRDEFINE_H