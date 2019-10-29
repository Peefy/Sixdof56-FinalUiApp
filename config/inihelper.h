#pragma once

#ifndef __INI_HELPER_H_
#define __INI_HELPER_H_

#include <Windows.h> 
#include <fstream>
#include <string>

using namespace std;

#include "../TYPE_DEF.H"
#include "../communication/sixdof.h"

// 软件参数json配置文件名称
#define JSON_FILE_NAME                "config.json"
// 平台算法json配置文件名称
#define JSON_PARA_FILE_NAME           "paraconfig.json"

// json-key:是否停止回中
#define JSON_STOP_AND_MIDDLE_KEY      "stopAndMiddle"
// json-key:UDP通信对方IP地址
#define JSON_UDP_IP_KEY               "udpip"
// json-key:UDP通信对方IP端口号
#define JSON_UDP_PORT_KEY             "udpport"
// json-key:UDP通信自身IP地址
#define JSON_UDP_SELF_IP_KEY          "udpselfip"
// json-key:UDP通信自身IP端口号
#define JSON_UDP_SELF_PORT_KEY        "udpselfport"

// json-key:洗出算法加速度高通滤波器参数
#define JSON_HPF_ACC_WN_KEY           "hpfAccWn"
// json-key:洗出算法加速度低通滤波器参数
#define JSON_LPF_ACC_WN_KEY           "lpfAccWn"
// json-key:洗出算法角速度高通滤波器参数
#define JSON_ANG_SPD_WN_KEY           "hpfAngleSpdWn"
// json-key:横滚角增益
#define JSON_ROLL_SCALE_KEY           "rollScale"
// json-key:偏航角增益
#define JSON_YAW_SCALE_KEY            "yawScale"
// json-key:俯仰角增益
#define JSON_PITCH_SCALE_KEY          "pitchScale"
// json-key:横滚角平滑系数
#define JSON_ROLL_FILTER_LEVEL_KEY    "rollFilterLevel"
// json-key:偏航角平滑系数
#define JSON_YAW_FILTER_LEVEL_KEY     "yawFilterLevel"
// json-key:俯仰角平滑系数
#define JSON_PITCH_FILTER_LEVEL_KEY   "pitchFilterLevel"

// json-key:x线位移增益
#define JSON_X_SCALE_KEY              "xScale"
// json-key:y线位移增益
#define JSON_Y_SCALE_KEY              "yScale"
// json-key:z线位移增益
#define JSON_Z_SCALE_KEY              "zScale"
// json-key:x线位移平滑系数
#define JSON_X_FILTER_LEVEL_KEY       "xFilterLevel"
// json-key:y线位移平滑系数
#define JSON_Y_FILTER_LEVEL_KEY       "yFilterLevel"
// json-key:z线位移平滑系数
#define JSON_Z_FILTER_LEVEL_KEY       "zFilterLevel"

// json-key:直控X加速度增益
#define JSON_DIRECT_ACC_X_SCALE_KEY        "directAccXScale"
//  json-key:直控X加速度增益
#define JSON_DIRECT_ACC_Y_SCALE_KEY        "directAccYScale"
//  json-key:直控X加速度增益
#define JSON_DIRECT_ACC_Z_SCALE_KEY        "directAccZScale"
//  json-key:直控横滚角速度增益
#define JSON_DIRECT_SPEED_ROLL_SCALE_KEY   "directSpeedRollScale"
//  json-key:直控俯仰角速度增益
#define JSON_DIRECT_SPPED_PITCH_SCALE_KEY  "directSpeedPitchScale"
//  json-key:直控偏航角速度增益
#define JSON_DIRECT_SPEED_YAW_SCALE_KEY    "directSpeedYawScale"
//  json-key:直控横滚角增益	
#define JSON_DIRECT_ANGLE_ROLL_SCALE_KEY   "directAngleRollScale"
//  json-key:直控俯仰角增益
#define JSON_DIRECT_ANGLE_PITCH_SCALE_KEY  "directAnglePitchScale"
//  json-key:直控偏航角增益
#define JSON_DIRECT_ANGLE_YAW_SCALE_KEY    "directAngleYawScale"

// json-key:惯导稳定PID控制器P
#define JSON_NAVI_P_KEY               "naviP"
// json-key:惯导稳定PID控制器I
#define JSON_NAVI_I_KEY               "naviI"
// json-key:惯导稳定PID控制器D
#define JSON_NAVI_D_KEY               "naviD"

// json-key:平台上升PID控制器P_1
#define JSON_RISE_P_1_KEY               "riseP1"
// json-key:平台上升PID控制器I_1
#define JSON_RISE_I_1_KEY               "riseI1"
// json-key:平台上升PID控制器D_1
#define JSON_RISE_D_1_KEY               "riseD1"

// json-key:平台上升PID控制器P_2
#define JSON_RISE_P_2_KEY               "riseP2"
// json-key:平台上升PID控制器I_2
#define JSON_RISE_I_2_KEY               "riseI2"
// json-key:平台上升PID控制器D_2
#define JSON_RISE_D_2_KEY               "riseD2"

// json-key:平台上升PID控制器P_3
#define JSON_RISE_P_3_KEY               "riseP3"
// json-key:平台上升PID控制器I_3
#define JSON_RISE_I_3_KEY               "riseI3"
// json-key:平台上升PID控制器D_3
#define JSON_RISE_D_3_KEY               "riseD3"

// json-key:平台上升PID控制器P_4
#define JSON_RISE_P_4_KEY               "riseP4"
// json-key:平台上升PID控制器I_4
#define JSON_RISE_I_4_KEY               "riseI4"
// json-key:平台上升PID控制器D_4
#define JSON_RISE_D_4_KEY               "riseD4"

// json-key:平台上升PID控制器P_5
#define JSON_RISE_P_5_KEY               "riseP5"
// json-key:平台上升PID控制器I_5
#define JSON_RISE_I_5_KEY               "riseI5"
// json-key:平台上升PID控制器D_5
#define JSON_RISE_D_5_KEY               "riseD5"

// json-key:平台上升PID控制器P_6
#define JSON_RISE_P_6_KEY               "riseP6"
// json-key:平台上升PID控制器I_6
#define JSON_RISE_I_6_KEY               "riseI6"
// json-key:平台上升PID控制器D_6
#define JSON_RISE_D_6_KEY               "riseD6"

// json-key:平台运行PID控制器P_1
#define JSON_MOTION_P_1_KEY             "motionP1"
// json-key:平台运行PID控制器I_1
#define JSON_MOTION_I_1_KEY             "motionI1"
// json-key:平台运行PID控制器D_1
#define JSON_MOTION_D_1_KEY             "motionD1"

// json-key:平台运行PID控制器P_2
#define JSON_MOTION_P_2_KEY             "motionP2"
// json-key:平台运行PID控制器I_2
#define JSON_MOTION_I_2_KEY             "motionI2"
// json-key:平台运行PID控制器D_2
#define JSON_MOTION_D_2_KEY             "motionD2"

// json-key:平台运行PID控制器P_3
#define JSON_MOTION_P_3_KEY             "motionP3"
// json-key:平台运行PID控制器I_3
#define JSON_MOTION_I_3_KEY             "motionI3"
// json-key:平台运行PID控制器D_3
#define JSON_MOTION_D_3_KEY             "motionD3"

// json-key:平台运行PID控制器P_4
#define JSON_MOTION_P_4_KEY             "motionP4"
// json-key:平台运行PID控制器I_4
#define JSON_MOTION_I_4_KEY             "motionI4"
// json-key:平台运行PID控制器D_4
#define JSON_MOTION_D_4_KEY             "motionD4"

// json-key:平台运行PID控制器P_5
#define JSON_MOTION_P_5_KEY             "motionP5"
// json-key:平台运行PID控制器I_5
#define JSON_MOTION_I_5_KEY             "motionI5"
// json-key:平台运行PID控制器D_5
#define JSON_MOTION_D_5_KEY             "motionD5"

// json-key:平台运行PID控制器P_6
#define JSON_MOTION_P_6_KEY             "motionP6"
// json-key:平台运行PID控制器I_6
#define JSON_MOTION_I_6_KEY             "motionI6"
// json-key:平台运行PID控制器D_6
#define JSON_MOTION_D_6_KEY             "motionD6"

namespace config {
	// 产生默认的config.ini文件
    void GenerateDefaultConfigFile();
	// 读取config.ini文件中的串口端口号和波特率
    void ReadAll(bool& result, int& baud, int& portnum);
	// 读取config.ini文件中的增益系数
	int ReadScale();
	// 记录recorddouble.txt中记录的平台状态和编码器读数
	void RecordStatusAndPulse(char* status, int statusInt, I32* pulse);
	// 记录recorddouble.txt中记录的平台状态和编码器读数
	void RecordStatusAndPulse(char* status, int statusInt, double* pulse);
	// 读取recorddouble.txt中记录的平台状态和编码器读数
	void ReadStatusAndPulse(int& statusInt, I32* pulse);
	// 读取recorddouble.txt中记录的平台状态和编码器读数
	void ReadStatusAndPulse(int& statusInt, double* pulse);
	// 读取是否停止并回中的配置
	bool ReadIsAutoStopAndMiddle();
	// 读取json文件-value类型：范形
	template<typename T>
	T ParseJsonFromFile(const char* filename, const char* key);
	// 读取json文件-value类型：字符串类型 std::string
	string ParseStringJsonFromFile(const char* filename, const char* key);
	// 读取json文件-value类型：32位整数类型 int
	int ParseIntJsonFromFile(const char* filename, const char* key);
	// 读取json文件-value类型：64位双精度浮点型 double
	double ParseDoubleJsonFromFile(const char* filename, const char* key);
	// 记录姿态数据
	void RecordData(const char * filename, double roll, double pitch, double yaw);
}

#endif // !__INI_HELPER_H_

