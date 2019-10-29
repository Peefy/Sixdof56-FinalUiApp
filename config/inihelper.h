#pragma once

#ifndef __INI_HELPER_H_
#define __INI_HELPER_H_

#include <Windows.h> 
#include <fstream>
#include <string>

using namespace std;

#include "../TYPE_DEF.H"
#include "../communication/sixdof.h"

// �������json�����ļ�����
#define JSON_FILE_NAME                "config.json"
// ƽ̨�㷨json�����ļ�����
#define JSON_PARA_FILE_NAME           "paraconfig.json"

// json-key:�Ƿ�ֹͣ����
#define JSON_STOP_AND_MIDDLE_KEY      "stopAndMiddle"
// json-key:UDPͨ�ŶԷ�IP��ַ
#define JSON_UDP_IP_KEY               "udpip"
// json-key:UDPͨ�ŶԷ�IP�˿ں�
#define JSON_UDP_PORT_KEY             "udpport"
// json-key:UDPͨ������IP��ַ
#define JSON_UDP_SELF_IP_KEY          "udpselfip"
// json-key:UDPͨ������IP�˿ں�
#define JSON_UDP_SELF_PORT_KEY        "udpselfport"

// json-key:ϴ���㷨���ٶȸ�ͨ�˲�������
#define JSON_HPF_ACC_WN_KEY           "hpfAccWn"
// json-key:ϴ���㷨���ٶȵ�ͨ�˲�������
#define JSON_LPF_ACC_WN_KEY           "lpfAccWn"
// json-key:ϴ���㷨���ٶȸ�ͨ�˲�������
#define JSON_ANG_SPD_WN_KEY           "hpfAngleSpdWn"
// json-key:���������
#define JSON_ROLL_SCALE_KEY           "rollScale"
// json-key:ƫ��������
#define JSON_YAW_SCALE_KEY            "yawScale"
// json-key:����������
#define JSON_PITCH_SCALE_KEY          "pitchScale"
// json-key:�����ƽ��ϵ��
#define JSON_ROLL_FILTER_LEVEL_KEY    "rollFilterLevel"
// json-key:ƫ����ƽ��ϵ��
#define JSON_YAW_FILTER_LEVEL_KEY     "yawFilterLevel"
// json-key:������ƽ��ϵ��
#define JSON_PITCH_FILTER_LEVEL_KEY   "pitchFilterLevel"

// json-key:x��λ������
#define JSON_X_SCALE_KEY              "xScale"
// json-key:y��λ������
#define JSON_Y_SCALE_KEY              "yScale"
// json-key:z��λ������
#define JSON_Z_SCALE_KEY              "zScale"
// json-key:x��λ��ƽ��ϵ��
#define JSON_X_FILTER_LEVEL_KEY       "xFilterLevel"
// json-key:y��λ��ƽ��ϵ��
#define JSON_Y_FILTER_LEVEL_KEY       "yFilterLevel"
// json-key:z��λ��ƽ��ϵ��
#define JSON_Z_FILTER_LEVEL_KEY       "zFilterLevel"

// json-key:ֱ��X���ٶ�����
#define JSON_DIRECT_ACC_X_SCALE_KEY        "directAccXScale"
//  json-key:ֱ��X���ٶ�����
#define JSON_DIRECT_ACC_Y_SCALE_KEY        "directAccYScale"
//  json-key:ֱ��X���ٶ�����
#define JSON_DIRECT_ACC_Z_SCALE_KEY        "directAccZScale"
//  json-key:ֱ�غ�����ٶ�����
#define JSON_DIRECT_SPEED_ROLL_SCALE_KEY   "directSpeedRollScale"
//  json-key:ֱ�ظ������ٶ�����
#define JSON_DIRECT_SPPED_PITCH_SCALE_KEY  "directSpeedPitchScale"
//  json-key:ֱ��ƫ�����ٶ�����
#define JSON_DIRECT_SPEED_YAW_SCALE_KEY    "directSpeedYawScale"
//  json-key:ֱ�غ��������	
#define JSON_DIRECT_ANGLE_ROLL_SCALE_KEY   "directAngleRollScale"
//  json-key:ֱ�ظ���������
#define JSON_DIRECT_ANGLE_PITCH_SCALE_KEY  "directAnglePitchScale"
//  json-key:ֱ��ƫ��������
#define JSON_DIRECT_ANGLE_YAW_SCALE_KEY    "directAngleYawScale"

// json-key:�ߵ��ȶ�PID������P
#define JSON_NAVI_P_KEY               "naviP"
// json-key:�ߵ��ȶ�PID������I
#define JSON_NAVI_I_KEY               "naviI"
// json-key:�ߵ��ȶ�PID������D
#define JSON_NAVI_D_KEY               "naviD"

// json-key:ƽ̨����PID������P_1
#define JSON_RISE_P_1_KEY               "riseP1"
// json-key:ƽ̨����PID������I_1
#define JSON_RISE_I_1_KEY               "riseI1"
// json-key:ƽ̨����PID������D_1
#define JSON_RISE_D_1_KEY               "riseD1"

// json-key:ƽ̨����PID������P_2
#define JSON_RISE_P_2_KEY               "riseP2"
// json-key:ƽ̨����PID������I_2
#define JSON_RISE_I_2_KEY               "riseI2"
// json-key:ƽ̨����PID������D_2
#define JSON_RISE_D_2_KEY               "riseD2"

// json-key:ƽ̨����PID������P_3
#define JSON_RISE_P_3_KEY               "riseP3"
// json-key:ƽ̨����PID������I_3
#define JSON_RISE_I_3_KEY               "riseI3"
// json-key:ƽ̨����PID������D_3
#define JSON_RISE_D_3_KEY               "riseD3"

// json-key:ƽ̨����PID������P_4
#define JSON_RISE_P_4_KEY               "riseP4"
// json-key:ƽ̨����PID������I_4
#define JSON_RISE_I_4_KEY               "riseI4"
// json-key:ƽ̨����PID������D_4
#define JSON_RISE_D_4_KEY               "riseD4"

// json-key:ƽ̨����PID������P_5
#define JSON_RISE_P_5_KEY               "riseP5"
// json-key:ƽ̨����PID������I_5
#define JSON_RISE_I_5_KEY               "riseI5"
// json-key:ƽ̨����PID������D_5
#define JSON_RISE_D_5_KEY               "riseD5"

// json-key:ƽ̨����PID������P_6
#define JSON_RISE_P_6_KEY               "riseP6"
// json-key:ƽ̨����PID������I_6
#define JSON_RISE_I_6_KEY               "riseI6"
// json-key:ƽ̨����PID������D_6
#define JSON_RISE_D_6_KEY               "riseD6"

// json-key:ƽ̨����PID������P_1
#define JSON_MOTION_P_1_KEY             "motionP1"
// json-key:ƽ̨����PID������I_1
#define JSON_MOTION_I_1_KEY             "motionI1"
// json-key:ƽ̨����PID������D_1
#define JSON_MOTION_D_1_KEY             "motionD1"

// json-key:ƽ̨����PID������P_2
#define JSON_MOTION_P_2_KEY             "motionP2"
// json-key:ƽ̨����PID������I_2
#define JSON_MOTION_I_2_KEY             "motionI2"
// json-key:ƽ̨����PID������D_2
#define JSON_MOTION_D_2_KEY             "motionD2"

// json-key:ƽ̨����PID������P_3
#define JSON_MOTION_P_3_KEY             "motionP3"
// json-key:ƽ̨����PID������I_3
#define JSON_MOTION_I_3_KEY             "motionI3"
// json-key:ƽ̨����PID������D_3
#define JSON_MOTION_D_3_KEY             "motionD3"

// json-key:ƽ̨����PID������P_4
#define JSON_MOTION_P_4_KEY             "motionP4"
// json-key:ƽ̨����PID������I_4
#define JSON_MOTION_I_4_KEY             "motionI4"
// json-key:ƽ̨����PID������D_4
#define JSON_MOTION_D_4_KEY             "motionD4"

// json-key:ƽ̨����PID������P_5
#define JSON_MOTION_P_5_KEY             "motionP5"
// json-key:ƽ̨����PID������I_5
#define JSON_MOTION_I_5_KEY             "motionI5"
// json-key:ƽ̨����PID������D_5
#define JSON_MOTION_D_5_KEY             "motionD5"

// json-key:ƽ̨����PID������P_6
#define JSON_MOTION_P_6_KEY             "motionP6"
// json-key:ƽ̨����PID������I_6
#define JSON_MOTION_I_6_KEY             "motionI6"
// json-key:ƽ̨����PID������D_6
#define JSON_MOTION_D_6_KEY             "motionD6"

namespace config {
	// ����Ĭ�ϵ�config.ini�ļ�
    void GenerateDefaultConfigFile();
	// ��ȡconfig.ini�ļ��еĴ��ڶ˿ںźͲ�����
    void ReadAll(bool& result, int& baud, int& portnum);
	// ��ȡconfig.ini�ļ��е�����ϵ��
	int ReadScale();
	// ��¼recorddouble.txt�м�¼��ƽ̨״̬�ͱ���������
	void RecordStatusAndPulse(char* status, int statusInt, I32* pulse);
	// ��¼recorddouble.txt�м�¼��ƽ̨״̬�ͱ���������
	void RecordStatusAndPulse(char* status, int statusInt, double* pulse);
	// ��ȡrecorddouble.txt�м�¼��ƽ̨״̬�ͱ���������
	void ReadStatusAndPulse(int& statusInt, I32* pulse);
	// ��ȡrecorddouble.txt�м�¼��ƽ̨״̬�ͱ���������
	void ReadStatusAndPulse(int& statusInt, double* pulse);
	// ��ȡ�Ƿ�ֹͣ�����е�����
	bool ReadIsAutoStopAndMiddle();
	// ��ȡjson�ļ�-value���ͣ�����
	template<typename T>
	T ParseJsonFromFile(const char* filename, const char* key);
	// ��ȡjson�ļ�-value���ͣ��ַ������� std::string
	string ParseStringJsonFromFile(const char* filename, const char* key);
	// ��ȡjson�ļ�-value���ͣ�32λ�������� int
	int ParseIntJsonFromFile(const char* filename, const char* key);
	// ��ȡjson�ļ�-value���ͣ�64λ˫���ȸ����� double
	double ParseDoubleJsonFromFile(const char* filename, const char* key);
	// ��¼��̬����
	void RecordData(const char * filename, double roll, double pitch, double yaw);
}

#endif // !__INI_HELPER_H_

