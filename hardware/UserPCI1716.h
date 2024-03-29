//#pragma once

#ifndef _PCI1716_H_
#define _PCI1716_H_

#include "UserPCICard.h"
//#include"../ECATSampleDlg.h"

#include "../bdaqctrl.h"

// 包含研华命名空间
using namespace Automation::BDaq;

#define AI_V_OUT_CHANNEL_COUNT 6

class UserPCI1716 :public UserPCICard
{
public:
	UserPCI1716(void);
	~UserPCI1716(void);
	// 获取简介
	string GetIntroduction();
	// 初始化板卡
	bool Init();
	// 关闭板卡
	bool Close();
	//读取模拟输入量
	void ReadAnalogVotiageData();
	// 更新模拟输入量
	void RenewVoltageValue();
	// 模拟输入量清零
	void ClearVoltageValue();
	// 设备号
	int DeviceID;
	// 模拟输入量当前新值
	double VoltageValue[AI_V_OUT_CHANNEL_COUNT]; 
private:
	InstantAiCtrl * instantAiCtrl;

	bool disposed;

};

#endif
