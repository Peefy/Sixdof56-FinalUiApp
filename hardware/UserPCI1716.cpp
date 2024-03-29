#include "stdafx.h"
#include "UserPCI1716.h"


#define CHK_RESULT(ret) {if(BioFailed(ret))break;}
#define ASSERT_CHANNEL(channel) if (channel < 0 || channel > 7) return;
#define ASSERT_RANGE_CHANNEL(channel) if (channel < 0 || channel > 15) return;

#define AI_PORT_START_INDE 0
#define AI_PORT_COUNT 6
const wchar_t* PCI1716DeviceDescription = L"PCI-1716L,BID#0";
const wchar_t* PCI1716ProfilePath = L"DemoDevice.xml";

// 包含研华命名空间
using namespace Automation::BDaq;

UserPCI1716::UserPCI1716(void)
{
		instantAiCtrl = AdxInstantAiCtrlCreate();
		Init();
}


UserPCI1716::~UserPCI1716(void)
{
	if (disposed == false)
	{
		Close();
	}
}

string UserPCI1716::GetIntroduction()
{
	return "D/A卡输出+-5V电压控制信号";
}


bool UserPCI1716::Init()
{
	ErrorCode ret = Success;
	do 
	{
		DeviceInformation devInfo(PCI1716DeviceDescription);
		ret = instantAiCtrl->setSelectedDevice(devInfo);
		CHK_RESULT(ret);
	} while (false);
	return ret == Success;
}


bool UserPCI1716::Close()
{
	instantAiCtrl->Cleanup();
	instantAiCtrl->Dispose();
	disposed = true;
	return true;
}

void UserPCI1716::ReadAnalogVotiageData()
{
	instantAiCtrl->Read(AI_PORT_START_INDE, AI_PORT_COUNT, &VoltageValue[0]);
}

void UserPCI1716::RenewVoltageValue()
{
	//方法过期，功能同ReadAnalogVotiageData()
	instantAiCtrl->Read(AI_PORT_START_INDE, AI_PORT_COUNT, &VoltageValue[0]);
}

void UserPCI1716::ClearVoltageValue()
{

}