
#include "stdafx.h"
#include "SixdofDioAndPulseCount.h"

#define ASSERT_INDEX(index) if (index < 0 || index >= SXIDOF_MOTION_NUM) return;
#define AI_V_OUT_CHANNEL_COUNT 6
UserPCI1750_OutputPort MotionLockOutPorts[SXIDOF_MOTION_NUM] = 
{
	UserPCI1750_IDO0,
	UserPCI1750_IDO2,
	UserPCI1750_IDO4,
	UserPCI1750_IDO6,
	UserPCI1750_IDO8,
	UserPCI1750_IDO10,
};

UserPCI1750_OutputPort MotionEnableOutPorts[SXIDOF_MOTION_NUM] = 
{
	UserPCI1750_IDO1,
	UserPCI1750_IDO3,
	UserPCI1750_IDO5,
	UserPCI1750_IDO7,
	UserPCI1750_IDO9,
	UserPCI1750_IDO11,
};

UserPCI1750_InputPort SwitchInPorts[SXIDOF_MOTION_NUM] = 
{
	UserPCI1750_IDI0,
	UserPCI1750_IDI2,
	UserPCI1750_IDI4,
	UserPCI1750_IDI6,
	UserPCI1750_IDI8,
	UserPCI1750_IDI10,
};

UserPCI1723_AVO_Channel MotionAnalogVotiageOutPorts[SXIDOF_MOTION_NUM] = 
{
	UserPCI1723_AVO_Channel0,
	UserPCI1723_AVO_Channel1,
	UserPCI1723_AVO_Channel2,
	UserPCI1723_AVO_Channel3,
	UserPCI1723_AVO_Channel4,
	UserPCI1723_AVO_Channel5,
};

UserPCI1723_DIO_Channel BigMotionSwitchInPorts[SXIDOF_MOTION_NUM] =
{
	UserPCI1723_DIO_Channel8,
	UserPCI1723_DIO_Channel9,
	UserPCI1723_DIO_Channel10,
	UserPCI1723_DIO_Channel11,
	UserPCI1723_DIO_Channel12,
	UserPCI1723_DIO_Channel13,
};

UserPCI1723_DIO_Channel BigMotionAlarmInPort = UserPCI1723_DIO_Channel14;
UserPCI1723_DIO_Channel BigMotionBeiYongInPort = UserPCI1723_DIO_Channel15;

UserPCI1723_DIO_Channel BigMotionEnableOutPort[SXIDOF_MOTION_NUM] = 
{
	UserPCI1723_DIO_Channel0,
	UserPCI1723_DIO_Channel1,
	UserPCI1723_DIO_Channel2,
	UserPCI1723_DIO_Channel3,
	UserPCI1723_DIO_Channel4,
	UserPCI1723_DIO_Channel5,
};

UserPCI1750_OutputPort BigMotionStartOutPort = UserPCI1750_IDO1;
UserPCI1750_OutputPort BigMotionCheckStartOutPort = UserPCI1750_IDO3;

SixdofDioAndCount::SixdofDioAndCount()
{
	DataInit();
	Init();
}

SixdofDioAndCount::~SixdofDioAndCount()
{
	if (disposed == false)
	{
		Close();
	}
}

void SixdofDioAndCount::DataInit()
{
	for (auto i = 0; i < SXIDOF_MOTION_NUM; ++i)
	{
		VoltageValues[i] = 0;
	}
}

bool SixdofDioAndCount::Init()
{
	bool result = true;

	result = result && pci1716Card.Init();
	result = result && pci1723Card.Init();
	result = result && pci1750Card.Init();
	InitStatus();
	return result;
}

void SixdofDioAndCount::InitStatus()
{
	double vels[6] = {0, 0, 0, 0, 0, 0};

	SetMotionVel(vels);
	for (int i = 0; i < SXIDOF_MOTION_NUM; i++)
	{
		//BigMotionEnable(0, false);
		//SetMotionLockBit(i, false);
		//SetMotionEnableBit(i, false);
	}
	//Start(false);
	//CheckStart(false);
}

bool SixdofDioAndCount::Close()
{
	try
	{	
		InitStatus();

		pci1716Card.Close();
		pci1723Card.Close();
		pci1750Card.Close();
	}
	catch (exception* e)
	{
		printf(e->what());
		return false;
	}
	return true;
}

bool SixdofDioAndCount::Test()
{
	SetOilStart(true);
	Sleep(1000);
	SetOilStart(false);
	Sleep(1000);
	SetOilStop(true);
	Sleep(1000);
	SetOilStop(false);
	Sleep(1000);
	SetEnable(true);
	Sleep(1000);
	SetEnable(false);
	Sleep(1000);
	SetDisable(true);
	Sleep(1000);
	SetDisable(false);
	Sleep(1000);
	return true;
}

double * SixdofDioAndCount::ReadVoltageValues()	
{
    pci1716Card.ReadAnalogVotiageData();
	for(int i=0;i<SXIDOF_MOTION_NUM;i++)
	{

		VoltageValues[i]=pci1716Card.VoltageValue[i];

	}
	//VoltageValues=pci1716Card.VoltageValue;

	return pci1716Card.VoltageValue;
}

void SixdofDioAndCount::ClearVoltageValue()
{
	pci1716Card.ClearVoltageValue();
}

void SixdofDioAndCount::SetMotionLockBit(int index, bool bit)
{
	ASSERT_INDEX(index);
	pci1750Card.WriteBit(MotionLockOutPorts[index], bit);
}

void SixdofDioAndCount::SetMotionLockBit(bool* bits)
{
	for (auto i = 0;i < SXIDOF_MOTION_NUM;++i)
	{
		pci1750Card.WriteBit(MotionLockOutPorts[i], bits[i]);
	}
}

void SixdofDioAndCount::SetMotionEnableBit(int index, bool bit)
{
	ASSERT_INDEX(index);
	pci1750Card.WriteBit(MotionEnableOutPorts[index], bit);
}

void SixdofDioAndCount::SetMotionEnableBit(bool* bits)
{
	for (auto i = 0;i < SXIDOF_MOTION_NUM; ++i)
	{
		pci1750Card.WriteBit(MotionEnableOutPorts[i], bits[i]);
	}
	/////////
	pci1750Card.WriteBit(UserPCI1750_IDO7,0);
}

void SixdofDioAndCount::ReadKBit(int index, bool* bit)
{
	ASSERT_INDEX(index);
	pci1750Card.ReadBit(SwitchInPorts[index], bit);
}

void SixdofDioAndCount::ReadKBit(bool* bits)
{
	bool allBits[DI_BIT_PORT_COUNT];
	pci1750Card.ReadAllBits(allBits);
	for (auto i = 0;i < SXIDOF_MOTION_NUM;++i)
	{
		bits[i] = allBits[SwitchInPorts[i]];
	}
}

void SixdofDioAndCount::SetMotionVel(double * vels)
{
	double setV[AO_V_OUT_CHANNEL_COUNT] = {0};
	for (auto i = 0;i < SXIDOF_MOTION_NUM; ++i)
	{
		setV[(int)MotionAnalogVotiageOutPorts[i]] = vels[i];
	}
	pci1723Card.WriteAnalogVotiageData(setV);
}

void SixdofDioAndCount::SetMotionVel(int index, double vel)
{
	//ASSERT_INDEX(index);
	pci1723Card.WriteAnalogVotiageData(MotionAnalogVotiageOutPorts[index], vel);
}

void SixdofDioAndCount::EnableAllMotor(bool isEnable)
{
    for (int i = 0; i < SXIDOF_MOTION_NUM; i++)
    {     
        SetMotionEnableBit(i, isEnable);
		SetMotionEnableBit(i, isEnable);
    }

}

void SixdofDioAndCount::BigMotionEnable(int index, bool bit)
{
	ASSERT_INDEX(index);
	pci1723Card.WriteBit(BigMotionEnableOutPort[index], bit);
}

void SixdofDioAndCount::BigMotionEnable(bool* bits)
{
	for (auto i = 0;i < SXIDOF_MOTION_NUM; ++i)
	{
		pci1723Card.WriteBit(BigMotionEnableOutPort[i], bits[i]);
	}
}

void SixdofDioAndCount::BigMotionReadKBit(int index, bool* bit)
{
	ASSERT_INDEX(index);
	pci1723Card.ReadBit(BigMotionSwitchInPorts[index], bit);
}

void SixdofDioAndCount::BigMotionReadKBit(bool* bits)
{
	bool allBits[DI_BIT_PORT_COUNT];
	pci1723Card.ReadAllBits(allBits);
	for (auto i = 0;i < SXIDOF_MOTION_NUM;++i)
	{
		bits[i] = allBits[BigMotionSwitchInPorts[i]];
	}
}

void SixdofDioAndCount::BigMotionEnableAllMotor(bool isEnable)
{
	for (int i = 0; i < SXIDOF_MOTION_NUM; i++)
	{
		BigMotionEnable(i, isEnable);
	}
}

bool SixdofDioAndCount::BigMotionReadAlarm()
{
	bool bit = false;
	pci1723Card.ReadBit(BigMotionAlarmInPort, &bit);
	return bit;
}

void SixdofDioAndCount::Start(bool isStart)
{
	pci1750Card.WriteBit(BigMotionStartOutPort, isStart);
}

void SixdofDioAndCount::CheckStart(bool isStart)
{
	pci1750Card.WriteBit(BigMotionCheckStartOutPort, isStart);
}

bool SixdofDioAndCount::BigMotionTest()
{
	Start(true);
	CheckStart(true);
	bool bit = true;
	for (int i = 0;i < 6 ;++i)
	{
		BigMotionEnable(i, 1);
	}
	auto r = bit;
	bool kbits[SXIDOF_MOTION_NUM] = { true,true,true,true,true,true };
	BigMotionReadKBit(kbits);
	r = bit;
	double vels[6] = {0.05, 0, 0, 0.1, 0, 0};
	SetMotionVel(vels);
	double vels0[6] = {0, 0, 0, 0, 0, 0};
	SetMotionVel(vels0);
	r = bit;
	auto count = ReadVoltageValues();
	double count0 = count[3];
	r = bit;
	for (int i = 0;i < 6 ;++i)
	{
		BigMotionEnable(i, 0);
	}
	r = bit;
	Start(false);
	CheckStart(false);
	return true;
}

//读取是否断电
void SixdofDioAndCount::IsNoPower(bool* bit)
{
	pci1750Card.ReadBit(UserPCI1750_IDI12, bit);
}

//读取是否有超低压保护
void SixdofDioAndCount::IsLowerPower(bool* bit)
{
	pci1750Card.ReadBit(UserPCI1750_IDI14, bit);
}

// 设置使能
void SixdofDioAndCount::SetDisable(bool bit)
{
	pci1750Card.WriteBit(UserPCI1750_IDO5, bit);
}

void SixdofDioAndCount::SetEnable(bool bit)
{
	pci1750Card.WriteBit(UserPCI1750_IDO7, bit);
}

void SixdofDioAndCount::SetOilStart(bool bit)
{
	pci1750Card.WriteBit(UserPCI1750_IDO13, bit);

}

void SixdofDioAndCount::SetOilStop(bool bit)
{
	pci1750Card.WriteBit(UserPCI1750_IDO15, bit);
}
