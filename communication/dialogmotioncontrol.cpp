
#include "stdafx.h"

#include <stdlib.h>
#include <math.h>
#include <thread>

#include "dialogmotioncontrol.h"
#include "../Sixdofdll2010.h"

#include "../config/inihelper.h"
#include "../control/pid.h"

#define RANGE_V(x, min, max)    (((x)<(min) ? (min) : ( (x)>(max) ? (max):(x) )))
#define ASSERT_INDEX(index) 	if (index < 0 && index >= AXES_COUNT) return;

// 平台运行过程PID控制参数-P
#define MOTION_P 0.44
// 平台运行过程PID控制参数-I
#define MOTION_I 0.0002
// 平台运行过程PID控制参数-D
#define MOTION_D 0.0
// 平台运行过程当中最大速度
#define MAX_VEL  5.0
// 平台上升过程PID控制参数-P
#define RISE_MOTION_P 0.06
// 平台上升过程PID控制参数-I
#define RISE_MOTION_I 0.00002
// 平台上升过程PID控制参数-D
#define RISE_MOTION_D 0.0
// 平台上升过程当中最大速度
#define RISE_MIN_VEL  0.1
#define RISE_MAX_VEL  0.5

// PID临时变量
static double p = 0.0001;
static double i = 0.00001;
static double d = 0.0;

// 开环连续位置控制模式Csp的临时变量
static double last_pulse[AXES_COUNT] = { 0, 0, 0, 0, 0, 0 };
static double now_vel[AXES_COUNT] = { 0, 0, 0, 0, 0, 0 };
static double last_str_vel[AXES_COUNT] = { 0, 0, 0, 0, 0, 0 };
static double speed_scale = 100;
static ULONG last_pulses[AXES_COUNT] = {0};

// 运行过程六个电机的PID控制器
static PID_Type MotionLocationPidControler[AXES_COUNT] = 
{
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL }
};

// 上升过程六个电机的PID控制器
static PID_Type MotionRisePidControler[AXES_COUNT] = 
{
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL }
};

// 构造函数
DialogMotionControl::DialogMotionControl()
{
	InitData();
	ReadParaFromFile();
	InitCard();
}

// 析构函数
DialogMotionControl::~DialogMotionControl()
{
	if (this->disposed == false)
	{
		
	}
}

void DialogMotionControl::InitData()
{
	MIDDLE_POS = 2.4;
	disposed = false; 
	for (int ii = 0; ii < AXES_COUNT; ++ii)
	{
		NowPluse[ii] = 0;
		pos[ii] = 0;
		vels[ii] = 0;
	}
	AvrPulse = 0;
}

// 从配置文件中读取PID参数
void DialogMotionControl::ReadParaFromFile()
{
	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_P_1_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_I_1_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_D_1_KEY);
	MyControllerSetPidPara(&MotionRisePidControler[0], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_P_2_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_I_2_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_D_2_KEY);
	MyControllerSetPidPara(&MotionRisePidControler[1], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_P_3_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_I_3_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_D_3_KEY);
	MyControllerSetPidPara(&MotionRisePidControler[2], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_P_4_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_I_4_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_D_4_KEY);
	MyControllerSetPidPara(&MotionRisePidControler[3], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_P_5_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_I_5_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_D_5_KEY);
	MyControllerSetPidPara(&MotionRisePidControler[4], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_P_6_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_I_6_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_RISE_D_6_KEY);
	MyControllerSetPidPara(&MotionRisePidControler[5], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_P_1_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_I_1_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_D_1_KEY);
	MyControllerSetPidPara(&MotionLocationPidControler[0], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_P_2_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_I_2_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_D_2_KEY);
	MyControllerSetPidPara(&MotionLocationPidControler[1], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_P_3_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_I_3_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_D_3_KEY);
	MyControllerSetPidPara(&MotionLocationPidControler[2], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_P_4_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_I_4_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_D_4_KEY);
	MyControllerSetPidPara(&MotionLocationPidControler[3], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_P_5_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_I_5_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_D_5_KEY);
	MyControllerSetPidPara(&MotionLocationPidControler[4], p, i, d);

	p = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_P_6_KEY);
	i = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_I_6_KEY);
	d = config::ParseDoubleJsonFromFile(JSON_PARA_FILE_NAME, JSON_MOTION_D_6_KEY);
	MyControllerSetPidPara(&MotionLocationPidControler[5], p, i, d);
}

// 初始化所有硬件板卡
bool DialogMotionControl::InitCard()
{
	return sixdofDioAndCount.Init();
}

// 关闭所有板卡，并保存平台状态
void DialogMotionControl::Close(SixDofPlatformStatus laststatus)
{
	RenewNowPulse();
	if (lockobj.try_lock())
	{
		config::RecordStatusAndPulse(nullptr, laststatus, NowPluse);
		lockobj.unlock();
	}
	this->disposed = true;
}

// 设置单个电机的速度
void DialogMotionControl::SetMotionVeloctySingle(int index, double velocity)
{
	//ASSERT_INDEX(index);
	velocity = RANGE_V(velocity, -MAX_VEL, MAX_VEL);
	vels[index] = velocity;
	sixdofDioAndCount.SetMotionVel(index, velocity);
}

// 设置多个电机的速度
void DialogMotionControl::SetMotionVelocty(double* velocity, int axexnum)
{
	memmove(vels, velocity, sizeof(double) * AXES_COUNT);
	sixdofDioAndCount.SetMotionVel(velocity);
}
 
// 单缸向上运动
void DialogMotionControl::SingleUp(int index)
{
	SetMotionVeloctySingle(index, RISE_VEL);
}

// 单缸向下运动
void DialogMotionControl::SingleDown(int index)
{
	SetMotionVeloctySingle(index, -DOWN_VEL);
}

// 所有缸向上测试运动
void DialogMotionControl::AllTestUp()
{
	double vel = RISE_VEL;
	double vels[AXES_COUNT] = {vel, vel, vel, vel, vel, vel};
	SetMotionVelocty(vels, AXES_COUNT);
}

// 所有缸向下测试运动
void DialogMotionControl::AllTestDown()
{
	double vel = -DOWN_VEL;
	double vels[AXES_COUNT] = {vel, vel, vel, vel, vel, vel};
	SetMotionVelocty(vels, AXES_COUNT);
}

// 上升
void DialogMotionControl::Rise()
{
	StopRiseDownMove();
	for (int i = 0;i < AXES_COUNT;++i)
	{
		MyPidParaInit(MotionRisePidControler);
	}
	Sleep(10);
	isrising = true;
	Sleep(10);
	isfalling = false;
	enableMove = false;
}

// 下降
void DialogMotionControl::Down()
{
#if IS_PID_DOWN
	Sleep(10);
	isfalling = true;
	isrising = false;
	enableMove = false;
#else
	AllTestDown();
#endif	
}

// 连续位置控制
void DialogMotionControl::Csp(double * pulse)
{	
	for (auto i = 0; i < AXES_COUNT; ++i)
	{
		pulse[i] = RANGE_V(pulse[i], 0, MAX_POS);
		auto re = (pulse[i] - last_pulse[i]) * speed_scale;
		now_vel[i] = re;
		now_vel[i] = RANGE_V(now_vel[i], -4, 4);
		last_pulse[i] = pulse[i];
		last_str_vel[i] = now_vel[i];
	}
	SetMotionVelocty(now_vel, AXES_COUNT);
}

// 连续位置控制(PID控制)
void DialogMotionControl::PidCsp(double * pulse)
{
	auto vel_delta = 0.01;
	if (lockobj.try_lock())
	{	
		for (auto i = 0; i < AXES_COUNT; ++i)
		{
			pulse[i] = pulse[i] + MIDDLE_POS;
			pulse[i] = RANGE_V(pulse[i], HALF_RPM_POS, MAX_POS - HALF_RPM_POS);
			now_vel[i] = MyDeltaPID_Real(&MotionLocationPidControler[i], \
				NowPluse[i], pulse[i]);
			if (now_vel[i] - last_str_vel[i] >= vel_delta){
				now_vel[i] = last_str_vel[i] + vel_delta;
			}
			else if (now_vel[i] - last_str_vel[i] <= vel_delta){
				now_vel[i] = last_str_vel[i] - vel_delta;
			}
			last_str_vel[i] = now_vel[i];
		}
		lockobj.unlock();
	}
	SetMotionVelocty(now_vel, AXES_COUNT);
}


// 缓慢的连续位置控制(PID控制)
void DialogMotionControl::SlowPidCsp(double * pulse)
{
	if (lockobj.try_lock())
	{	
		for (auto i = 0; i < AXES_COUNT; ++i)
		{
			pulse[i] = pulse[i] + MIDDLE_POS;
			pulse[i] = RANGE_V(pulse[i], 0, MAX_POS);
			now_vel[i] = MyDeltaPID_Real(&MotionRisePidControler[i], \
				NowPluse[i], pulse[i]);
		}
		lockobj.unlock();
	}
	SetMotionVelocty(now_vel, AXES_COUNT);
}

// 获取所有电机编码器脉冲的平均值
double DialogMotionControl::GetMotionAveragePulse()
{
	double pulse_num = 0;
	if (lockobj.try_lock())
	{
		for(auto i = 0; i < AXES_COUNT; ++i)
		{
			auto actual_pos = NowPluse[i];
			pulse_num += actual_pos;
		}
		lockobj.unlock();
	}

	double avr_pulse = pulse_num / (double)AXES_COUNT;
	AvrPulse = avr_pulse;
	return avr_pulse;
}

// 更新所有电机编码器读数
void DialogMotionControl::RenewNowPulse()
{
	if(lockobj.try_lock())
	{
		auto pulses = sixdofDioAndCount.ReadVoltageValues();	
		NowPluse[0] = pulses[0];
		NowPluse[1] = pulses[1];
		NowPluse[2] = pulses[2];
		NowPluse[3] = pulses[3];
		NowPluse[4] = pulses[4];
		NowPluse[5] = pulses[5];
		lockobj.unlock();
	}
}

// 获取所有电机编码器读数
void DialogMotionControl::GetNowPulse(double* pulses)
{
	RenewNowPulse();
	if(lockobj.try_lock())
	{
		memmove(pulses, NowPluse, sizeof(double) * AXES_COUNT);
		lockobj.unlock();
	}
}

// 获取中位缸伸长长度
double* DialogMotionControl::GetNowPoleLength()
{
	if (lockobj.try_lock())
	{
		memcpy(polelenthmm, NowPluse, sizeof(double) * AXES_COUNT);
		lockobj.unlock();
	}
	for (int i = 0;i < AXES_COUNT;++i)
	{
		polelenthmm[i] = (polelenthmm[i] - MIDDLE_POS) * PULSE_COUNT_TO_MM_SCALE; 
	}
	return polelenthmm;
}

// 运动学正解获取姿态 x y z roll pitch yaw
double* DialogMotionControl::GetNowPoseFromLength()
{
	if (lockobj.try_lock())
	{
		memcpy(polelenthmm, NowPluse, sizeof(double) * AXES_COUNT);
		lockobj.unlock();
	}
	for (int i = 0;i < AXES_COUNT;++i)
	{
		polelenthmm[i] = (polelenthmm[i] - MIDDLE_POS) * PULSE_COUNT_TO_MM_SCALE; 
	}
	auto pose = FromLengthToPose(polelenthmm);
	memcpy(posefromlength, pose, sizeof(double) * AXES_COUNT);
	return posefromlength;
}

// 电机上升下降的PID控制函数
void DialogMotionControl::DDAControlThread()
{
	while (true)
	{
		double eps = 0.02;
		if (isrising == true)
		{
			RenewNowPulse();
			double pulses[AXES_COUNT] = {0};
			SlowPidCsp(pulses);
			for (int i = 0; i < AXES_COUNT; ++i)
			{
				if (abs(NowPluse[i] - MIDDLE_POS) <= eps)
				{
					//ServoSingleStop(i);
					//LockServo(i);
				}
			}
		}
		if(isfalling == true)
		{
			RenewNowPulse();
			double pulses[AXES_COUNT] = {-MIDDLE_POS,-MIDDLE_POS,-MIDDLE_POS,-MIDDLE_POS,-MIDDLE_POS,-MIDDLE_POS};
			SlowPidCsp(pulses);
			for (int i = 0; i < AXES_COUNT; ++i)
			{
				if (abs(NowPluse[i] - 0) <= 1)
				{
					//ServoSingleStop(i);
					//LockServo(i);
				}
			}
		}
		if(enableMove == true)
		{
			RenewNowPulse();
			double pulses[AXES_COUNT] = {0};
			double sum = 0;
			memcpy(pulses, pos, sizeof(double) * AXES_COUNT);
			SlowPidCsp(pulses);
			for (int i = 0; i < AXES_COUNT; ++i)
			{
				sum += abs(pulses[i] - NowPluse[i]); 
			}
			if (sum <= eps * AXES_COUNT)
			{
				ServoStop();
			}	
		}
		//ReadAllSwitchStatus();
		Sleep(DDA_CONTROL_THREAD_DELAY);
	}
}

// 结束运动后回中
void DialogMotionControl::MoveToZeroPulseNumber()
{
	for (int i = 0;i < AXES_COUNT;++i)
	{
		MyPidParaInit(&MotionRisePidControler[i]);
	}
	Sleep(10);
	isfalling = true;
}

// PID控制器初始化
void DialogMotionControl::PidControllerInit()
{
	for (int i = 0;i < AXES_COUNT;++i)
	{
		MyPidParaInit(&MotionLocationPidControler[i]);
	}
}

// 所有电机停转
bool DialogMotionControl::ServoStop()
{
	StopRiseDownMove();
	return true;
}

// 单个电机停转
bool DialogMotionControl::ServoSingleStop(int index)
{
	enableMove = false;
	double vel = 0;
	SetMotionVeloctySingle(index, vel);
	return true;
}

// 停止上升或者下降的PID控制
void DialogMotionControl::StopRiseDownMove()
{
	enableMove = false;
	isrising = false;
	isfalling = false;
	double vel[AXES_COUNT];
	memset(vel, 0, sizeof(double) * AXES_COUNT);
	SetMotionVelocty(vel, AXES_COUNT);
}

void DialogMotionControl::servoCurveStopThread()
{
	enableMove = false;
	isrising = false;
	isfalling = false;
	double stopvel[AXES_COUNT] = {0};
	double setvel[AXES_COUNT] = {0};
	int totalcount = 200;
	int delay = 5;
	memmove(stopvel, vels, sizeof(double) * AXES_COUNT);
	memmove(setvel, vels, sizeof(double) * AXES_COUNT);
	for (int i = 0;i < totalcount;++i)
	{
		for (int j = 0;j < AXES_COUNT;++j)
		{
			setvel[j] -= stopvel[j] / ((double)totalcount);
		}
		SetMotionVelocty(setvel, AXES_COUNT);
		Sleep(delay);
	}
}

// 所有缸是否位于底部
bool DialogMotionControl::IsAllAtBottom()
{
	static double eps = 0.05;
	auto avr = GetMotionAveragePulse();
	return avr < eps;
}

// 检查六自由度平台状态
bool DialogMotionControl::CheckStatus(SixDofPlatformStatus& status)
{
	char* str = SixDofStatusText[status];
	double pulse = 0;
	switch (status)
	{
	case SIXDOF_STATUS_BOTTOM:
		break;
	case SIXDOF_STATUS_READY:
		break;
	case SIXDOF_STATUS_MIDDLE:
		break;
	case SIXDOF_STATUS_RUN:
		break;
	case SIXDOF_STATUS_ISRISING:
		status = SIXDOF_STATUS_READY;		
		break;
	case SIXDOF_STATUS_ISFALLING:			
		status = SIXDOF_STATUS_BOTTOM;
		break;
	default:
		break;
	}
	str = SixDofStatusText[status];
	Status = status;
	return true;
}

// 六自由度平台开机自检
bool DialogMotionControl::PowerOnSelfTest(SixDofPlatformStatus laststatus, double * lastpulse)
{
	if(isSelfTest == true)
		return false;
	switch (laststatus)
	{
	case SIXDOF_STATUS_BOTTOM:
		AllTestDown();
		break;
	case SIXDOF_STATUS_READY:
		//下降
		AllTestDown();
		break;
	case SIXDOF_STATUS_MIDDLE:
		AllTestDown();
		break;
	case SIXDOF_STATUS_RUN:
		//回中
		//MoveToLocation(lastpulse, AXES_COUNT, false);
		//下降
		AllTestDown();
		break;
	case SIXDOF_STATUS_ISRISING:
		//下降
		AllTestDown();
		break;
	case SIXDOF_STATUS_ISFALLING:
		//下降
		AllTestDown();
		break;
	default:
		break;
	}
	isSelfTest = true;
	return true;
}

// 测试所有硬件
void DialogMotionControl::TestHardware()
{
#if IS_BIG_MOTION
	sixdofDioAndCount.BigMotionTest();
#else
	sixdofDioAndCount.Test();
#endif
}

// 油源启动
void DialogMotionControl::SetOilStart(bool bit)
{
	sixdofDioAndCount.SetOilStart(bit);
}

// 油源停止
void DialogMotionControl::SetOilStop(bool bit)
{
	sixdofDioAndCount.SetOilStop(bit);
}

// 读取是否断电
bool DialogMotionControl::IsNoPower()
{
	bool bit;
	sixdofDioAndCount.IsNoPower(&bit);
	return bit;
}

// 读取是否低压保护
bool DialogMotionControl::IsLowPower()
{
	bool bit = false;
	sixdofDioAndCount.IsLowerPower(&bit);
	return bit;
}

void DialogMotionControl::SetEnable(bool bit)
{
	sixdofDioAndCount.SetEnable(bit);
}

void DialogMotionControl::SetDisable(bool bit)
{
	sixdofDioAndCount.SetDisable(bit);
}

// 将所有输出IO置为低电平
void DialogMotionControl::ResetAllIoPorts()
{
	SetOilStart(false);
	SetOilStop(false);
	SetEnable(false);
	SetDisable(false);
}

// 设置中立位
void DialogMotionControl::setMiddle(double zlw)
{
	MIDDLE_POS = RANGE(zlw, ZERO_POS, MAX_POS);
}


