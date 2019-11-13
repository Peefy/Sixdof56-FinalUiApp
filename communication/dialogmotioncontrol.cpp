
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

// ƽ̨���й���PID���Ʋ���-P
#define MOTION_P 0.44
// ƽ̨���й���PID���Ʋ���-I
#define MOTION_I 0.0002
// ƽ̨���й���PID���Ʋ���-D
#define MOTION_D 0.0
// ƽ̨���й��̵�������ٶ�
#define MAX_VEL  5.0
// ƽ̨��������PID���Ʋ���-P
#define RISE_MOTION_P 0.06
// ƽ̨��������PID���Ʋ���-I
#define RISE_MOTION_I 0.00002
// ƽ̨��������PID���Ʋ���-D
#define RISE_MOTION_D 0.0
// ƽ̨�������̵�������ٶ�
#define RISE_MIN_VEL  0.1
#define RISE_MAX_VEL  0.5

// PID��ʱ����
static double p = 0.0001;
static double i = 0.00001;
static double d = 0.0;

// ��������λ�ÿ���ģʽCsp����ʱ����
static double last_pulse[AXES_COUNT] = { 0, 0, 0, 0, 0, 0 };
static double now_vel[AXES_COUNT] = { 0, 0, 0, 0, 0, 0 };
static double last_str_vel[AXES_COUNT] = { 0, 0, 0, 0, 0, 0 };
static double speed_scale = 100;
static ULONG last_pulses[AXES_COUNT] = {0};

// ���й������������PID������
static PID_Type MotionLocationPidControler[AXES_COUNT] = 
{
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL },
	{ MOTION_P, MOTION_I, MOTION_D, -MAX_VEL, MAX_VEL }
};

// �����������������PID������
static PID_Type MotionRisePidControler[AXES_COUNT] = 
{
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL },
	{ RISE_MOTION_P, RISE_MOTION_I, RISE_MOTION_D, -RISE_MAX_VEL, RISE_MAX_VEL }
};

// ���캯��
DialogMotionControl::DialogMotionControl()
{
	InitData();
	ReadParaFromFile();
	InitCard();
}

// ��������
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

// �������ļ��ж�ȡPID����
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

// ��ʼ������Ӳ���忨
bool DialogMotionControl::InitCard()
{
	return sixdofDioAndCount.Init();
}

// �ر����а忨��������ƽ̨״̬
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

// ���õ���������ٶ�
void DialogMotionControl::SetMotionVeloctySingle(int index, double velocity)
{
	//ASSERT_INDEX(index);
	velocity = RANGE_V(velocity, -MAX_VEL, MAX_VEL);
	vels[index] = velocity;
	sixdofDioAndCount.SetMotionVel(index, velocity);
}

// ���ö��������ٶ�
void DialogMotionControl::SetMotionVelocty(double* velocity, int axexnum)
{
	memmove(vels, velocity, sizeof(double) * AXES_COUNT);
	sixdofDioAndCount.SetMotionVel(velocity);
}
 
// ���������˶�
void DialogMotionControl::SingleUp(int index)
{
	SetMotionVeloctySingle(index, RISE_VEL);
}

// ���������˶�
void DialogMotionControl::SingleDown(int index)
{
	SetMotionVeloctySingle(index, -DOWN_VEL);
}

// ���и����ϲ����˶�
void DialogMotionControl::AllTestUp()
{
	double vel = RISE_VEL;
	double vels[AXES_COUNT] = {vel, vel, vel, vel, vel, vel};
	SetMotionVelocty(vels, AXES_COUNT);
}

// ���и����²����˶�
void DialogMotionControl::AllTestDown()
{
	double vel = -DOWN_VEL;
	double vels[AXES_COUNT] = {vel, vel, vel, vel, vel, vel};
	SetMotionVelocty(vels, AXES_COUNT);
}

// ����
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

// �½�
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

// ����λ�ÿ���
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

// ����λ�ÿ���(PID����)
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


// ����������λ�ÿ���(PID����)
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

// ��ȡ���е�������������ƽ��ֵ
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

// �������е������������
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

// ��ȡ���е������������
void DialogMotionControl::GetNowPulse(double* pulses)
{
	RenewNowPulse();
	if(lockobj.try_lock())
	{
		memmove(pulses, NowPluse, sizeof(double) * AXES_COUNT);
		lockobj.unlock();
	}
}

// ��ȡ��λ���쳤����
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

// �˶�ѧ�����ȡ��̬ x y z roll pitch yaw
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

// ��������½���PID���ƺ���
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

// �����˶������
void DialogMotionControl::MoveToZeroPulseNumber()
{
	for (int i = 0;i < AXES_COUNT;++i)
	{
		MyPidParaInit(&MotionRisePidControler[i]);
	}
	Sleep(10);
	isfalling = true;
}

// PID��������ʼ��
void DialogMotionControl::PidControllerInit()
{
	for (int i = 0;i < AXES_COUNT;++i)
	{
		MyPidParaInit(&MotionLocationPidControler[i]);
	}
}

// ���е��ͣת
bool DialogMotionControl::ServoStop()
{
	StopRiseDownMove();
	return true;
}

// �������ͣת
bool DialogMotionControl::ServoSingleStop(int index)
{
	enableMove = false;
	double vel = 0;
	SetMotionVeloctySingle(index, vel);
	return true;
}

// ֹͣ���������½���PID����
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

// ���и��Ƿ�λ�ڵײ�
bool DialogMotionControl::IsAllAtBottom()
{
	static double eps = 0.05;
	auto avr = GetMotionAveragePulse();
	return avr < eps;
}

// ��������ɶ�ƽ̨״̬
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

// �����ɶ�ƽ̨�����Լ�
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
		//�½�
		AllTestDown();
		break;
	case SIXDOF_STATUS_MIDDLE:
		AllTestDown();
		break;
	case SIXDOF_STATUS_RUN:
		//����
		//MoveToLocation(lastpulse, AXES_COUNT, false);
		//�½�
		AllTestDown();
		break;
	case SIXDOF_STATUS_ISRISING:
		//�½�
		AllTestDown();
		break;
	case SIXDOF_STATUS_ISFALLING:
		//�½�
		AllTestDown();
		break;
	default:
		break;
	}
	isSelfTest = true;
	return true;
}

// ��������Ӳ��
void DialogMotionControl::TestHardware()
{
#if IS_BIG_MOTION
	sixdofDioAndCount.BigMotionTest();
#else
	sixdofDioAndCount.Test();
#endif
}

// ��Դ����
void DialogMotionControl::SetOilStart(bool bit)
{
	sixdofDioAndCount.SetOilStart(bit);
}

// ��Դֹͣ
void DialogMotionControl::SetOilStop(bool bit)
{
	sixdofDioAndCount.SetOilStop(bit);
}

// ��ȡ�Ƿ�ϵ�
bool DialogMotionControl::IsNoPower()
{
	bool bit;
	sixdofDioAndCount.IsNoPower(&bit);
	return bit;
}

// ��ȡ�Ƿ��ѹ����
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

// ���������IO��Ϊ�͵�ƽ
void DialogMotionControl::ResetAllIoPorts()
{
	SetOilStart(false);
	SetOilStop(false);
	SetEnable(false);
	SetDisable(false);
}

// ��������λ
void DialogMotionControl::setMiddle(double zlw)
{
	MIDDLE_POS = RANGE(zlw, ZERO_POS, MAX_POS);
}


