
#ifndef _DIALOG_MOTION_CONTROL_H_
#define _DIALOG_MOTION_CONTROL_H_

#include <memory>
#include <vector>
#include <deque>
// �߳�ͬ����ͷ�ļ�
#include <mutex>   
// �����ɶ�ƽ̨ͷ�ļ�
#include "sixdof.h"
// ���ư忨ͷ�ļ�
#include "../hardware/SixdofDioAndPulseCount.h"
// Ӧ������ͷ�ļ�
#include "../config/appconfig.h"

using namespace std;

//�����½����Ƽ��
#define DDA_CONTROL_THREAD_DELAY 5

// �׵�����г�mm
#define MAX_MM 1500.0
// ��ƽ̨�ϱ��������ƽ̨�����Ĵ�ֱ����mm
#define PlaneAboveHingeLength       590.0
// ��ƽ̨�ϱ���������Ĵ�ֱ����mm
#define PlaneAboveBottomLength      3695.0
// ��ƽ̨ԲȦ�뾶mm
#define CircleTopRadius             1700.0
// ��ƽ̨ԲȦ�뾶mm
#define CircleBottomRadius          2400.025
// ��ƽ̨ͬһ���������������ľ���mm
#define DistanceBetweenHingeTop     330.078
// ��ƽ̨ͬһ���������������ľ���mm
#define DistanceBetweenHingeBottom  330.181

// ��λV
#define RISE_VEL 0.5
// ��λV
#define DOWN_VEL 0.5

// ƽ̨�˶����Ƕ�deg
#define MAX_DEG 30
// �Ƕȱ�Ϊ��������ϵ��
#define DEG_SCALE 0.01
// ƽ̨�˶����λ��mm
#define MAX_XYZ 800
// λ�Ʊ�Ϊ��������ϵ��
#define XYZ_SCALE 0.1
// ƽ̨�˶����Ƶ��Hz
#define MAX_HZ 5
// ƽ̨�����˶������λdeg
#define MAX_PHASE 360

// ƽ̨�˶����λ�� ����mm
#define MAX_XYZ_X      800
// ƽ̨�˶����λ�� ����mm
#define MAX_XYZ_Y      800
// ƽ̨�˶����λ�� ����mm
#define MAX_XYZ_Z      800
// ƽ̨�˶����Ƕ� ������deg
#define MAX_DEG_PITCH  30
// ƽ̨�˶����Ƕ� �����deg
#define MAX_DEG_ROLL   30
// ƽ̨�˶����Ƕ� ƫ����deg
#define MAX_DEG_YAW    30
// ƽ̨�����˶� ����λ�������λmm
#define MAX_XYZ_ZERO_POS_X        (MAX_XYZ_X)
// ƽ̨�����˶� ����λ�������λmm
#define MAX_XYZ_ZERO_POS_Y        (MAX_XYZ_Y)
// ƽ̨�����˶� ����λ�������λmm
#define MAX_XYZ_ZERO_POS_Z        (MAX_XYZ_Z)
// ƽ̨�����˶� �����Ƕ������λdeg
#define MAX_DEG_ZERO_POS_PITCH    (MAX_DEG_PITCH)
// ƽ̨�����˶� ����������λdeg
#define MAX_DEG_ZERO_POS_ROLL     (MAX_DEG_ROLL)
// ƽ̨�����˶� ƫ���Ƕ������λdeg
#define MAX_DEG_ZERO_POS_YAW      (MAX_DEG_YAW) 

// ������г�(�忨������)
#define MAX_POS 5.0
// ����λ�г�(�忨��λ����)
//double MIDDLE_POS=2.4
// ������г�(�忨������)
#define ZERO_POS 0.0
// �׵����Ȧ�г�(����������)
//#define HALF_RPM_POS (ZERO_POS + PULSE_COUNT_RPM / 2.0)
#define HALF_RPM_POS 0.2
// ������λʱ�˶������λ��
//#define MAX_POLE_LENGTH (MAX_MM / 2.0)
#define MAX_POLE_LENGTH 0.0
// ���쳤��mm��������λ�õ�ת��ϵ��
#define MM_TO_PULSE_COUNT_SCALE (MAX_POS / MAX_MM)
// ������λ�õ����쳤��mm��ת��ϵ��
#define PULSE_COUNT_TO_MM_SCALE (MAX_MM / MAX_POS)
// �Ƿ����PID����ƽ̨�½���0Ϊ��1Ϊ�ǣ�Ĭ��Ϊ�񼴿�
#define IS_PID_DOWN 0

using namespace std;

class DialogMotionControl
{
public:
	// ���캯��
	DialogMotionControl();
	// ��������
	~DialogMotionControl();
	// ��ʼ������
	void InitData();
	// ��ʼ������Ӳ���忨
	bool InitCard();
	// �������ļ��ж�ȡPID����
	void ReadParaFromFile();
	// �ر����а忨��������ƽ̨״̬
	void Close(SixDofPlatformStatus laststatus);
	// ���õ���������ٶ�
	void SetMotionVeloctySingle(int index, double velocity);
	// ���ö��������ٶ�
	void SetMotionVelocty(double* velocity, int axexnum);
	// ���������˶�
	void SingleUp(int index);
	// ���������˶�
	void SingleDown(int index);
	// ���и����ϲ����˶�
	void AllTestUp();
	// ���и����²����˶�
	void AllTestDown();
	// �����˶������
	void MoveToZeroPulseNumber();
	// PID��������ʼ��
	void PidControllerInit();
	// ���е��ͣת
	bool ServoStop();
	// �������ͣת
	bool ServoSingleStop(int index);
	// ֹͣ���������½���PID����
	void StopRiseDownMove();
	// ����
	void Rise();
	// �½�
	void Down();
	// ����λ�ÿ���
	void Csp(double * pulse);
	// ����λ�ÿ���(PID����)
	void PidCsp(double * pulse);
	// ����������λ�ÿ���(PID����)
	void SlowPidCsp(double * pulse);
	// ��ȡ���е�������������ƽ��ֵ
	double GetMotionAveragePulse();
	// �������е������������
	void RenewNowPulse();
	// ��ȡ���е������������
	void GetNowPulse(double* pulses);
	// ��ȡ��λ���쳤����
	double* GetNowPoleLength();
	// �˶�ѧ�����ȡ��̬
	double* GetNowPoseFromLength();
	// ��������½���PID���ƺ���
	void DDAControlThread();
	// ���и��Ƿ�λ�ڵײ�
	bool IsAllAtBottom();
	// ��������ɶ�ƽ̨״̬
	bool CheckStatus(SixDofPlatformStatus& status);
	// �����ɶ�ƽ̨�����Լ�
	bool PowerOnSelfTest(SixDofPlatformStatus laststatus, double * lastpulse);
	// ��������Ӳ��
	void TestHardware();
	// ��Դ����
	void SetOilStart(bool bit);
	// ��Դֹͣ
	void SetOilStop(bool bit);
	// ϵͳʹ��
	void SetEnable(bool bit);
	// ϵͳж��
	void SetDisable(bool bit);
	// ��ȡ�Ƿ�ϵ�
	bool IsNoPower();
	// ��ȡ�Ƿ��ѹ����
	bool IsLowPower();
	// ���������IO��Ϊ�͵�ƽ
	void ResetAllIoPorts();
public:
	// ���������λ��
	double NowPluse[AXES_COUNT];
	//// ����λ�г�(��������λ����)
	double MIDDLE_POS;
	// ���������ƽ��λ��
	double AvrPulse;
	// �����ɶ�ƽ̨״̬
	SixDofPlatformStatus Status;
private:
	// �Ƿ������½�
	bool isrising;
	// �Ƿ���������
	bool isfalling;
	// �Ƿ񿪻��Լ��
	bool isSelfTest;
	// �Ƿ�������ת��
	bool enableMove;
	// �Ƿ��ͷ�������Դ
	bool disposed;
	// ���λ�û������
	double pos[AXES_COUNT];
	// ����λ�쳤���Ȼ������
	double polelenthmm[AXES_COUNT];
	// �˶�ѧ���⻺�����
	double posefromlength[AXES_COUNT];
	// ��������λ
	void setMiddle(double zlw);
	// �忨������
	SixdofDioAndCount sixdofDioAndCount;
protected:
	// �߳�ͬ����
	mutex lockobj;

};



#endif
