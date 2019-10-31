
#ifndef _DIALOG_MOTION_CONTROL_H_
#define _DIALOG_MOTION_CONTROL_H_

#include <memory>
#include <vector>
#include <deque>
// 线程同步锁头文件
#include <mutex>   
// 六自由度平台头文件
#include "sixdof.h"
// 控制板卡头文件
#include "../hardware/SixdofDioAndPulseCount.h"
// 应用配置头文件
#include "../config/appconfig.h"

using namespace std;

//上升下降控制间隔
#define DDA_CONTROL_THREAD_DELAY 5

// 缸的最大行程mm
#define MAX_MM 1500.0
// 上平台上表面距离上平台铰链的垂直距离mm
#define PlaneAboveHingeLength       590.0
// 上平台上表面距离地面的垂直距离mm
#define PlaneAboveBottomLength      3695.0
// 上平台圆圈半径mm
#define CircleTopRadius             1700.0
// 下平台圆圈半径mm
#define CircleBottomRadius          2400.025
// 上平台同一组两个铰链的中心距离mm
#define DistanceBetweenHingeTop     330.078
// 下平台同一组两个铰链的中心距离mm
#define DistanceBetweenHingeBottom  330.181

// 单位V
#define RISE_VEL 0.5
// 单位V
#define DOWN_VEL 0.5

// 平台运动最大角度deg
#define MAX_DEG 30
// 角度变为整数缩放系数
#define DEG_SCALE 0.01
// 平台运动最大位移mm
#define MAX_XYZ 800
// 位移变为整数缩放系数
#define XYZ_SCALE 0.1
// 平台运动最大频率Hz
#define MAX_HZ 5
// 平台正弦运动最大相位deg
#define MAX_PHASE 360

// 平台运动最大位移 纵向mm
#define MAX_XYZ_X      800
// 平台运动最大位移 横向mm
#define MAX_XYZ_Y      800
// 平台运动最大位移 垂向mm
#define MAX_XYZ_Z      800
// 平台运动最大角度 俯仰角deg
#define MAX_DEG_PITCH  30
// 平台运动最大角度 横滚角deg
#define MAX_DEG_ROLL   30
// 平台运动最大角度 偏航角deg
#define MAX_DEG_YAW    30
// 平台正弦运动 纵向位移最大零位mm
#define MAX_XYZ_ZERO_POS_X        (MAX_XYZ_X)
// 平台正弦运动 纵向位移最大零位mm
#define MAX_XYZ_ZERO_POS_Y        (MAX_XYZ_Y)
// 平台正弦运动 纵向位移最大零位mm
#define MAX_XYZ_ZERO_POS_Z        (MAX_XYZ_Z)
// 平台正弦运动 俯仰角度最大零位deg
#define MAX_DEG_ZERO_POS_PITCH    (MAX_DEG_PITCH)
// 平台正弦运动 横滚角最大零位deg
#define MAX_DEG_ZERO_POS_ROLL     (MAX_DEG_ROLL)
// 平台正弦运动 偏航角度最大零位deg
#define MAX_DEG_ZERO_POS_YAW      (MAX_DEG_YAW) 

// 缸最大行程(板卡最大读数)
#define MAX_POS 5.0
// 缸中位行程(板卡中位读数)
//double MIDDLE_POS=2.4
// 缸零点行程(板卡零点读数)
#define ZERO_POS 0.0
// 缸电机半圈行程(编码器读数)
//#define HALF_RPM_POS (ZERO_POS + PULSE_COUNT_RPM / 2.0)
#define HALF_RPM_POS 0.2
// 缸在中位时运动的最大位移
//#define MAX_POLE_LENGTH (MAX_MM / 2.0)
#define MAX_POLE_LENGTH 0.0
// 缸伸长量mm到编码器位置的转换系数
#define MM_TO_PULSE_COUNT_SCALE (MAX_POS / MAX_MM)
// 编码器位置到缸伸长量mm的转换系数
#define PULSE_COUNT_TO_MM_SCALE (MAX_MM / MAX_POS)
// 是否采用PID控制平台下降，0为否，1为是，默认为否即可
#define IS_PID_DOWN 0

using namespace std;

class DialogMotionControl
{
public:
	// 构造函数
	DialogMotionControl();
	// 析构函数
	~DialogMotionControl();
	// 初始化数据
	void InitData();
	// 初始化所有硬件板卡
	bool InitCard();
	// 从配置文件中读取PID参数
	void ReadParaFromFile();
	// 关闭所有板卡，并保存平台状态
	void Close(SixDofPlatformStatus laststatus);
	// 设置单个电机的速度
	void SetMotionVeloctySingle(int index, double velocity);
	// 设置多个电机的速度
	void SetMotionVelocty(double* velocity, int axexnum);
	// 单缸向上运动
	void SingleUp(int index);
	// 单缸向下运动
	void SingleDown(int index);
	// 所有缸向上测试运动
	void AllTestUp();
	// 所有缸向下测试运动
	void AllTestDown();
	// 结束运动后回中
	void MoveToZeroPulseNumber();
	// PID控制器初始化
	void PidControllerInit();
	// 所有电机停转
	bool ServoStop();
	// 单个电机停转
	bool ServoSingleStop(int index);
	// 停止上升或者下降的PID控制
	void StopRiseDownMove();
	// 上升
	void Rise();
	// 下降
	void Down();
	// 连续位置控制
	void Csp(double * pulse);
	// 连续位置控制(PID控制)
	void PidCsp(double * pulse);
	// 缓慢的连续位置控制(PID控制)
	void SlowPidCsp(double * pulse);
	// 获取所有电机编码器脉冲的平均值
	double GetMotionAveragePulse();
	// 更新所有电机编码器读数
	void RenewNowPulse();
	// 获取所有电机编码器读数
	void GetNowPulse(double* pulses);
	// 获取中位缸伸长长度
	double* GetNowPoleLength();
	// 运动学正解获取姿态
	double* GetNowPoseFromLength();
	// 电机上升下降的PID控制函数
	void DDAControlThread();
	// 所有缸是否位于底部
	bool IsAllAtBottom();
	// 检查六自由度平台状态
	bool CheckStatus(SixDofPlatformStatus& status);
	// 六自由度平台开机自检
	bool PowerOnSelfTest(SixDofPlatformStatus laststatus, double * lastpulse);
	// 测试所有硬件
	void TestHardware();
	// 油源启动
	void SetOilStart(bool bit);
	// 油源停止
	void SetOilStop(bool bit);
	// 系统使能
	void SetEnable(bool bit);
	// 系统卸荷
	void SetDisable(bool bit);
	// 读取是否断电
	bool IsNoPower();
	// 读取是否低压保护
	bool IsLowPower();
	// 将所有输出IO置为低电平
	void ResetAllIoPorts();
public:
	// 电机编码器位置
	double NowPluse[AXES_COUNT];
	//// 缸中位行程(编码器中位读数)
	double MIDDLE_POS;
	// 电机编码器平均位置
	double AvrPulse;
	// 六自由度平台状态
	SixDofPlatformStatus Status;
private:
	// 是否正在下降
	bool isrising;
	// 是否正在上升
	bool isfalling;
	// 是否开机自检过
	bool isSelfTest;
	// 是否允许电机转动
	bool enableMove;
	// 是否释放所有资源
	bool disposed;
	// 电机位置缓冲变量
	double pos[AXES_COUNT];
	// 缸中位伸长长度缓冲变量
	double polelenthmm[AXES_COUNT];
	// 运动学正解缓冲变量
	double posefromlength[AXES_COUNT];
	// 设置中立位
	void setMiddle(double zlw);
	// 板卡控制类
	SixdofDioAndCount sixdofDioAndCount;
protected:
	// 线程同步锁
	mutex lockobj;

};



#endif
