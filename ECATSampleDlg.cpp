
#include "StdAfx.h"
#include "ECATSample.h"
#include "ECATSampleDlg.h"

#include <math.h>

#include <fstream>
#include <deque>
#include <thread>

#include "DialogRegister.h"

#include "chart.h"

#include "Sixdofdll2010.h"

#include "communication/sixdof.h"
#include "communication/dialogmotioncontrol.h"
#include "communication/SerialPort.h"

#include "control/sensor.h"
#include "control/pid.h"
#include "control/kalman_filter.h"
#include "control/inertialnavigation.h"
#include "control/water.h"
#include "control/illusion.h"
#include "control/platformapi.h"

#include "config/recordpath.h"
#include "ui/uiconfig.h"
#include "signal/roadspectrum.h"
#include "util/model.h"
#include "register/register.h"
#include "hardware/SixdofDioAndPulseCount.h"

#include "glut.h"
#include "opengl/sixdofopenglhelper.h"

#include "cwnds/panel.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

using namespace std;

#define COLOR_RED     RGB(255, 123, 123)
#define COLOR_GREEN   RGB(123, 255, 123)

// 定时器周期
#define TIMER_MS 10

// 六自由度平台控制周期
#define SIXDOF_CONTROL_DELEY 1
#define SCENE_THREAD_DELAY 1000
#define SENSOR_THREAD_DELAY 1000
#define DATA_BUFFER_THREAD_DELAY 1000

// 平滑启动时间
#define CHIRP_TIME       2.5
// 到达姿态零偏时间
#define ZERO_POS_TIME    2.5
#define ENABLE_CHIRP  false
#define ENABLE_SHOCK  false
#define ENABLE_ZERO_POS true

// 是否显示对话框
#define IS_USE_MESSAGE_BOX 1

bool enableShock = ENABLE_SHOCK;
bool enableChirp = ENABLE_CHIRP;
bool enableZeroPos = ENABLE_ZERO_POS;

void SixdofControl();

bool closeDataThread = true;
bool stopAndMiddle = true;

volatile HANDLE DataThread;
volatile HANDLE SensorThread;
volatile HANDLE SceneThread;
volatile HANDLE DataBufferThread;
volatile HANDLE PlatInitThread;  
volatile HANDLE PlatCloseThread;

// 六自由度平台逻辑控制
DialogMotionControl delta;
// 六自由度数据
DataPackage data = {0};

// 六自由度平台状态
double pulse_cal[AXES_COUNT];
double poleLength[AXES_COUNT];
double lastStartPulse[AXES_COUNT];
SixDofPlatformStatus status = SIXDOF_STATUS_BOTTOM;
SixDofPlatformStatus lastStartStatus = SIXDOF_STATUS_BOTTOM;
Signal::RoadSpectrum roadSpectrum;

// 图表
CChartCtrl m_ChartCtrl1; 
CChartLineSerie *pLineSerie1;
CChartLineSerie *pLineSerie2;
CChartLineSerie *pLineSerie3;
CChartLineSerie *pLineSerie4;
CChartLineSerie *pLineSerie5;
CChartLineSerie *pLineSerie6;

#define DATA_COL_NUM 18
#define MAX_REPRODUCE_LINE 1000
double SourceBuf[DATA_COL_NUM] = {0};

bool isTest = true;
bool isCosMode = false;
bool stopSCurve = false;

double testVal[FREEDOM_NUM] = { 0 };
double testHz[FREEDOM_NUM] = { 0 };
double testPhase[FREEDOM_NUM] = { 0 };
double testZeroPos[FREEDOM_NUM] = { 0 };

double chartBottomAxisPoint[CHART_POINT_NUM] = { 0 };
double chartXValPoint[CHART_POINT_NUM] = { 0 };
double chartYValPoint[CHART_POINT_NUM] = { 0 };
double chartZValPoint[CHART_POINT_NUM] = { 0 };
double chartRollValPoint[CHART_POINT_NUM] = { 0 };
double chartPitchValPoint[CHART_POINT_NUM] = { 0 };
double chartYawValPoint[CHART_POINT_NUM] = { 0 };

double t = 0;
double runTime = 0;
double chartTime = 0;

mutex csdata;
mutex ctrlCommandLockobj;
DataPackageDouble visionData = {0};
DataPackageDouble lastData = {0};
config::FileRecord fileSave;
double cardVols[AXES_COUNT] = {0};
double cardPoleLength[AXES_COUNT] = {0};

int startFlag = 0;
int finishFlag = 0;
int fileCount = 0;

DWORD WINAPI DataTransThread(LPVOID pParam)
{
	while (true)
	{
		SixdofControl();
	}
	return 0;
}

DWORD WINAPI SensorInfoThread(LPVOID pParam)
{
	while (true)
	{
		Sleep(SENSOR_THREAD_DELAY);
	}
	return 0;
}

DWORD WINAPI SceneInfoThread(LPVOID pParam)
{
	while (true)
	{	
		Sleep(SCENE_THREAD_DELAY);
	}
	return 0;
}


DWORD WINAPI DataBufferInfoThread(LPVOID pParam)
{
	// 上升、下降、中立位的逻辑控制
	delta.DDAControlThread();
	return 0;
}

void PlatformInitThread()
{
	startFlag = 0;
	delta.SetOilStart(true);
	Sleep(5000);
	delta.SetOilStart(false);
	Sleep(50);
	delta.SetEnable(true);
	delta.AllTestDown();
	Sleep(10000);
	startFlag = 1;
	Sleep(50000);
	if (startFlag != 3)
		delta.SetDisable(false);
	startFlag = 2;
}

void PlatformCloseThread()
{
	finishFlag = 0;
	delta.SetOilStop(true);
	Sleep(5000);
	delta.SetOilStop(false);
	Sleep(50);
	delta.SetDisable(true);
	delta.AllTestDown();
	finishFlag = 1;
	Sleep(60000);
	delta.SetEnable(false);
	Sleep(5);
	delta.SetDisable(false);
	Sleep(5);
	finishFlag = 2;
}

void CECATSampleDlg::PlatTimerFunc()
{
	if (startFlag == 1 && delta.IsLowPower() == true)
	{
		startFlag = 3;
		MessageBox(_T("低压报警！"));
		OnBnClickedOk();
	}
	else if (startFlag == 2)
	{
		startFlag = 3;
		EanbleButton(1);
	}
	if (finishFlag == 2)
	{
		finishFlag = 3;
		Sleep(10);
		CloseThread();
		CDialog::OnOK();
	}
}

// 释放所有线程资源
void CECATSampleDlg::CloseThread()
{
	try
	{
		if (DataThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(DataThread);
			DataThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (SensorThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(SensorThread);
			SensorThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (SceneThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(SceneThread);
			SceneThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (DataBufferThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(DataBufferThread);
			DataBufferThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (PlatInitThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(PlatInitThread);
			PlatInitThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
	try
	{
		if (PlatCloseThread != INVALID_HANDLE_VALUE)
		{
			CloseHandle(PlatCloseThread);
			PlatCloseThread = INVALID_HANDLE_VALUE;
		}
	}
	catch (CException* e)
	{
		printf("%d\r\n", e->ReportError());
	}
}

void OpenThread()
{
	DataThread = (HANDLE)CreateThread(NULL, 0, DataTransThread, NULL, 0, NULL);
	DataBufferThread = (HANDLE)CreateThread(NULL, 0, DataBufferInfoThread, NULL, 0, NULL);
	//SensorThread = (HANDLE)CreateThread(NULL, 0, SensorInfoThread, NULL, 0, NULL);
	//SceneThread = (HANDLE)CreateThread(NULL, 0, SceneInfoThread, NULL, 0, NULL);	
	//TimerThread = (HANDLE)CreateThread(NULL, 0, TimerInfoThread, NULL, 0, NULL);
}

double vision_x = 0;
double vision_y = 0;
double vision_z = 0;
double vision_roll = 0;
double vision_pitch = 0;
double vision_yaw = 0;

void SixdofControl()
{
	static double deltat = 0.026;
	Sleep(10);
	if(closeDataThread == false)
	{	
		DWORD start_time = 0;
		start_time = GetTickCount();
		delta.RenewNowPulse();
		auto delay = SIXDOF_CONTROL_DELEY;
		double dis[AXES_COUNT] = {0};
		// 正弦测试运动
		if (isTest == true)
		{
			double nowHz[AXES_COUNT] = {testHz[0], testHz[1], testHz[2], testHz[3], testHz[4], testHz[5]};
			double chirpTime = CHIRP_TIME;
			double zeroposTime = ZERO_POS_TIME;
			double pi = 3.1415926;
			double nowt = t;
			double x = 0;
			double y = 0;
			double z = 0;
			double roll = 0;
			double pitch = 0;
			double yaw = 0;
			double stopTime = 0;
			// 开始正弦运动时加一个chrip信号缓冲
			if (t <= chirpTime && enableChirp == true)
			{
				for (auto i = 0; i < AXES_COUNT; ++i)
				{
					nowHz[i] = t * testHz[i] / chirpTime / chirpTime; 
				}
				x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
			}		
			else if (enableChirp == true)
			{
				nowt = t - chirpTime;
				x = sin(2 * pi * nowHz[0] * nowt + 2 * pi * testHz[0]) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowt + 2 * pi * testHz[1]) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowt + 2 * pi * testHz[2]) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowt + 2 * pi * testHz[3]) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowt + 2 * pi * testHz[4]) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowt + 2 * pi * testHz[5]) * testVal[5];
			}
			else if(isCosMode == false)
			{
				x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
			}
			else
			{

			}
			if (isCosMode = true)
			{
				double nowphase_t[AXES_COUNT] = {t,t,t,t,t,t};
				for (int i = 0;i < AXES_COUNT; ++i)
				{
					if (nowHz[i] != 0)
					{
						auto phase_t = t - 1.0 / nowHz[i] * (testPhase[i] / 360.0);
						nowphase_t[i] = DOWN_RANGE(phase_t, 0);
					}			
					else
					{
						nowphase_t[i] = t;
					}					
				}
				x = sin(2 * pi * nowHz[0] * nowphase_t[0]) * testVal[0];
				y = sin(2 * pi * nowHz[1] * nowphase_t[1]) * testVal[1];
				z = sin(2 * pi * nowHz[2] * nowphase_t[2]) * testVal[2];
				roll = sin(2 * pi * nowHz[3] * nowphase_t[3]) * testVal[3];
				pitch = sin(2 * pi * nowHz[4] * nowphase_t[4]) * testVal[4];
				yaw = sin(2 * pi * nowHz[5] * nowphase_t[5]) * testVal[5];
			}
			if (enableZeroPos == true)
			{
				auto minZeroPosTime = min(t, zeroposTime) / zeroposTime;
				x += testZeroPos[0] * minZeroPosTime;
				y += testZeroPos[1] * minZeroPosTime;
				z += testZeroPos[2] * minZeroPosTime;
				roll += testZeroPos[3] * minZeroPosTime;
				pitch += testZeroPos[4] * minZeroPosTime;
				yaw += testZeroPos[5] * minZeroPosTime;
			}
			data.X = (int16_t)(x * 10);
			data.Y = (int16_t)(y * 10);
			data.Z = (int16_t)(z * 10);
			data.Roll = (int16_t)(roll * 100);
			data.Pitch = (int16_t)(pitch * 100);
			data.Yaw = (int16_t)(yaw * 100);
			double* pulse_dugu = Control(x, y, z, roll, yaw, pitch);
			for (auto ii = 0; ii < AXES_COUNT; ++ii)
			{
				pulse_cal[ii] = pulse_dugu[ii];
				poleLength[ii] = pulse_dugu[ii];
				pulse_cal[ii] *= MM_TO_PULSE_COUNT_SCALE;
				auto pulse = pulse_cal[ii];
				dis[ii] = pulse;
			}
			if (status != SIXDOF_STATUS_PAUSING)
				t += deltat;
			delta.PidCsp(dis);
		}
		// 模拟船视景
		else
		{
			if (roadSpectrum.DataBuffer.size() > 0) {
				auto roaddata = roadSpectrum.DataBuffer.front();
				roadSpectrum.DataBuffer.pop_front();
				vision_x = roaddata.Position.X;
				vision_y = roaddata.Position.Y;
				vision_z = roaddata.Position.Z;
				vision_roll = roaddata.Position.Roll;
				vision_pitch = roaddata.Position.Pitch;
				vision_yaw = roaddata.Position.Yaw;
			}
			else {			
				if (status != SIXDOF_STATUS_RUN) {
					t = 0;
					closeDataThread = true;	
				}
			}
			auto x = RANGE(vision_x, -MAX_XYZ, MAX_XYZ);
			auto y = RANGE(vision_y, -MAX_XYZ, MAX_XYZ);
			auto z = RANGE(vision_z, -MAX_XYZ, MAX_XYZ);
			auto roll = RANGE(vision_roll, -MAX_DEG, MAX_DEG);
			auto pitch = RANGE(vision_pitch, -MAX_DEG, MAX_DEG);
			auto yaw = RANGE(vision_yaw, -MAX_DEG, MAX_DEG);

			double* pulse_dugu = Control(x, y, z, roll, yaw, pitch);
			lastData.X = x;
			lastData.Y = y;
			lastData.Z = z;
			lastData.Roll = roll;
			lastData.Pitch = pitch;
			lastData.Yaw = yaw;
			for (auto ii = 0; ii < AXES_COUNT; ++ii)
			{
				pulse_cal[ii] = pulse_dugu[ii];
				poleLength[ii] = pulse_dugu[ii];
				pulse_cal[ii] *= MM_TO_PULSE_COUNT_SCALE;
				auto pulse = pulse_cal[ii];
				dis[ii] = pulse;
			}
			data.X = (int16_t)(x * 10);
			data.Y = (int16_t)(y * 10);
			data.Z = (int16_t)(z * 10);
			data.Roll = (int16_t)(roll * 100);
			data.Yaw = (int16_t)(yaw * 100);
			data.Pitch = (int16_t)(pitch * 100);
			if (status != SIXDOF_STATUS_PAUSING)
				t += deltat;
			delta.PidCsp(dis);
		}
		Sleep(delay);
		DWORD end_time = GetTickCount();
		runTime = end_time - start_time;
	}
}

void MoveValPoint()
{
	chartTime += 0.067;
	for (auto i = 0; i < CHART_POINT_NUM; ++i)
	{
		chartBottomAxisPoint[i] = chartBottomAxisPoint[i + 1];

		chartXValPoint[i] = chartXValPoint[i + 1];
		chartYValPoint[i] = chartYValPoint[i + 1];
		chartZValPoint[i] = chartZValPoint[i + 1];
		chartRollValPoint[i] = chartRollValPoint[i + 1];
		chartPitchValPoint[i] = chartPitchValPoint[i + 1];
		chartYawValPoint[i] = chartYawValPoint[i + 1];
	}

	chartBottomAxisPoint[CHART_POINT_NUM - 1] = chartTime;
	chartXValPoint[CHART_POINT_NUM - 1] = data.X * XYZ_SCALE;
	chartYValPoint[CHART_POINT_NUM - 1] = data.Y * XYZ_SCALE; 
	chartZValPoint[CHART_POINT_NUM - 1] = data.Z * XYZ_SCALE;
	chartRollValPoint[CHART_POINT_NUM - 1] = data.Roll * DEG_SCALE;
	chartPitchValPoint[CHART_POINT_NUM - 1] = data.Pitch * DEG_SCALE;
	chartYawValPoint[CHART_POINT_NUM - 1] = data.Yaw * DEG_SCALE;

	m_ChartCtrl1.EnableRefresh(false);

	pLineSerie1->ClearSerie();
	pLineSerie2->ClearSerie();
	pLineSerie3->ClearSerie();
	pLineSerie4->ClearSerie();
	pLineSerie5->ClearSerie();
	pLineSerie6->ClearSerie();

	pLineSerie1->AddPoints(chartBottomAxisPoint, chartXValPoint, CHART_POINT_NUM);
	pLineSerie2->AddPoints(chartBottomAxisPoint, chartYValPoint, CHART_POINT_NUM);
	pLineSerie3->AddPoints(chartBottomAxisPoint, chartZValPoint, CHART_POINT_NUM);
	pLineSerie4->AddPoints(chartBottomAxisPoint, chartRollValPoint, CHART_POINT_NUM);
	pLineSerie5->AddPoints(chartBottomAxisPoint, chartPitchValPoint, CHART_POINT_NUM);
	pLineSerie6->AddPoints(chartBottomAxisPoint, chartYawValPoint, CHART_POINT_NUM);

	m_ChartCtrl1.EnableRefresh(true);

}

CECATSampleDlg::CECATSampleDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CECATSampleDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_ICON1);
}

void CECATSampleDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHART_CTRL, m_ChartCtrl1);
	DDX_Control(pDX, IDC_EDIT_Sensor, Sensor_edit);
}

BEGIN_MESSAGE_MAP(CECATSampleDlg, CDialog)
	//{{AFX_MSG_MAP(CECATSampleDlg)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_InitialCard, OnBTNInitialCard)
	ON_BN_CLICKED(IDC_BTN_FindSlave, OnBTNFindSlave)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_CHK_SVON, OnChkSvon)
	ON_BN_CLICKED(IDC_CHK_ABS, OnChkAbs)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BTN_Rise, &CECATSampleDlg::OnBnClickedBtnRise)
	ON_BN_CLICKED(IDC_BTN_Middle, &CECATSampleDlg::OnBnClickedBtnMiddle)
	ON_BN_CLICKED(IDC_BTN_StopMe, &CECATSampleDlg::OnBnClickedBtnStopme)
	ON_BN_CLICKED(IDC_BTN_Down, &CECATSampleDlg::OnBnClickedBtnDown)
	ON_BN_CLICKED(IDOK, &CECATSampleDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BTN_CONNECT, &CECATSampleDlg::OnBnClickedBtnConnect)
	ON_BN_CLICKED(IDC_BTN_Resetme, &CECATSampleDlg::OnBnClickedBtnResetme)
	ON_BN_CLICKED(IDC_BTN_DISCONNECT, &CECATSampleDlg::OnBnClickedBtnDisconnect)
	ON_WM_SHOWWINDOW()
	ON_BN_CLICKED(IDC_BTN_SINGLE_UP, &CECATSampleDlg::OnBnClickedBtnSingleUp)
	ON_BN_CLICKED(IDC_BTN_SINGLE_DOWN, &CECATSampleDlg::OnBnClickedBtnSingleDown)
	ON_WM_KEYDOWN()
	ON_WM_CHAR()
	ON_BN_CLICKED(IDC_BUTTON_TEST, &CECATSampleDlg::OnBnClickedButtonTest)
	ON_BN_CLICKED(IDC_BUTTON_TEST3, &CECATSampleDlg::OnBnClickedButtonTest3)
	ON_BN_CLICKED(IDC_BUTTON_STOP_TEST, &CECATSampleDlg::OnBnClickedButtonStopTest)
	ON_BN_CLICKED(IDC_BUTTON_REPRODUCE, &CECATSampleDlg::OnBnClickedButtonReproduce)
	ON_BN_CLICKED(IDC_BUTTON_SHOW_ROADDATA, &CECATSampleDlg::OnBnClickedButtonShowRoadData)
	ON_BN_CLICKED(IDC_BUTTON_OIL_START, &CECATSampleDlg::OnBnClickedButtonOilStart)
	ON_BN_CLICKED(IDC_BTN_Pause, &CECATSampleDlg::OnBnClickedBtnPause)
	ON_BN_CLICKED(IDC_BTN_Recover, &CECATSampleDlg::OnBnClickedBtnRecover)
END_MESSAGE_MAP()


void CECATSampleDlg::ChartInit()
{
	CRect rect, rectChart;
	GetDlgItem(IDC_CHART_CTRL)->GetWindowRect(&rect);
	ScreenToClient(rect);
	rectChart = rect;
	rectChart.top = rect.bottom + 3;
	rectChart.bottom = rectChart.top + rect.Height();

	CChartAxis *pAxis = NULL;
	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::BottomAxis);
	pAxis->SetAutomatic(true);
	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::LeftAxis);
	pAxis->SetAutomatic(true);

	m_ChartCtrl1.GetTitle()->AddString(_T(CHART_TITLE));

	m_ChartCtrl1.
		GetLeftAxis()->
		GetLabel()->
		SetText(_T(CHART_LEFT_AXIS_TITLE));

	m_ChartCtrl1.EnableRefresh(false);

	m_ChartCtrl1.RemoveAllSeries();
	m_ChartCtrl1.GetLegend()->SetVisible(true);

	pLineSerie1 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie1->SetSeriesOrdering(poNoOrdering);
	pLineSerie1->SetWidth(2);
	pLineSerie1->SetName(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
	
	pLineSerie2 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie2->SetSeriesOrdering(poNoOrdering);
	
	pLineSerie2->SetWidth(2);
	pLineSerie2->SetName(_T(IDC_STATIC_Y_VAL_SHOW_TEXT)); 

	pLineSerie3 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie3->SetSeriesOrdering(poNoOrdering);
	pLineSerie3->SetWidth(2);
	pLineSerie3->SetName(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));

	pLineSerie4 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie4->SetSeriesOrdering(poNoOrdering);
	pLineSerie4->SetWidth(2);
	pLineSerie4->SetName(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));

	pLineSerie5 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie5->SetSeriesOrdering(poNoOrdering); 
	pLineSerie5->SetWidth(2);
	pLineSerie5->SetName(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));

	pLineSerie6 = m_ChartCtrl1.CreateLineSerie();
	pLineSerie6->SetSeriesOrdering(poNoOrdering);  
	pLineSerie6->SetWidth(2);
	pLineSerie6->SetName(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));

	m_ChartCtrl1.EnableRefresh(true);

}

void CECATSampleDlg::AppInit()
{
	int statusTemp = 0;
	// 检测应用是否注册
	auto isRegister = TestAppIsRegister();
	if (isRegister == false)
	{
		DialogRegister * dlg = new DialogRegister(this);
		dlg->DoModal();
		dlg->Create(IDD_DIALOG_REGISTER, this);
		dlg->ShowWindow(SW_SHOW);
		delete dlg;
		if (DialogRegister::IsRegister == false)
		{
			// 不注册就退出应用
			CDialog::OnOK();
		}
		else
		{
			MessageBox(_T("注册成功！"));
		}
	}
	// 读取上次应用退出时平台的状态
	config::ReadStatusAndPulse(statusTemp, lastStartPulse);
	// 读取停止并回中模式
	stopAndMiddle = config::ReadIsAutoStopAndMiddle();
	// 上次应用退出时平台的状态
	lastStartStatus = (SixDofPlatformStatus)statusTemp;

	SetDlgItemText(IDC_EDIT_X_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_VAL, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_VAL, _T("0"));

	SetDlgItemText(IDC_EDIT_X_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_HZ, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_HZ, _T("0"));

	SetDlgItemText(IDC_EDIT_X_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_PHASE, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_PHASE, _T("0"));

	SetDlgItemText(IDC_EDIT_X_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_Y_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_Z_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_ROLL_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_PITCH_ZERO_POS, _T("0"));
	SetDlgItemText(IDC_EDIT_YAW_ZERO_POS, _T("0"));

	CDialog::SetWindowTextW(_T(WINDOW_TITLE));
	GetDlgItem(IDC_BTN_Start)->SetWindowTextW(_T(IDC_BTN_START_SHOW_TEXT));
	GetDlgItem(IDC_BTN_Pause)->SetWindowTextW(_T(IDC_BTN_Pause_SHOW_TEXT));
	GetDlgItem(IDC_BTN_Recover)->SetWindowTextW(_T(IDC_BTN_Recover_SHOW_TEXT));
	GetDlgItem(IDC_BTN_SINGLE_UP)->SetWindowTextW(_T(IDC_BTN_SINGLE_UP_SHOW_TEXT));
	GetDlgItem(IDC_BTN_SINGLE_DOWN)->SetWindowTextW(_T(IDC_BTN_SINGLE_DOWN_SHOW_TEXT));

	GetDlgItem(IDC_BTN_CONNECT)->SetWindowTextW(_T(IDC_BTN_CONNECT_SHOW_TEXT));
	GetDlgItem(IDC_BTN_DISCONNECT)->SetWindowTextW(_T(IDC_BTN_DISCONNECT_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_POSE)->SetWindowTextW(_T(IDC_STATIC_POSE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_LENGTH)->SetWindowTextW(_T(IDC_STATIC_LENGTH_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_SENSOR)->SetWindowTextW(_T(IDC_STATIC_SENSOR_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_VAL)->SetWindowTextW(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_VAL)->SetWindowTextW(_T(IDC_STATIC_Y_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_VAL)->SetWindowTextW(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_VAL)->SetWindowTextW(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_VAL)->SetWindowTextW(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_VAL)->SetWindowTextW(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_HZ)->SetWindowTextW(_T(IDC_STATIC_X_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_HZ)->SetWindowTextW(_T(IDC_STATIC_Y_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_HZ)->SetWindowTextW(_T(IDC_STATIC_Z_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_HZ)->SetWindowTextW(_T(IDC_STATIC_ROLL_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_HZ)->SetWindowTextW(_T(IDC_STATIC_PITCH_HZ_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_HZ)->SetWindowTextW(_T(IDC_STATIC_YAW_HZ_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_PHASE)->SetWindowTextW(_T(IDC_STATIC_X_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_PHASE)->SetWindowTextW(_T(IDC_STATIC_Y_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_PHASE)->SetWindowTextW(_T(IDC_STATIC_Z_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_PHASE)->SetWindowTextW(_T(IDC_STATIC_ROLL_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_PHASE)->SetWindowTextW(_T(IDC_STATIC_PITCH_PHASE_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_PHASE)->SetWindowTextW(_T(IDC_STATIC_YAW_PHASE_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_X_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_X_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Y_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_Y_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_Z_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_Z_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_ROLL_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_ROLL_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_PITCH_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_PITCH_ZERO_POS_SHOW_TEXT));
	GetDlgItem(IDC_STATIC_YAW_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_YAW_ZERO_POS_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_TEST)->SetWindowTextW(_T(IDC_STATIC_TEST_SHOW_TEXT));
	GetDlgItem(IDC_BUTTON_TEST)->SetWindowTextW(_T(IDC_BUTTON_TEST_SHOW_TEXT));

	GetDlgItem(IDC_BUTTON_OIL_START)->SetWindowTextW(_T(IDC_BUTTON_OIL_START_SHOW_TEXT));
	GetDlgItem(IDC_BUTTON_SET_MID_POS)->SetWindowTextW(_T(IDC_BUTTON_SET_MID_POS_SHOW_TEXT));
	GetDlgItem(IDC_BUTTON_REPRODUCE)->SetWindowTextW(_T(IDC_BUTTON_REPRODUCE_SHOW_TEXT));
	GetDlgItem(IDC_BUTTON_SHOW_ROADDATA)->SetWindowTextW(_T(IDC_BUTTON_SHOW_ROADDATA_SHOW_TEXT));

	GetDlgItem(IDC_STATIC_APP_STATUS)->SetWindowTextW(_T(CORPORATION_NAME));
	GetDlgItem(IDC_STATIC_APP_TITLE)->SetWindowTextW(_T(APP_TITLE));
	CFont* font = new CFont();
	font->CreatePointFont(APP_TITLE_FONT_SIZE, _T("Times New Roman"));
	GetDlgItem(IDC_STATIC_APP_TITLE)->SetFont(font);
	ChartInit();
	for (auto i = 1; i <= AXES_COUNT; ++i)
	{
		CString xx;
		xx.Format(_T("%d"), i);
		((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->AddString(xx);
	}
	((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->SetCurSel(0);
	SetPlatformPara(PlaneAboveHingeLength, PlaneAboveBottomLength, 
		CircleTopRadius, CircleBottomRadius, DistanceBetweenHingeTop,
		DistanceBetweenHingeBottom);
	delta.ResetAllIoPorts();
	OpenThread();
}

double CECATSampleDlg::GetCEditNumber(int cEditId)
{
	CString str;
	GetDlgItemText(cEditId, str);
	auto val = _tstof(str);
	return val;
}

void CECATSampleDlg::OnShowWindow(BOOL bShow, UINT nStatus)
{
	CDialog::OnShowWindow(bShow, nStatus);
}

BOOL CECATSampleDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	SetIcon(m_hIcon, TRUE);			
	SetIcon(m_hIcon, FALSE);	
	// 应用初始化
	AppInit();
	// openGl初始化
	InitOpenGlControl();
	SetTimer(0, TIMER_MS, NULL);
	return TRUE;  
}

int isShowSingleUpDown = 0;
BOOL CECATSampleDlg::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		 if (pMsg->wParam == VK_SHIFT)
//		if (false)
		{
			isShowSingleUpDown = !isShowSingleUpDown;
			((CButton*)GetDlgItem(IDC_BTN_SINGLE_UP))->ShowWindow(isShowSingleUpDown);
			((CButton*)GetDlgItem(IDC_BTN_SINGLE_DOWN))->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_FindSlave)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_EDIT_SlaveNum)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_CBO_SingleNo)->ShowWindow(isShowSingleUpDown);

			GetDlgItem(IDC_CHK_SVON)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_Resetme)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_CONNECT)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_BTN_DISCONNECT)->ShowWindow(isShowSingleUpDown);

			GetDlgItem(IDC_BTN_InitialCard)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_CBO_CardNo)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_STATIC_SHOW_INIT)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_EDIT_InitialStatus)->ShowWindow(isShowSingleUpDown);
			GetDlgItem(IDC_SHOW)->ShowWindow(!isShowSingleUpDown);
		}
		if (pMsg->wParam == 'X')
		{
			closeDataThread = true;
			delta.ServoStop();
		}
	}
	else if (pMsg->message == WM_KEYUP)
	{

	}
	return CDialog::PreTranslateMessage(pMsg);
}

BOOL CECATSampleDlg::PreCreateWindow(CREATESTRUCT& cs)
{
	cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
	return CDialog::PreCreateWindow(cs);
}

void CECATSampleDlg::InitOpenGlControl()
{
	CWnd *pWnd = GetDlgItem(IDC_SHOW);
	hrenderDC = ::GetDC(pWnd->GetSafeHwnd());
	if(SetWindowPixelFormat(hrenderDC)==FALSE) 
		return; 
	if(CreateViewGLContext(hrenderDC)==FALSE) 
		return; 
	OpenGlLightInit();
}

BOOL CECATSampleDlg::SetWindowPixelFormat(HDC hDC) 
{ 
	PIXELFORMATDESCRIPTOR pixelDesc; 
	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR); 
	pixelDesc.nVersion = 1; 
	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER | PFD_STEREO_DONTCARE; 
	pixelDesc.iPixelType = PFD_TYPE_RGBA; 
	pixelDesc.cColorBits = 32; 
	pixelDesc.cRedBits = 8; 
	pixelDesc.cRedShift = 16; 
	pixelDesc.cGreenBits = 8; 
	pixelDesc.cGreenShift = 8; 
	pixelDesc.cBlueBits = 8; 
	pixelDesc.cBlueShift = 0; 
	pixelDesc.cAlphaBits = 0; 
	pixelDesc.cAlphaShift = 0; 
	pixelDesc.cAccumBits = 64; 
	pixelDesc.cAccumRedBits = 16; 
	pixelDesc.cAccumGreenBits = 16; 
	pixelDesc.cAccumBlueBits = 16; 
	pixelDesc.cAccumAlphaBits = 0; 
	pixelDesc.cDepthBits = 32; 
	pixelDesc.cStencilBits = 8; 
	pixelDesc.cAuxBuffers = 0; 
	pixelDesc.iLayerType = PFD_MAIN_PLANE; 
	pixelDesc.bReserved = 0; 
	pixelDesc.dwLayerMask = 0; 
	pixelDesc.dwVisibleMask = 0; 
	pixelDesc.dwDamageMask = 0; 
	PixelFormat = ChoosePixelFormat(hDC,&pixelDesc); 
	if(PixelFormat==0) 
	{ 
		PixelFormat = 1; 
		if(DescribePixelFormat(hDC,PixelFormat, 
			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc) == 0) 
		{ 
			return FALSE; 
		} 
	} 
	if(SetPixelFormat(hDC,PixelFormat,&pixelDesc) == FALSE) 
	{ 
		return FALSE; 
	} 
	return TRUE; 
} 

BOOL CECATSampleDlg::CreateViewGLContext(HDC hDC) 
{ 
	hrenderRC = wglCreateContext(hDC); 
	if(hrenderRC==NULL) 
		return FALSE; 
	if(wglMakeCurrent(hDC,hrenderRC)==FALSE) 
		return FALSE; 
	return TRUE; 
} 

void CECATSampleDlg::RenderScene()   
{ 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	OpenGL_SetData(data.X * XYZ_SCALE, data.Y * XYZ_SCALE, data.Z * XYZ_SCALE, 
		data.Roll * DEG_SCALE, data.Yaw * DEG_SCALE, data.Pitch * DEG_SCALE);
	RenderSixdofImage();
	SwapBuffers(hrenderDC); 
} 

void CECATSampleDlg::FillCtlColor(CWnd* cwnd, COLORREF color)
{
	CDC * pDC = cwnd->GetDC();
	CRect rct;
	cwnd->GetWindowRect(&rct);
	CBrush brs;
	brs.CreateSolidBrush(color);
	CRect picrct;
	picrct.top = 0;
	picrct.left = 0;
	picrct.bottom = rct.Height();
	picrct.right = rct.Width();
	pDC->FillRect(&picrct, &brs);
	pDC->ReleaseAttribDC();
	cwnd->ReleaseDC(pDC);
}

void CECATSampleDlg::ShowSingleInitImage(int ctlId)
{
	CRect rect; 
	GetDlgItem(ctlId)->GetClientRect(&rect);
	Gdiplus::Graphics g(GetDlgItem(ctlId)->GetDC()->m_hDC);   
	g.Clear(Gdiplus::Color::White);
	g.DrawImage(GetPumpImage(0, _T("mm")), 0, 0, rect.Width(), rect.Height());
}

void CECATSampleDlg::ShowSingleInitImage(CWnd* pic, float value)
{
	CRect rect; 
	pic->GetClientRect(&rect);
	Gdiplus::Graphics g(pic->GetDC()->m_hDC);   
	g.Clear(Gdiplus::Color::White);
	g.DrawImage(GetPumpImage(value, _T("mm")), 0, 0, rect.Width(), rect.Height());
}

void CECATSampleDlg::ShowInitImage()
{

}

CPen drawLinePen(PS_SOLID, 2, RGB(0, 0, 0));
CPen clearLinePen(PS_SOLID, 10, RGB(244, 244, 244));
CBrush clearBrush(RGB(244, 244, 244));
double ptCenterX = 52;
double ptCenterY = 52;
double f0;
double p0X;
double p0Y;
double p1X;
double p1Y;
//	张开的弧度
float radian=60;
//	同心圆半径
int radius[]=
{
	50,
	24,
	1
};

double fMax = 100;
double fMin = -100;

void CECATSampleDlg::ShowSingleImage(CWnd* pic, float value)
{
	value *= 2;
	CDC* dc = pic->GetDC();   
	f0 = ((180-radian)/2+radian/(fMax-fMin)*(-value + fMax)) / 180 * PI;
	dc->SelectObject(&clearBrush);
	dc->FillSolidRect(2,28,100,32, RGB(255, 255, 255));
	p0X = ptCenterX+(radius[2]*cos(f0));
	p0Y = ptCenterY-(radius[2]*sin(f0));
	p1X = ptCenterX+((radius[1])*cos(f0));
	p1Y = ptCenterY-((radius[1])*sin(f0));
	dc->SelectObject(&drawLinePen);
	dc->MoveTo(p0X, p0Y);
	dc->LineTo(p1X, p1Y);
	pic->ReleaseDC(dc);
}

void CECATSampleDlg::ShowImage()
{

}

void CECATSampleDlg::RenderSwitchStatus()
{

}

void CECATSampleDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); 

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		dc.DrawIcon(x, y, m_hIcon);		
	}
	else
	{
		CDialog::OnPaint();
	}
}

HCURSOR CECATSampleDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

void CECATSampleDlg::OnBTNInitialCard() 
{

}

void CECATSampleDlg::OnBTNFindSlave() 
{
	
}

void CECATSampleDlg::EanbleButton(int isenable)
{	
	((CButton*) GetDlgItem(IDC_CHK_SVON))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Rise))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Middle))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Start))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_StopMe))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Down))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Pause))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Recover))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BTN_Resetme))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_TEST))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_STOP_TEST))->EnableWindow(isenable);

	((CButton*) GetDlgItem(IDC_BUTTON_SET_MID_POS))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_REPRODUCE))->EnableWindow(isenable);
	((CButton*) GetDlgItem(IDC_BUTTON_SHOW_ROADDATA))->EnableWindow(isenable);

	((CEdit*) GetDlgItem(IDC_EDIT_X_VAL))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Y_VAL))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Z_VAL))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_X_HZ))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Y_HZ))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Z_HZ))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_X_PHASE))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Y_PHASE))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Z_PHASE))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_X_ZERO_POS))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Y_ZERO_POS))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_Z_ZERO_POS))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_VAL))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_VAL))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_YAW_VAL))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_HZ))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_HZ))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_YAW_HZ))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_PHASE))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_PHASE))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_YAW_PHASE))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_ZERO_POS))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_YAW_ZERO_POS))->EnableWindow(isenable);
	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_ZERO_POS))->EnableWindow(isenable);

}

CString statusStr = "";

void CECATSampleDlg::OnTimer(UINT nIDEvent) 
{
	MoveValPoint();
	RenderScene();
	PlatTimerFunc();
	//RenderSwitchStatus();
	//ShowImage();
	statusStr.Format(_T("x:%d y:%d z:%d y:%d a:%d b:%d time:%.2f count:%d"), data.X, data.Y, data.Z,
		data.Yaw, data.Pitch, data.Roll, runTime, 0);
	SetDlgItemText(IDC_EDIT_Pose, statusStr);

	//delta.GetNowPulse(cardVols);
	statusStr.Format(_T("1:%.2f 2:%.2f 3:%.2f 4:%.2f 5:%.2f 6:%.2f"), 
		cardVols[0], cardVols[1], cardVols[2],
		cardVols[3], cardVols[4], cardVols[5]);
	SetDlgItemText(IDC_EDIT_Sensor, statusStr);

	for (int i = 0;i < AXES_COUNT;++i)
	{
		cardPoleLength[i] = cardVols[i] * PULSE_COUNT_TO_MM_SCALE;
	}

	statusStr.Format(_T("1:%.2f 2:%.2f 3:%.2f 4:%.2f 5:%.2f 6:%.2f"), 
		cardPoleLength[0], cardPoleLength[1], cardPoleLength[2],
		cardPoleLength[3], cardPoleLength[4], cardPoleLength[5]);
	SetDlgItemText(IDC_EDIT_Pulse, statusStr);

	if (++fileCount >= 10) {
		fileCount = 0;
		fileSave.Record(data.X, data.Y, data.Z, data.Roll, data.Yaw, data.Pitch, 
			poleLength, cardVols);
	}

	CDialog::OnTimer(nIDEvent);
}

void CECATSampleDlg::OnChkSvon() 
{
	
}

void CECATSampleDlg::OnOK() 
{
	delta.ServoStop();
	Sleep(100);
	delta.Close(status);
	CDialog::OnOK();
}

void CECATSampleDlg::OnChkAbs() 
{
	
}

void CECATSampleDlg::OnBnClickedBtnRise()
{	
	if (status != SIXDOF_STATUS_BOTTOM)
	{
#if IS_USE_MESSAGE_BOX
		MessageBox(_T(SIXDOF_NOT_BOTTOM_MESSAGE));
#endif	
		return;
	}
	Sleep(20);
	status = SIXDOF_STATUS_ISRISING;	
	delta.Rise();
	Sleep(20);
	status = SIXDOF_STATUS_READY;
}

void CECATSampleDlg::OnBnClickedBtnMiddle()
{
	if (stopAndMiddle == true)
	{
		if (status != SIXDOF_STATUS_READY)
		{
#if IS_USE_MESSAGE_BOX
			MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
#endif
			return;
		}
	}
	else
	{
		if (status != SIXDOF_STATUS_MIDDLE)
		{
#if IS_USE_MESSAGE_BOX
			MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
#endif	
			return;
		}
	}
	status = SIXDOF_STATUS_READY;
	delta.MoveToZeroPulseNumber();
}

void CECATSampleDlg::OnBnClickedBtnStart()
{
	if (status != SIXDOF_STATUS_READY)
	{
#if IS_USE_MESSAGE_BOX
		MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
#endif
		return;
	}
	status = SIXDOF_STATUS_RUN;
	delta.ServoStop();
	Sleep(100);
	delta.RenewNowPulse();
	delta.GetMotionAveragePulse();
	delta.PidControllerInit();
	// 正常使用模式
	isTest = false;
	isCosMode = false;
	t = 0;
	closeDataThread = false;
}

void CECATSampleDlg::OnCommandStopme()
{
	if (status != SIXDOF_STATUS_RUN)
	{
		return;
	}
	status = SIXDOF_STATUS_READY;
	closeDataThread = true;
	delta.servoCurveStopThread();
	delta.MoveToZeroPulseNumber();
	ResetDefaultData(&data);
}

void CECATSampleDlg::OnBnClickedBtnStopme()
{
	// 停止Csp运动
	stopSCurve = true;
	closeDataThread = true;
	if (status == SIXDOF_STATUS_RUN)
	{
		delta.servoCurveStopThread();
		status = SIXDOF_STATUS_READY;
		OnBnClickedBtnDown();
	}
	ResetDefaultData(&data);
}

void CECATSampleDlg::OnBnClickedBtnDown()
{	
	if (status == SIXDOF_STATUS_RUN || status == SIXDOF_STATUS_PAUSING) {
#if IS_USE_MESSAGE_BOX
		MessageBox(_T(SIXDOF_NOT_FALLING_MESSAGE));
#endif
		return;
	}
	// status = SIXDOF_STATUS_ISFALLING;
	delta.SetEnable(true);
	delta.StopRiseDownMove();
	Sleep(100);
	delta.Down();
	status = SIXDOF_STATUS_BOTTOM;
}

void CECATSampleDlg::OnBnClickedOk()
{
	delta.ServoStop();
	delta.Close(status);
	Sleep(50);
	if (finishFlag != 0)
		return;
	((CButton*) GetDlgItem(IDOK))->EnableWindow(0);
	GetDlgItem(IDOK)->SetWindowTextW(_T(IDC_BUTTON_FINISH_ING_SHOW_TEXT));
	thread td(PlatformCloseThread);
	td.detach();
}

void CECATSampleDlg::OnBnClickedBtnConnect()
{
	delta.AllTestUp();
}

void CECATSampleDlg::OnBnClickedBtnResetme()
{
	
}

void CECATSampleDlg::OnBnClickedBtnDisconnect()
{
	delta.AllTestDown();
}

void CECATSampleDlg::OnBnClickedBtnSingleUp()
{
	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
	delta.SingleUp(index);
}

void CECATSampleDlg::OnBnClickedBtnSingleDown()
{
	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
	delta.SingleDown(index);
}

void CECATSampleDlg::OnBnClickedButtonTest()
{
	int valid = 0;
	if (status != SIXDOF_STATUS_READY)
	{
#if IS_USE_MESSAGE_BOX
		MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
#endif
		return;
	}
	status = SIXDOF_STATUS_RUN;

	memset(testPhase, 0, sizeof(double) * AXES_COUNT);
	//位移单位mm 角度单位 度
	auto xval = RANGE(GetCEditNumber(IDC_EDIT_X_VAL), -MAX_XYZ_X, MAX_XYZ_X); 
	auto yval = RANGE(GetCEditNumber(IDC_EDIT_Y_VAL), -MAX_XYZ_Y, MAX_XYZ_Y);
	auto zval = RANGE(GetCEditNumber(IDC_EDIT_Z_VAL), -MAX_XYZ_Z, MAX_XYZ_Z);
	auto rollval = RANGE(GetCEditNumber(IDC_EDIT_ROLL_VAL), -MAX_DEG_ROLL, MAX_DEG_ROLL);
	auto pitchval = RANGE(GetCEditNumber(IDC_EDIT_PITCH_VAL), -MAX_DEG_PITCH, MAX_DEG_PITCH);
	auto yawval = RANGE(GetCEditNumber(IDC_EDIT_YAW_VAL), -MAX_DEG_YAW, MAX_DEG_YAW);
	//频率单位1hz
	auto xhz = RANGE(GetCEditNumber(IDC_EDIT_X_HZ), 0, MAX_HZ);
	auto yhz = RANGE(GetCEditNumber(IDC_EDIT_Y_HZ), 0, MAX_HZ);
	auto zhz = RANGE(GetCEditNumber(IDC_EDIT_Z_HZ), 0, MAX_HZ);
	auto rollhz = RANGE(GetCEditNumber(IDC_EDIT_ROLL_HZ), 0, MAX_HZ);
	auto pitchhz = RANGE(GetCEditNumber(IDC_EDIT_PITCH_HZ), 0, MAX_HZ);
	auto yawhz = RANGE(GetCEditNumber(IDC_EDIT_YAW_HZ), 0, MAX_HZ);

	auto xphase = RANGE(GetCEditNumber(IDC_EDIT_X_PHASE), 0, MAX_PHASE);
	auto yphase = RANGE(GetCEditNumber(IDC_EDIT_Y_PHASE), 0, MAX_PHASE);
	auto zphase = RANGE(GetCEditNumber(IDC_EDIT_Z_PHASE), 0, MAX_PHASE);
	auto rollphase = RANGE(GetCEditNumber(IDC_EDIT_ROLL_PHASE), 0, MAX_PHASE);
	auto pitchphase = RANGE(GetCEditNumber(IDC_EDIT_PITCH_PHASE), 0, MAX_PHASE);
	auto yawphase = RANGE(GetCEditNumber(IDC_EDIT_YAW_PHASE), 0, MAX_PHASE);

	auto xzeropos = RANGE(GetCEditNumber(IDC_EDIT_X_ZERO_POS), -MAX_XYZ_ZERO_POS_X, MAX_XYZ_ZERO_POS_X);
	auto yzeropos = RANGE(GetCEditNumber(IDC_EDIT_Y_ZERO_POS), -MAX_XYZ_ZERO_POS_Y, MAX_XYZ_ZERO_POS_Y);
	auto zzeropos = RANGE(GetCEditNumber(IDC_EDIT_Z_ZERO_POS), -MAX_XYZ_ZERO_POS_Z, MAX_XYZ_ZERO_POS_Z);
	auto rollzeropos = RANGE(GetCEditNumber(IDC_EDIT_ROLL_ZERO_POS), -MAX_DEG_ZERO_POS_PITCH, MAX_DEG_ZERO_POS_PITCH);
	auto pitchzeropos = RANGE(GetCEditNumber(IDC_EDIT_PITCH_ZERO_POS), -MAX_DEG_ZERO_POS_ROLL, MAX_DEG_ZERO_POS_ROLL);
	auto yawzeropos = RANGE(GetCEditNumber(IDC_EDIT_YAW_ZERO_POS), -MAX_DEG_ZERO_POS_YAW, MAX_DEG_ZERO_POS_YAW);

	testVal[0] = xval;
	valid += abs(testVal[0]) < MAX_XYZ_X ? 0 : 1;
	testVal[1] = yval;
	valid += abs(testVal[1]) < MAX_XYZ_Y ? 0 : 1;
	testVal[2] = zval;
	valid += abs(testVal[2]) < MAX_XYZ_Z ? 0 : 1;
	testVal[3] = rollval;
	valid += abs(testVal[3]) < MAX_DEG_ROLL ? 0 : 1;
	testVal[4] = pitchval;
	valid += abs(testVal[4]) < MAX_DEG_PITCH ? 0 : 1;
	testVal[5] = yawval;
	valid += abs(testVal[5]) < MAX_DEG_YAW ? 0 : 1;
	if(valid != 0)
	{
		MessageBox(_T("参数超限请重新输入"));
		return ;
	}

	testHz[0] = xhz;
	testHz[1] = yhz;
	testHz[2] = zhz;
	testHz[3] = rollhz;
	testHz[4] = pitchhz;
	testHz[5] = yawhz;

	testPhase[0] = xphase;
	testPhase[1] = yphase;
	testPhase[2] = zphase;
	testPhase[3] = rollphase;
	testPhase[4] = pitchphase;
	testPhase[5] = yawphase;

	testZeroPos[0] = xzeropos;
	testZeroPos[1] = yzeropos;
	testZeroPos[2] = zzeropos;
	testZeroPos[3] = rollzeropos;
	testZeroPos[4] = pitchzeropos;
	testZeroPos[5] = yawzeropos;

	if (xphase != 0 || yphase != 0 || zphase != 0 || 
		rollphase != 0 || pitchphase != 0 || yawphase != 0)
	{
		enableChirp = false;
		isCosMode = true;
	}
	else
	{
		enableChirp = ENABLE_CHIRP;
		isCosMode = false;
	}
	stopSCurve = false;

	// 电机先停后启动
	delta.StopRiseDownMove();
	delta.RenewNowPulse();
	delta.GetMotionAveragePulse();
	delta.PidControllerInit();
	// 正弦测试运动模式
	isTest = true;
	// 正弦时间清0
	t = 0;
	// 允许运动
	closeDataThread = false;
}


void CECATSampleDlg::OnBnClickedButtonTest3()
{
	delta.TestHardware();
}

void CECATSampleDlg::OnBnClickedButtonStopTest()
{
	OnBnClickedBtnStopme();
}

void CECATSampleDlg::DataFromFile()
{
	CString targetHistoryPath = "";
	CString defaultDir = L"C:\\";  //默认打开的文件路径
	CString defaultFile = L"test.txt"; //默认打开的文件名
	CFileDialog dlg(TRUE, _T("txt"), defaultDir + "\\" + defaultFile, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, _T("数据文件|*.txt||"));
	if (dlg.DoModal() == IDOK)
	{
		targetHistoryPath = dlg.GetPathName();
		if (targetHistoryPath == "")
		{
			MessageBox(_T("未选择文件！"));
			return;
		}
	}

	ifstream fin(targetHistoryPath);		
	double *ptr = &SourceBuf[0]; 
	int readcount = 0;
	int arrcount = 0;
	if (!fin.is_open())
	{
		AfxMessageBox(_T("未找到文件!"));
		return;
	}
	roadSpectrum.DataBuffer.clear();
	while (!fin.eof() && readcount < MAX_REPRODUCE_LINE * DATA_COL_NUM)
	{
		fin >> *ptr; 
		ptr++;
		readcount++;
		arrcount++;
		if (arrcount >= DATA_COL_NUM)
		{
			arrcount = 0;
			roadSpectrum.DataBuffer.push_back(Signal::RoadSpectrumData::FromArray(SourceBuf));
			ptr = &SourceBuf[0];
		}
	}
	fin.close();
}

void CECATSampleDlg::OnBnClickedButtonReproduce()
{
#if _DEBUG
#else
	if (status != SIXDOF_STATUS_READY)
	{
#if IS_USE_MESSAGEBOX
		MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
#endif
		return;
	}
#endif
	DataFromFile();
	Sleep(20);
	if (roadSpectrum.DataBuffer.size() <= 0)
		return;
#if _DEBUG
	status = SIXDOF_STATUS_READY;
#endif
	OnBnClickedBtnStart();
}

void CECATSampleDlg::OnBnClickedButtonShowRoadData()
{
	DataFromFile();
	isTest = false;
	t = 0;
	closeDataThread = false;
}

void CECATSampleDlg::OnBnClickedButtonOilStart()
{
	if (startFlag != 0)
		return;
	((CButton*) GetDlgItem(IDC_BUTTON_OIL_START))->EnableWindow(0);
	GetDlgItem(IDC_BUTTON_OIL_START)->SetWindowTextW(_T(IDC_BUTTON_OIL_ING_START_SHOW_TEXT));
	thread td(PlatformInitThread);
	td.detach();
}

void CECATSampleDlg::OnBnClickedBtnPause()
{
	if (status == SIXDOF_STATUS_RUN)
	{
		Sleep(10);
		status = SIXDOF_STATUS_PAUSING;
	}
}

void CECATSampleDlg::OnBnClickedBtnRecover()
{
	if (status == SIXDOF_STATUS_PAUSING)
	{
		status = SIXDOF_STATUS_RUN;
		Sleep(10);	
	}
}



























//
//#include "StdAfx.h"
//#include "ECATSample.h"
//#include "ECATSampleDlg.h"
//
//#include <math.h>
//
//#include <fstream>
//#include <deque>
//#include <thread>
//
//#include "DialogRegister.h"
//
//#include "chart.h"
//
//#include "Sixdofdll2010.h"
//
//#include "communication/sixdof.h"
//#include "communication/dialogmotioncontrol.h"
//#include "communication/SerialPort.h"
//
//#include "control/sensor.h"
//#include "control/pid.h"
//#include "control/kalman_filter.h"
//#include "control/inertialnavigation.h"
//#include "control/water.h"
//#include "control/illusion.h"
//#include "control/platformapi.h"
//
//#include "config/recordpath.h"
//#include "ui/uiconfig.h"
//#include "signal/roadspectrum.h"
//#include "util/model.h"
//#include "register/register.h"
//#include "hardware/SixdofDioAndPulseCount.h"
//
//#include "glut.h"
//#include "opengl/sixdofopenglhelper.h"
//
//#include "cwnds/panel.h"
//
//#ifdef _DEBUG
//#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
//#endif
//
//using namespace std;
//
//#define COLOR_RED     RGB(255, 123, 123)
//#define COLOR_GREEN   RGB(123, 255, 123)
//
//// 定时器周期
//#define TIMER_MS 10
//
//// 六自由度平台控制周期
//#define SIXDOF_CONTROL_DELEY 1
//#define SCENE_THREAD_DELAY 1000
//#define SENSOR_THREAD_DELAY 1000
//#define DATA_BUFFER_THREAD_DELAY 1000
//
//// 平滑启动时间
//#define CHIRP_TIME       2.5
//// 到达姿态零偏时间
//#define ZERO_POS_TIME    2.5
//#define ENABLE_CHIRP  false
//#define ENABLE_SHOCK  false
//#define ENABLE_ZERO_POS true
//
//// 是否显示对话框
//#define IS_USE_MESSAGE_BOX 1
//
//bool enableShock = ENABLE_SHOCK;
//bool enableChirp = ENABLE_CHIRP;
//bool enableZeroPos = ENABLE_ZERO_POS;
//
//void SixdofControl();
//
//bool closeDataThread = true;
//bool stopAndMiddle = true;
//
//volatile HANDLE DataThread;
//volatile HANDLE SensorThread;
//volatile HANDLE SceneThread;
//volatile HANDLE DataBufferThread;
//volatile HANDLE PlatInitThread;  
//volatile HANDLE PlatCloseThread;
//
//// 六自由度平台逻辑控制
//DialogMotionControl delta;
//// 六自由度数据
//DataPackage data = {0};
//
//// 六自由度平台状态
//double pulse_cal[AXES_COUNT];
//double poleLength[AXES_COUNT];
//double lastStartPulse[AXES_COUNT];
//SixDofPlatformStatus status = SIXDOF_STATUS_BOTTOM;
//SixDofPlatformStatus lastStartStatus = SIXDOF_STATUS_BOTTOM;
//Signal::RoadSpectrum roadSpectrum;
//
//// 图表
//CChartCtrl m_ChartCtrl1; 
//CChartLineSerie *pLineSerie1;
//CChartLineSerie *pLineSerie2;
//CChartLineSerie *pLineSerie3;
//CChartLineSerie *pLineSerie4;
//CChartLineSerie *pLineSerie5;
//CChartLineSerie *pLineSerie6;
//
//#define DATA_COL_NUM 18
//#define MAX_REPRODUCE_LINE 1000
//double SourceBuf[DATA_COL_NUM] = {0};
//
//bool isTest = true;
//bool isCosMode = false;
//bool stopSCurve = false;
//
//double testVal[FREEDOM_NUM] = { 0 };
//double testHz[FREEDOM_NUM] = { 0 };
//double testPhase[FREEDOM_NUM] = { 0 };
//double testZeroPos[FREEDOM_NUM] = { 0 };
//
//double chartBottomAxisPoint[CHART_POINT_NUM] = { 0 };
//double chartXValPoint[CHART_POINT_NUM] = { 0 };
//double chartYValPoint[CHART_POINT_NUM] = { 0 };
//double chartZValPoint[CHART_POINT_NUM] = { 0 };
//double chartRollValPoint[CHART_POINT_NUM] = { 0 };
//double chartPitchValPoint[CHART_POINT_NUM] = { 0 };
//double chartYawValPoint[CHART_POINT_NUM] = { 0 };
//
//double t = 0;
//double runTime = 0;
//double chartTime = 0;
//
//mutex csdata;
//mutex ctrlCommandLockobj;
//DataPackageDouble visionData = {0};
//DataPackageDouble lastData = {0};
//config::FileRecord fileSave;
//double cardVols[AXES_COUNT] = {0};
//double cardPoleLength[AXES_COUNT] = {0};
//
//int startFlag = 0;
//int finishFlag = 0;
//int fileCount = 0;
//
//DWORD WINAPI DataTransThread(LPVOID pParam)
//{
//	while (true)
//	{
//		SixdofControl();
//	}
//	return 0;
//}
//
//DWORD WINAPI SensorInfoThread(LPVOID pParam)
//{
//	while (true)
//	{
//		Sleep(SENSOR_THREAD_DELAY);
//	}
//	return 0;
//}
//
//DWORD WINAPI SceneInfoThread(LPVOID pParam)
//{
//	while (true)
//	{	
//		Sleep(SCENE_THREAD_DELAY);
//	}
//	return 0;
//}
//
//
//DWORD WINAPI DataBufferInfoThread(LPVOID pParam)
//{
//	// 上升、下降、中立位的逻辑控制
//	delta.DDAControlThread();
//	return 0;
//}
//
//void PlatformInitThread()
//{
//	startFlag = 0;
//	delta.SetOilStart(true);
//	Sleep(5000);
//	delta.SetOilStart(false);
//	Sleep(5000);
//	delta.SetEnable(true);
//	delta.AllTestDown();
//	startFlag = 1;
//	Sleep(60000);
//	if (startFlag != 3)
//		delta.SetDisable(false);
//	startFlag = 2;
//}
//
//void PlatformCloseThread()
//{
//	finishFlag = 0;
//	delta.SetOilStop(true);
//	Sleep(5000);
//	delta.SetOilStop(false);
//	Sleep(60000);
//	delta.SetEnable(false);
//	delta.SetDisable(true);
//	delta.AllTestDown();
//	finishFlag = 1;
//	Sleep(10);
//	delta.SetDisable(false);
//	Sleep(5);
//	finishFlag = 2;
//}
//
//void CECATSampleDlg::PlatTimerFunc()
//{
//	if (startFlag == 1 && delta.IsLowPower() == true)
//	{
//		startFlag = 3;
//		MessageBox(_T("低压报警！"));
//		OnBnClickedOk();
//	}
//	else if (startFlag == 2)
//	{
//		startFlag = 3;
//		EanbleButton(1);
//	}
//	if (finishFlag == 2)
//	{
//		finishFlag = 3;
//		Sleep(10);
//		CloseThread();
//		CDialog::OnOK();
//	}
//}
//
//// 释放所有线程资源
//void CECATSampleDlg::CloseThread()
//{
//	try
//	{
//		if (DataThread != INVALID_HANDLE_VALUE)
//		{
//			CloseHandle(DataThread);
//			DataThread = INVALID_HANDLE_VALUE;
//		}
//	}
//	catch (CException* e)
//	{
//		printf("%d\r\n", e->ReportError());
//	}
//	try
//	{
//		if (SensorThread != INVALID_HANDLE_VALUE)
//		{
//			CloseHandle(SensorThread);
//			SensorThread = INVALID_HANDLE_VALUE;
//		}
//	}
//	catch (CException* e)
//	{
//		printf("%d\r\n", e->ReportError());
//	}
//	try
//	{
//		if (SceneThread != INVALID_HANDLE_VALUE)
//		{
//			CloseHandle(SceneThread);
//			SceneThread = INVALID_HANDLE_VALUE;
//		}
//	}
//	catch (CException* e)
//	{
//		printf("%d\r\n", e->ReportError());
//	}
//	try
//	{
//		if (DataBufferThread != INVALID_HANDLE_VALUE)
//		{
//			CloseHandle(DataBufferThread);
//			DataBufferThread = INVALID_HANDLE_VALUE;
//		}
//	}
//	catch (CException* e)
//	{
//		printf("%d\r\n", e->ReportError());
//	}
//	try
//	{
//		if (PlatInitThread != INVALID_HANDLE_VALUE)
//		{
//			CloseHandle(PlatInitThread);
//			PlatInitThread = INVALID_HANDLE_VALUE;
//		}
//	}
//	catch (CException* e)
//	{
//		printf("%d\r\n", e->ReportError());
//	}
//	try
//	{
//		if (PlatCloseThread != INVALID_HANDLE_VALUE)
//		{
//			CloseHandle(PlatCloseThread);
//			PlatCloseThread = INVALID_HANDLE_VALUE;
//		}
//	}
//	catch (CException* e)
//	{
//		printf("%d\r\n", e->ReportError());
//	}
//}
//
//void OpenThread()
//{
//	DataThread = (HANDLE)CreateThread(NULL, 0, DataTransThread, NULL, 0, NULL);
//	DataBufferThread = (HANDLE)CreateThread(NULL, 0, DataBufferInfoThread, NULL, 0, NULL);
//	//SensorThread = (HANDLE)CreateThread(NULL, 0, SensorInfoThread, NULL, 0, NULL);
//	//SceneThread = (HANDLE)CreateThread(NULL, 0, SceneInfoThread, NULL, 0, NULL);	
//	//TimerThread = (HANDLE)CreateThread(NULL, 0, TimerInfoThread, NULL, 0, NULL);
//}
//
//double vision_x = 0;
//double vision_y = 0;
//double vision_z = 0;
//double vision_roll = 0;
//double vision_pitch = 0;
//double vision_yaw = 0;
//
//void SixdofControl()
//{
//	static double deltat = 0.026;
//	Sleep(10);
//	if(closeDataThread == false)
//	{	
//		DWORD start_time = 0;
//		start_time = GetTickCount();
//		delta.RenewNowPulse();
//		auto delay = SIXDOF_CONTROL_DELEY;
//		double dis[AXES_COUNT] = {0};
//		// 正弦测试运动
//		if (isTest == true)
//		{
//			double nowHz[AXES_COUNT] = {testHz[0], testHz[1], testHz[2], testHz[3], testHz[4], testHz[5]};
//			double chirpTime = CHIRP_TIME;
//			double zeroposTime = ZERO_POS_TIME;
//			double pi = 3.1415926;
//			double nowt = t;
//			double x = 0;
//			double y = 0;
//			double z = 0;
//			double roll = 0;
//			double pitch = 0;
//			double yaw = 0;
//			double stopTime = 0;
//			// 开始正弦运动时加一个chrip信号缓冲
//			if (t <= chirpTime && enableChirp == true)
//			{
//				for (auto i = 0; i < AXES_COUNT; ++i)
//				{
//					nowHz[i] = t * testHz[i] / chirpTime / chirpTime; 
//				}
//				x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
//				y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
//				z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
//				roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
//				pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
//				yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
//			}		
//			else if (enableChirp == true)
//			{
//				nowt = t - chirpTime;
//				x = sin(2 * pi * nowHz[0] * nowt + 2 * pi * testHz[0]) * testVal[0];
//				y = sin(2 * pi * nowHz[1] * nowt + 2 * pi * testHz[1]) * testVal[1];
//				z = sin(2 * pi * nowHz[2] * nowt + 2 * pi * testHz[2]) * testVal[2];
//				roll = sin(2 * pi * nowHz[3] * nowt + 2 * pi * testHz[3]) * testVal[3];
//				pitch = sin(2 * pi * nowHz[4] * nowt + 2 * pi * testHz[4]) * testVal[4];
//				yaw = sin(2 * pi * nowHz[5] * nowt + 2 * pi * testHz[5]) * testVal[5];
//			}
//			else if(isCosMode == false)
//			{
//				x = sin(2 * pi * nowHz[0] * nowt) * testVal[0];
//				y = sin(2 * pi * nowHz[1] * nowt) * testVal[1];
//				z = sin(2 * pi * nowHz[2] * nowt) * testVal[2];
//				roll = sin(2 * pi * nowHz[3] * nowt) * testVal[3];
//				pitch = sin(2 * pi * nowHz[4] * nowt) * testVal[4];
//				yaw = sin(2 * pi * nowHz[5] * nowt) * testVal[5];
//			}
//			else
//			{
//
//			}
//			if (isCosMode = true)
//			{
//				double nowphase_t[AXES_COUNT] = {t,t,t,t,t,t};
//				for (int i = 0;i < AXES_COUNT; ++i)
//				{
//					if (nowHz[i] != 0)
//					{
//						auto phase_t = t - 1.0 / nowHz[i] * (testPhase[i] / 360.0);
//						nowphase_t[i] = DOWN_RANGE(phase_t, 0);
//					}			
//					else
//					{
//						nowphase_t[i] = t;
//					}					
//				}
//				x = sin(2 * pi * nowHz[0] * nowphase_t[0]) * testVal[0];
//				y = sin(2 * pi * nowHz[1] * nowphase_t[1]) * testVal[1];
//				z = sin(2 * pi * nowHz[2] * nowphase_t[2]) * testVal[2];
//				roll = sin(2 * pi * nowHz[3] * nowphase_t[3]) * testVal[3];
//				pitch = sin(2 * pi * nowHz[4] * nowphase_t[4]) * testVal[4];
//				yaw = sin(2 * pi * nowHz[5] * nowphase_t[5]) * testVal[5];
//			}
//			if (enableZeroPos == true)
//			{
//				auto minZeroPosTime = min(t, zeroposTime) / zeroposTime;
//				x += testZeroPos[0] * minZeroPosTime;
//				y += testZeroPos[1] * minZeroPosTime;
//				z += testZeroPos[2] * minZeroPosTime;
//				roll += testZeroPos[3] * minZeroPosTime;
//				pitch += testZeroPos[4] * minZeroPosTime;
//				yaw += testZeroPos[5] * minZeroPosTime;
//			}
//			data.X = (int16_t)(x * 10);
//			data.Y = (int16_t)(y * 10);
//			data.Z = (int16_t)(z * 10);
//			data.Roll = (int16_t)(roll * 100);
//			data.Pitch = (int16_t)(pitch * 100);
//			data.Yaw = (int16_t)(yaw * 100);
//			double* pulse_dugu = Control(x, y, z, roll, yaw, pitch);
//			for (auto ii = 0; ii < AXES_COUNT; ++ii)
//			{
//				pulse_cal[ii] = pulse_dugu[ii];
//				poleLength[ii] = pulse_dugu[ii];
//				pulse_cal[ii] *= MM_TO_PULSE_COUNT_SCALE;
//				auto pulse = pulse_cal[ii];
//				dis[ii] = pulse;
//			}
//			t += deltat;
//			delta.PidCsp(dis);
//		}
//		// 模拟船视景
//		else
//		{
//			if (roadSpectrum.DataBuffer.size() > 0) {
//				auto roaddata = roadSpectrum.DataBuffer.front();
//				roadSpectrum.DataBuffer.pop_front();
//				vision_x = roaddata.Position.X;
//				vision_y = roaddata.Position.Y;
//				vision_z = roaddata.Position.Z;
//				vision_roll = roaddata.Position.Roll;
//				vision_pitch = roaddata.Position.Pitch;
//				vision_yaw = roaddata.Position.Yaw;
//			}
//			else {			
//				if (status != SIXDOF_STATUS_RUN) {
//					t = 0;
//					closeDataThread = true;	
//				}
//			}
//			auto x = RANGE(vision_x, -MAX_XYZ, MAX_XYZ);
//			auto y = RANGE(vision_y, -MAX_XYZ, MAX_XYZ);
//			auto z = RANGE(vision_z, -MAX_XYZ, MAX_XYZ);
//			auto roll = RANGE(vision_roll, -MAX_DEG, MAX_DEG);
//			auto pitch = RANGE(vision_pitch, -MAX_DEG, MAX_DEG);
//			auto yaw = RANGE(vision_yaw, -MAX_DEG, MAX_DEG);
//
//			double* pulse_dugu = Control(x, y, z, roll, yaw, pitch);
//			lastData.X = x;
//			lastData.Y = y;
//			lastData.Z = z;
//			lastData.Roll = roll;
//			lastData.Pitch = pitch;
//			lastData.Yaw = yaw;
//			for (auto ii = 0; ii < AXES_COUNT; ++ii)
//			{
//				pulse_cal[ii] = pulse_dugu[ii];
//				poleLength[ii] = pulse_dugu[ii];
//				pulse_cal[ii] *= MM_TO_PULSE_COUNT_SCALE;
//				auto pulse = pulse_cal[ii];
//				dis[ii] = pulse;
//			}
//			data.X = (int16_t)(x * 10);
//			data.Y = (int16_t)(y * 10);
//			data.Z = (int16_t)(z * 10);
//			data.Roll = (int16_t)(roll * 100);
//			data.Yaw = (int16_t)(yaw * 100);
//			data.Pitch = (int16_t)(pitch * 100);
//			t += deltat;
//			delta.PidCsp(dis);
//		}
//		Sleep(delay);
//		DWORD end_time = GetTickCount();
//		runTime = end_time - start_time;
//	}
//}
//
//void MoveValPoint()
//{
//	chartTime += 0.067;
//	for (auto i = 0; i < CHART_POINT_NUM; ++i)
//	{
//		chartBottomAxisPoint[i] = chartBottomAxisPoint[i + 1];
//
//		chartXValPoint[i] = chartXValPoint[i + 1];
//		chartYValPoint[i] = chartYValPoint[i + 1];
//		chartZValPoint[i] = chartZValPoint[i + 1];
//		chartRollValPoint[i] = chartRollValPoint[i + 1];
//		chartPitchValPoint[i] = chartPitchValPoint[i + 1];
//		chartYawValPoint[i] = chartYawValPoint[i + 1];
//	}
//
//	chartBottomAxisPoint[CHART_POINT_NUM - 1] = chartTime;
//	chartXValPoint[CHART_POINT_NUM - 1] = data.X * XYZ_SCALE;
//	chartYValPoint[CHART_POINT_NUM - 1] = data.Y * XYZ_SCALE; 
//	chartZValPoint[CHART_POINT_NUM - 1] = data.Z * XYZ_SCALE;
//	chartRollValPoint[CHART_POINT_NUM - 1] = data.Roll * DEG_SCALE;
//	chartPitchValPoint[CHART_POINT_NUM - 1] = data.Pitch * DEG_SCALE;
//	chartYawValPoint[CHART_POINT_NUM - 1] = data.Yaw * DEG_SCALE;
//
//	m_ChartCtrl1.EnableRefresh(false);
//
//	pLineSerie1->ClearSerie();
//	pLineSerie2->ClearSerie();
//	pLineSerie3->ClearSerie();
//	pLineSerie4->ClearSerie();
//	pLineSerie5->ClearSerie();
//	pLineSerie6->ClearSerie();
//
//	pLineSerie1->AddPoints(chartBottomAxisPoint, chartXValPoint, CHART_POINT_NUM);
//	pLineSerie2->AddPoints(chartBottomAxisPoint, chartYValPoint, CHART_POINT_NUM);
//	pLineSerie3->AddPoints(chartBottomAxisPoint, chartZValPoint, CHART_POINT_NUM);
//	pLineSerie4->AddPoints(chartBottomAxisPoint, chartRollValPoint, CHART_POINT_NUM);
//	pLineSerie5->AddPoints(chartBottomAxisPoint, chartPitchValPoint, CHART_POINT_NUM);
//	pLineSerie6->AddPoints(chartBottomAxisPoint, chartYawValPoint, CHART_POINT_NUM);
//
//	m_ChartCtrl1.EnableRefresh(true);
//
//}
//
//CECATSampleDlg::CECATSampleDlg(CWnd* pParent /*=NULL*/)
//	: CDialog(CECATSampleDlg::IDD, pParent)
//{
//	m_hIcon = AfxGetApp()->LoadIcon(IDI_ICON1);
//}
//
//void CECATSampleDlg::DoDataExchange(CDataExchange* pDX)
//{
//	CDialog::DoDataExchange(pDX);
//	DDX_Control(pDX, IDC_CHART_CTRL, m_ChartCtrl1);
//	DDX_Control(pDX, IDC_EDIT_Sensor, Sensor_edit);
//}
//
//BEGIN_MESSAGE_MAP(CECATSampleDlg, CDialog)
//	//{{AFX_MSG_MAP(CECATSampleDlg)
//	ON_WM_PAINT()
//	ON_WM_QUERYDRAGICON()
//	ON_BN_CLICKED(IDC_BTN_InitialCard, OnBTNInitialCard)
//	ON_BN_CLICKED(IDC_BTN_FindSlave, OnBTNFindSlave)
//	ON_WM_TIMER()
//	ON_BN_CLICKED(IDC_CHK_SVON, OnChkSvon)
//	ON_BN_CLICKED(IDC_CHK_ABS, OnChkAbs)
//	//}}AFX_MSG_MAP
//	ON_BN_CLICKED(IDC_BTN_Rise, &CECATSampleDlg::OnBnClickedBtnRise)
//	ON_BN_CLICKED(IDC_BTN_Middle, &CECATSampleDlg::OnBnClickedBtnMiddle)
//	ON_BN_CLICKED(IDC_BTN_StopMe, &CECATSampleDlg::OnBnClickedBtnStopme)
//	ON_BN_CLICKED(IDC_BTN_Down, &CECATSampleDlg::OnBnClickedBtnDown)
//	ON_BN_CLICKED(IDOK, &CECATSampleDlg::OnBnClickedOk)
//	ON_BN_CLICKED(IDC_BTN_CONNECT, &CECATSampleDlg::OnBnClickedBtnConnect)
//	ON_BN_CLICKED(IDC_BTN_Resetme, &CECATSampleDlg::OnBnClickedBtnResetme)
//	ON_BN_CLICKED(IDC_BTN_DISCONNECT, &CECATSampleDlg::OnBnClickedBtnDisconnect)
//	ON_WM_SHOWWINDOW()
//	ON_BN_CLICKED(IDC_BTN_SINGLE_UP, &CECATSampleDlg::OnBnClickedBtnSingleUp)
//	ON_BN_CLICKED(IDC_BTN_SINGLE_DOWN, &CECATSampleDlg::OnBnClickedBtnSingleDown)
//	ON_WM_KEYDOWN()
//	ON_WM_CHAR()
//	ON_BN_CLICKED(IDC_BUTTON_TEST, &CECATSampleDlg::OnBnClickedButtonTest)
//	ON_BN_CLICKED(IDC_BUTTON_TEST3, &CECATSampleDlg::OnBnClickedButtonTest3)
//	ON_BN_CLICKED(IDC_BUTTON_STOP_TEST, &CECATSampleDlg::OnBnClickedButtonStopTest)
//	ON_BN_CLICKED(IDC_BUTTON_REPRODUCE, &CECATSampleDlg::OnBnClickedButtonReproduce)
//	ON_BN_CLICKED(IDC_BUTTON_SHOW_ROADDATA, &CECATSampleDlg::OnBnClickedButtonShowRoadData)
//	ON_BN_CLICKED(IDC_BUTTON_OIL_START, &CECATSampleDlg::OnBnClickedButtonOilStart)
//	ON_BN_CLICKED(IDC_BTN_Pause, &CECATSampleDlg::OnBnClickedBtnPause)
//	ON_BN_CLICKED(IDC_BTN_Recover, &CECATSampleDlg::OnBnClickedBtnRecover)
//END_MESSAGE_MAP()
//
//
//void CECATSampleDlg::ChartInit()
//{
//	CRect rect, rectChart;
//	GetDlgItem(IDC_CHART_CTRL)->GetWindowRect(&rect);
//	ScreenToClient(rect);
//	rectChart = rect;
//	rectChart.top = rect.bottom + 3;
//	rectChart.bottom = rectChart.top + rect.Height();
//
//	CChartAxis *pAxis = NULL;
//	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::BottomAxis);
//	pAxis->SetAutomatic(true);
//	pAxis = m_ChartCtrl1.CreateStandardAxis(CChartCtrl::LeftAxis);
//	pAxis->SetAutomatic(true);
//
//	m_ChartCtrl1.GetTitle()->AddString(_T(CHART_TITLE));
//
//	m_ChartCtrl1.
//		GetLeftAxis()->
//		GetLabel()->
//		SetText(_T(CHART_LEFT_AXIS_TITLE));
//
//	m_ChartCtrl1.EnableRefresh(false);
//
//	m_ChartCtrl1.RemoveAllSeries();
//	m_ChartCtrl1.GetLegend()->SetVisible(true);
//
//	pLineSerie1 = m_ChartCtrl1.CreateLineSerie();
//	pLineSerie1->SetSeriesOrdering(poNoOrdering);
//	pLineSerie1->SetWidth(2);
//	pLineSerie1->SetName(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
//	
//	pLineSerie2 = m_ChartCtrl1.CreateLineSerie();
//	pLineSerie2->SetSeriesOrdering(poNoOrdering);
//	
//	pLineSerie2->SetWidth(2);
//	pLineSerie2->SetName(_T(IDC_STATIC_Y_VAL_SHOW_TEXT)); 
//
//	pLineSerie3 = m_ChartCtrl1.CreateLineSerie();
//	pLineSerie3->SetSeriesOrdering(poNoOrdering);
//	pLineSerie3->SetWidth(2);
//	pLineSerie3->SetName(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));
//
//	pLineSerie4 = m_ChartCtrl1.CreateLineSerie();
//	pLineSerie4->SetSeriesOrdering(poNoOrdering);
//	pLineSerie4->SetWidth(2);
//	pLineSerie4->SetName(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));
//
//	pLineSerie5 = m_ChartCtrl1.CreateLineSerie();
//	pLineSerie5->SetSeriesOrdering(poNoOrdering); 
//	pLineSerie5->SetWidth(2);
//	pLineSerie5->SetName(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));
//
//	pLineSerie6 = m_ChartCtrl1.CreateLineSerie();
//	pLineSerie6->SetSeriesOrdering(poNoOrdering);  
//	pLineSerie6->SetWidth(2);
//	pLineSerie6->SetName(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));
//
//	m_ChartCtrl1.EnableRefresh(true);
//
//}
//
//void CECATSampleDlg::AppInit()
//{
//	int statusTemp = 0;
//	// 检测应用是否注册
//	auto isRegister = TestAppIsRegister();
//	if (isRegister == false)
//	{
//		DialogRegister * dlg = new DialogRegister(this);
//		dlg->DoModal();
//		dlg->Create(IDD_DIALOG_REGISTER, this);
//		dlg->ShowWindow(SW_SHOW);
//		delete dlg;
//		if (DialogRegister::IsRegister == false)
//		{
//			// 不注册就退出应用
//			CDialog::OnOK();
//		}
//		else
//		{
//			MessageBox(_T("注册成功！"));
//		}
//	}
//	// 读取上次应用退出时平台的状态
//	config::ReadStatusAndPulse(statusTemp, lastStartPulse);
//	// 读取停止并回中模式
//	stopAndMiddle = config::ReadIsAutoStopAndMiddle();
//	// 上次应用退出时平台的状态
//	lastStartStatus = (SixDofPlatformStatus)statusTemp;
//
//	SetDlgItemText(IDC_EDIT_X_VAL, _T("0"));
//	SetDlgItemText(IDC_EDIT_Y_VAL, _T("0"));
//	SetDlgItemText(IDC_EDIT_Z_VAL, _T("0"));
//	SetDlgItemText(IDC_EDIT_ROLL_VAL, _T("0"));
//	SetDlgItemText(IDC_EDIT_PITCH_VAL, _T("0"));
//	SetDlgItemText(IDC_EDIT_YAW_VAL, _T("0"));
//
//	SetDlgItemText(IDC_EDIT_X_HZ, _T("0"));
//	SetDlgItemText(IDC_EDIT_Y_HZ, _T("0"));
//	SetDlgItemText(IDC_EDIT_Z_HZ, _T("0"));
//	SetDlgItemText(IDC_EDIT_ROLL_HZ, _T("0"));
//	SetDlgItemText(IDC_EDIT_PITCH_HZ, _T("0"));
//	SetDlgItemText(IDC_EDIT_YAW_HZ, _T("0"));
//
//	SetDlgItemText(IDC_EDIT_X_PHASE, _T("0"));
//	SetDlgItemText(IDC_EDIT_Y_PHASE, _T("0"));
//	SetDlgItemText(IDC_EDIT_Z_PHASE, _T("0"));
//	SetDlgItemText(IDC_EDIT_ROLL_PHASE, _T("0"));
//	SetDlgItemText(IDC_EDIT_PITCH_PHASE, _T("0"));
//	SetDlgItemText(IDC_EDIT_YAW_PHASE, _T("0"));
//
//	SetDlgItemText(IDC_EDIT_X_ZERO_POS, _T("0"));
//	SetDlgItemText(IDC_EDIT_Y_ZERO_POS, _T("0"));
//	SetDlgItemText(IDC_EDIT_Z_ZERO_POS, _T("0"));
//	SetDlgItemText(IDC_EDIT_ROLL_ZERO_POS, _T("0"));
//	SetDlgItemText(IDC_EDIT_PITCH_ZERO_POS, _T("0"));
//	SetDlgItemText(IDC_EDIT_YAW_ZERO_POS, _T("0"));
//
//	CDialog::SetWindowTextW(_T(WINDOW_TITLE));
//	GetDlgItem(IDC_BTN_Start)->SetWindowTextW(_T(IDC_BTN_START_SHOW_TEXT));
//	GetDlgItem(IDC_BTN_Pause)->SetWindowTextW(_T(IDC_BTN_Pause_SHOW_TEXT));
//	GetDlgItem(IDC_BTN_Recover)->SetWindowTextW(_T(IDC_BTN_Recover_SHOW_TEXT));
//	GetDlgItem(IDC_BTN_SINGLE_UP)->SetWindowTextW(_T(IDC_BTN_SINGLE_UP_SHOW_TEXT));
//	GetDlgItem(IDC_BTN_SINGLE_DOWN)->SetWindowTextW(_T(IDC_BTN_SINGLE_DOWN_SHOW_TEXT));
//
//	GetDlgItem(IDC_BTN_CONNECT)->SetWindowTextW(_T(IDC_BTN_CONNECT_SHOW_TEXT));
//	GetDlgItem(IDC_BTN_DISCONNECT)->SetWindowTextW(_T(IDC_BTN_DISCONNECT_SHOW_TEXT));
//
//	GetDlgItem(IDC_STATIC_POSE)->SetWindowTextW(_T(IDC_STATIC_POSE_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_LENGTH)->SetWindowTextW(_T(IDC_STATIC_LENGTH_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_SENSOR)->SetWindowTextW(_T(IDC_STATIC_SENSOR_SHOW_TEXT));
//
//	GetDlgItem(IDC_STATIC_X_VAL)->SetWindowTextW(_T(IDC_STATIC_X_VAL_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Y_VAL)->SetWindowTextW(_T(IDC_STATIC_Y_VAL_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Z_VAL)->SetWindowTextW(_T(IDC_STATIC_Z_VAL_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_ROLL_VAL)->SetWindowTextW(_T(IDC_STATIC_ROLL_VAL_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_PITCH_VAL)->SetWindowTextW(_T(IDC_STATIC_PITCH_VAL_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_YAW_VAL)->SetWindowTextW(_T(IDC_STATIC_YAW_VAL_SHOW_TEXT));
//
//	GetDlgItem(IDC_STATIC_X_HZ)->SetWindowTextW(_T(IDC_STATIC_X_HZ_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Y_HZ)->SetWindowTextW(_T(IDC_STATIC_Y_HZ_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Z_HZ)->SetWindowTextW(_T(IDC_STATIC_Z_HZ_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_ROLL_HZ)->SetWindowTextW(_T(IDC_STATIC_ROLL_HZ_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_PITCH_HZ)->SetWindowTextW(_T(IDC_STATIC_PITCH_HZ_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_YAW_HZ)->SetWindowTextW(_T(IDC_STATIC_YAW_HZ_SHOW_TEXT));
//
//	GetDlgItem(IDC_STATIC_X_PHASE)->SetWindowTextW(_T(IDC_STATIC_X_PHASE_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Y_PHASE)->SetWindowTextW(_T(IDC_STATIC_Y_PHASE_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Z_PHASE)->SetWindowTextW(_T(IDC_STATIC_Z_PHASE_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_ROLL_PHASE)->SetWindowTextW(_T(IDC_STATIC_ROLL_PHASE_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_PITCH_PHASE)->SetWindowTextW(_T(IDC_STATIC_PITCH_PHASE_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_YAW_PHASE)->SetWindowTextW(_T(IDC_STATIC_YAW_PHASE_SHOW_TEXT));
//
//	GetDlgItem(IDC_STATIC_X_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_X_ZERO_POS_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Y_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_Y_ZERO_POS_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_Z_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_Z_ZERO_POS_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_ROLL_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_ROLL_ZERO_POS_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_PITCH_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_PITCH_ZERO_POS_SHOW_TEXT));
//	GetDlgItem(IDC_STATIC_YAW_ZERO_POS)->SetWindowTextW(_T(IDC_STATIC_YAW_ZERO_POS_SHOW_TEXT));
//
//	GetDlgItem(IDC_STATIC_TEST)->SetWindowTextW(_T(IDC_STATIC_TEST_SHOW_TEXT));
//	GetDlgItem(IDC_BUTTON_TEST)->SetWindowTextW(_T(IDC_BUTTON_TEST_SHOW_TEXT));
//
//	GetDlgItem(IDC_BUTTON_OIL_START)->SetWindowTextW(_T(IDC_BUTTON_OIL_START_SHOW_TEXT));
//	GetDlgItem(IDC_BUTTON_SET_MID_POS)->SetWindowTextW(_T(IDC_BUTTON_SET_MID_POS_SHOW_TEXT));
//	GetDlgItem(IDC_BUTTON_REPRODUCE)->SetWindowTextW(_T(IDC_BUTTON_REPRODUCE_SHOW_TEXT));
//	GetDlgItem(IDC_BUTTON_SHOW_ROADDATA)->SetWindowTextW(_T(IDC_BUTTON_SHOW_ROADDATA_SHOW_TEXT));
//
//	GetDlgItem(IDC_STATIC_APP_STATUS)->SetWindowTextW(_T(CORPORATION_NAME));
//	GetDlgItem(IDC_STATIC_APP_TITLE)->SetWindowTextW(_T(APP_TITLE));
//	CFont* font = new CFont();
//	font->CreatePointFont(APP_TITLE_FONT_SIZE, _T("Times New Roman"));
//	GetDlgItem(IDC_STATIC_APP_TITLE)->SetFont(font);
//	ChartInit();
//	for (auto i = 1; i <= AXES_COUNT; ++i)
//	{
//		CString xx;
//		xx.Format(_T("%d"), i);
//		((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->AddString(xx);
//	}
//	((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->SetCurSel(0);
//	SetPlatformPara(PlaneAboveHingeLength, PlaneAboveBottomLength, 
//		CircleTopRadius, CircleBottomRadius, DistanceBetweenHingeTop,
//		DistanceBetweenHingeBottom);
//	delta.ResetAllIoPorts();
//	OpenThread();
//}
//
//double CECATSampleDlg::GetCEditNumber(int cEditId)
//{
//	CString str;
//	GetDlgItemText(cEditId, str);
//	auto val = _tstof(str);
//	return val;
//}
//
//void CECATSampleDlg::OnShowWindow(BOOL bShow, UINT nStatus)
//{
//	CDialog::OnShowWindow(bShow, nStatus);
//}
//
//BOOL CECATSampleDlg::OnInitDialog()
//{
//	CDialog::OnInitDialog();
//
//	SetIcon(m_hIcon, TRUE);			
//	SetIcon(m_hIcon, FALSE);	
//	// 应用初始化
//	AppInit();
//	// openGl初始化
//	InitOpenGlControl();
//	SetTimer(0, TIMER_MS, NULL);
//	return TRUE;  
//}
//
//int isShowSingleUpDown = 0;
//BOOL CECATSampleDlg::PreTranslateMessage(MSG* pMsg)
//{
//	if (pMsg->message == WM_KEYDOWN)
//	{
//		 if (pMsg->wParam == VK_SHIFT)          //if (false)	
//		{
//			isShowSingleUpDown = !isShowSingleUpDown;
//			((CButton*)GetDlgItem(IDC_BTN_SINGLE_UP))->ShowWindow(isShowSingleUpDown);
//			((CButton*)GetDlgItem(IDC_BTN_SINGLE_DOWN))->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_BTN_FindSlave)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_EDIT_SlaveNum)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_CBO_SingleNo)->ShowWindow(isShowSingleUpDown);
//
//			GetDlgItem(IDC_CHK_SVON)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_BTN_Resetme)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_BTN_CONNECT)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_BTN_DISCONNECT)->ShowWindow(isShowSingleUpDown);
//
//			GetDlgItem(IDC_BTN_InitialCard)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_CBO_CardNo)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_STATIC_SHOW_INIT)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_EDIT_InitialStatus)->ShowWindow(isShowSingleUpDown);
//			GetDlgItem(IDC_SHOW)->ShowWindow(!isShowSingleUpDown);
//		}
//		if (pMsg->wParam == 'X')
//		{
//			closeDataThread = true;
//			delta.ServoStop();
//		}
//	}
//	else if (pMsg->message == WM_KEYUP)
//	{
//
//	}
//	return CDialog::PreTranslateMessage(pMsg);
//}
//
//BOOL CECATSampleDlg::PreCreateWindow(CREATESTRUCT& cs)
//{
//	cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
//	return CDialog::PreCreateWindow(cs);
//}
//
//void CECATSampleDlg::InitOpenGlControl()
//{
//	CWnd *pWnd = GetDlgItem(IDC_SHOW);
//	hrenderDC = ::GetDC(pWnd->GetSafeHwnd());
//	if(SetWindowPixelFormat(hrenderDC)==FALSE) 
//		return; 
//	if(CreateViewGLContext(hrenderDC)==FALSE) 
//		return; 
//	OpenGlLightInit();
//}
//
//BOOL CECATSampleDlg::SetWindowPixelFormat(HDC hDC) 
//{ 
//	PIXELFORMATDESCRIPTOR pixelDesc; 
//	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR); 
//	pixelDesc.nVersion = 1; 
//	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER | PFD_STEREO_DONTCARE; 
//	pixelDesc.iPixelType = PFD_TYPE_RGBA; 
//	pixelDesc.cColorBits = 32; 
//	pixelDesc.cRedBits = 8; 
//	pixelDesc.cRedShift = 16; 
//	pixelDesc.cGreenBits = 8; 
//	pixelDesc.cGreenShift = 8; 
//	pixelDesc.cBlueBits = 8; 
//	pixelDesc.cBlueShift = 0; 
//	pixelDesc.cAlphaBits = 0; 
//	pixelDesc.cAlphaShift = 0; 
//	pixelDesc.cAccumBits = 64; 
//	pixelDesc.cAccumRedBits = 16; 
//	pixelDesc.cAccumGreenBits = 16; 
//	pixelDesc.cAccumBlueBits = 16; 
//	pixelDesc.cAccumAlphaBits = 0; 
//	pixelDesc.cDepthBits = 32; 
//	pixelDesc.cStencilBits = 8; 
//	pixelDesc.cAuxBuffers = 0; 
//	pixelDesc.iLayerType = PFD_MAIN_PLANE; 
//	pixelDesc.bReserved = 0; 
//	pixelDesc.dwLayerMask = 0; 
//	pixelDesc.dwVisibleMask = 0; 
//	pixelDesc.dwDamageMask = 0; 
//	PixelFormat = ChoosePixelFormat(hDC,&pixelDesc); 
//	if(PixelFormat==0) 
//	{ 
//		PixelFormat = 1; 
//		if(DescribePixelFormat(hDC,PixelFormat, 
//			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc) == 0) 
//		{ 
//			return FALSE; 
//		} 
//	} 
//	if(SetPixelFormat(hDC,PixelFormat,&pixelDesc) == FALSE) 
//	{ 
//		return FALSE; 
//	} 
//	return TRUE; 
//} 
//
//BOOL CECATSampleDlg::CreateViewGLContext(HDC hDC) 
//{ 
//	hrenderRC = wglCreateContext(hDC); 
//	if(hrenderRC==NULL) 
//		return FALSE; 
//	if(wglMakeCurrent(hDC,hrenderRC)==FALSE) 
//		return FALSE; 
//	return TRUE; 
//} 
//
//void CECATSampleDlg::RenderScene()   
//{ 
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	OpenGL_SetData(data.X * XYZ_SCALE, data.Y * XYZ_SCALE, data.Z * XYZ_SCALE, 
//		data.Roll * DEG_SCALE, data.Yaw * DEG_SCALE, data.Pitch * DEG_SCALE);
//	RenderSixdofImage();
//	SwapBuffers(hrenderDC); 
//} 
//
//void CECATSampleDlg::FillCtlColor(CWnd* cwnd, COLORREF color)
//{
//	CDC * pDC = cwnd->GetDC();
//	CRect rct;
//	cwnd->GetWindowRect(&rct);
//	CBrush brs;
//	brs.CreateSolidBrush(color);
//	CRect picrct;
//	picrct.top = 0;
//	picrct.left = 0;
//	picrct.bottom = rct.Height();
//	picrct.right = rct.Width();
//	pDC->FillRect(&picrct, &brs);
//	pDC->ReleaseAttribDC();
//	cwnd->ReleaseDC(pDC);
//}
//
//void CECATSampleDlg::ShowSingleInitImage(int ctlId)
//{
//	CRect rect; 
//	GetDlgItem(ctlId)->GetClientRect(&rect);
//	Gdiplus::Graphics g(GetDlgItem(ctlId)->GetDC()->m_hDC);   
//	g.Clear(Gdiplus::Color::White);
//	g.DrawImage(GetPumpImage(0, _T("mm")), 0, 0, rect.Width(), rect.Height());
//}
//
//void CECATSampleDlg::ShowSingleInitImage(CWnd* pic, float value)
//{
//	CRect rect; 
//	pic->GetClientRect(&rect);
//	Gdiplus::Graphics g(pic->GetDC()->m_hDC);   
//	g.Clear(Gdiplus::Color::White);
//	g.DrawImage(GetPumpImage(value, _T("mm")), 0, 0, rect.Width(), rect.Height());
//}
//
//void CECATSampleDlg::ShowInitImage()
//{
//
//}
//
//CPen drawLinePen(PS_SOLID, 2, RGB(0, 0, 0));
//CPen clearLinePen(PS_SOLID, 10, RGB(244, 244, 244));
//CBrush clearBrush(RGB(244, 244, 244));
//double ptCenterX = 52;
//double ptCenterY = 52;
//double f0;
//double p0X;
//double p0Y;
//double p1X;
//double p1Y;
////	张开的弧度
//float radian=60;
////	同心圆半径
//int radius[]=
//{
//	50,
//	24,
//	1
//};
//
//double fMax = 100;
//double fMin = -100;
//
//void CECATSampleDlg::ShowSingleImage(CWnd* pic, float value)
//{
//	value *= 2;
//	CDC* dc = pic->GetDC();   
//	f0 = ((180-radian)/2+radian/(fMax-fMin)*(-value + fMax)) / 180 * PI;
//	dc->SelectObject(&clearBrush);
//	dc->FillSolidRect(2,28,100,32, RGB(255, 255, 255));
//	p0X = ptCenterX+(radius[2]*cos(f0));
//	p0Y = ptCenterY-(radius[2]*sin(f0));
//	p1X = ptCenterX+((radius[1])*cos(f0));
//	p1Y = ptCenterY-((radius[1])*sin(f0));
//	dc->SelectObject(&drawLinePen);
//	dc->MoveTo(p0X, p0Y);
//	dc->LineTo(p1X, p1Y);
//	pic->ReleaseDC(dc);
//}
//
//void CECATSampleDlg::ShowImage()
//{
//
//}
//
//void CECATSampleDlg::RenderSwitchStatus()
//{
//
//}
//
//void CECATSampleDlg::OnPaint() 
//{
//	if (IsIconic())
//	{
//		CPaintDC dc(this); 
//
//		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);
//
//		int cxIcon = GetSystemMetrics(SM_CXICON);
//		int cyIcon = GetSystemMetrics(SM_CYICON);
//		CRect rect;
//		GetClientRect(&rect);
//		int x = (rect.Width() - cxIcon + 1) / 2;
//		int y = (rect.Height() - cyIcon + 1) / 2;
//
//		dc.DrawIcon(x, y, m_hIcon);		
//	}
//	else
//	{
//		CDialog::OnPaint();
//	}
//}
//
//HCURSOR CECATSampleDlg::OnQueryDragIcon()
//{
//	return (HCURSOR) m_hIcon;
//}
//
//void CECATSampleDlg::OnBTNInitialCard() 
//{
//
//}
//
//void CECATSampleDlg::OnBTNFindSlave() 
//{
//	
//}
//
//void CECATSampleDlg::EanbleButton(int isenable)
//{	
//	((CButton*) GetDlgItem(IDC_CHK_SVON))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_Rise))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_Middle))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_Start))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_StopMe))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_Down))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_Pause))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_Recover))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BTN_Resetme))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BUTTON_TEST))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BUTTON_STOP_TEST))->EnableWindow(isenable);
//
//	((CButton*) GetDlgItem(IDC_BUTTON_SET_MID_POS))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BUTTON_REPRODUCE))->EnableWindow(isenable);
//	((CButton*) GetDlgItem(IDC_BUTTON_SHOW_ROADDATA))->EnableWindow(isenable);
//
//	((CEdit*) GetDlgItem(IDC_EDIT_X_VAL))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Y_VAL))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Z_VAL))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_X_HZ))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Y_HZ))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Z_HZ))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_X_PHASE))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Y_PHASE))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Z_PHASE))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_X_ZERO_POS))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Y_ZERO_POS))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_Z_ZERO_POS))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_VAL))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_VAL))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_YAW_VAL))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_HZ))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_HZ))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_YAW_HZ))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_PHASE))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_PHASE))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_YAW_PHASE))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_ROLL_ZERO_POS))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_YAW_ZERO_POS))->EnableWindow(isenable);
//	((CEdit*) GetDlgItem(IDC_EDIT_PITCH_ZERO_POS))->EnableWindow(isenable);
//
//}
//
//CString statusStr = "";
//
//void CECATSampleDlg::OnTimer(UINT nIDEvent) 
//{
//	MoveValPoint();
//	RenderScene();
//	PlatTimerFunc();
//	//RenderSwitchStatus();
//	//ShowImage();
//	statusStr.Format(_T("x:%d y:%d z:%d y:%d a:%d b:%d time:%.2f count:%d"), data.X, data.Y, data.Z,
//		data.Yaw, data.Pitch, data.Roll, runTime, 0);
//	SetDlgItemText(IDC_EDIT_Pose, statusStr);
//
//	delta.GetNowPulse(cardVols);
//	statusStr.Format(_T("1:%.2f 2:%.2f 3:%.2f 4:%.2f 5:%.2f 6:%.2f"), 
//		cardVols[0], cardVols[1], cardVols[2],
//		cardVols[3], cardVols[4], cardVols[5]);
//	SetDlgItemText(IDC_EDIT_Sensor, statusStr);
//
//	for (int i = 0;i < AXES_COUNT;++i)
//	{
//		cardPoleLength[i] = cardVols[i] * PULSE_COUNT_TO_MM_SCALE;
//	}
//
//	statusStr.Format(_T("1:%.2f 2:%.2f 3:%.2f 4:%.2f 5:%.2f 6:%.2f"), 
//		cardPoleLength[0], cardPoleLength[1], cardPoleLength[2],
//		cardPoleLength[3], cardPoleLength[4], cardPoleLength[5]);
//	SetDlgItemText(IDC_EDIT_Pulse, statusStr);
//
//	if (++fileCount >= 10) {
//		fileCount = 0;
//		fileSave.Record(data.X, data.Y, data.Z, data.Roll, data.Yaw, data.Pitch, 
//			poleLength, cardVols);
//	}
//
//	CDialog::OnTimer(nIDEvent);
//}
//
//void CECATSampleDlg::OnChkSvon() 
//{
//	
//}
//
//void CECATSampleDlg::OnOK() 
//{
//	delta.ServoStop();
//	Sleep(100);
//	delta.Close(status);
//	CDialog::OnOK();
//}
//
//void CECATSampleDlg::OnChkAbs() 
//{
//	
//}
//
//void CECATSampleDlg::OnBnClickedBtnRise()
//{	
//	if (status != SIXDOF_STATUS_BOTTOM)
//	{
//#if IS_USE_MESSAGE_BOX
//		MessageBox(_T(SIXDOF_NOT_BOTTOM_MESSAGE));
//#endif	
//		return;
//	}
//	Sleep(20);
//	status = SIXDOF_STATUS_ISRISING;	
//	delta.Rise();
//	Sleep(20);
//	status = SIXDOF_STATUS_READY;
//}
//
//void CECATSampleDlg::OnBnClickedBtnMiddle()
//{
//	if (stopAndMiddle == true)
//	{
//		if (status != SIXDOF_STATUS_READY)
//		{
//#if IS_USE_MESSAGE_BOX
//			MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
//#endif
//			return;
//		}
//	}
//	else
//	{
//		if (status != SIXDOF_STATUS_MIDDLE)
//		{
//#if IS_USE_MESSAGE_BOX
//			MessageBox(_T(SIXDOF_NOT_MIDDLE_MESSAGE));
//#endif	
//			return;
//		}
//	}
//	status = SIXDOF_STATUS_READY;
//	delta.MoveToZeroPulseNumber();
//}
//
//void CECATSampleDlg::OnBnClickedBtnStart()
//{
//	if (status != SIXDOF_STATUS_READY)
//	{
//#if IS_USE_MESSAGE_BOX
//		MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
//#endif
//		return;
//	}
//	status = SIXDOF_STATUS_RUN;
//	delta.ServoStop();
//	Sleep(100);
//	delta.RenewNowPulse();
//	delta.GetMotionAveragePulse();
//	delta.PidControllerInit();
//	// 正常使用模式
//	isTest = false;
//	isCosMode = false;
//	t = 0;
//	closeDataThread = false;
//}
//
//void CECATSampleDlg::OnCommandStopme()
//{
//	if (status != SIXDOF_STATUS_RUN)
//	{
//		return;
//	}
//	status = SIXDOF_STATUS_READY;
//	closeDataThread = true;
//	delta.servoCurveStopThread();
//	delta.MoveToZeroPulseNumber();
//	ResetDefaultData(&data);
//}
//
//void CECATSampleDlg::OnBnClickedBtnStopme()
//{
//	// 停止Csp运动
//	stopSCurve = true;
//	closeDataThread = true;
//	if (status == SIXDOF_STATUS_RUN)
//	{
//		delta.servoCurveStopThread();
//		status = SIXDOF_STATUS_READY;
//		OnBnClickedBtnDown();
//	}
//	ResetDefaultData(&data);
//}
//
//void CECATSampleDlg::OnBnClickedBtnDown()
//{	
//	if (status == SIXDOF_STATUS_RUN || status == SIXDOF_STATUS_PAUSING) {
//#if IS_USE_MESSAGE_BOX
//		MessageBox(_T(SIXDOF_NOT_FALLING_MESSAGE));
//#endif
//		return;
//	}
//	// status = SIXDOF_STATUS_ISFALLING;
//	delta.SetEnable(true);
//	delta.StopRiseDownMove();
//	Sleep(100);
//	delta.Down();
//	status = SIXDOF_STATUS_BOTTOM;
//}
//
//void CECATSampleDlg::OnBnClickedOk()
//{
//	delta.ServoStop();
//	delta.Close(status);
//	Sleep(50);
//	if (finishFlag != 0)
//		return;
//	((CButton*) GetDlgItem(IDOK))->EnableWindow(0);
//	GetDlgItem(IDOK)->SetWindowTextW(_T(IDC_BUTTON_FINISH_ING_SHOW_TEXT));
//	thread td(PlatformCloseThread);
//	td.detach();
//}
//
//void CECATSampleDlg::OnBnClickedBtnConnect()
//{
//	delta.AllTestUp();
//}
//
//void CECATSampleDlg::OnBnClickedBtnResetme()
//{
//	
//}
//
//void CECATSampleDlg::OnBnClickedBtnDisconnect()
//{
//	delta.AllTestDown();
//}
//
//void CECATSampleDlg::OnBnClickedBtnSingleUp()
//{
//	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
//	delta.SingleUp(index);
//}
//
//void CECATSampleDlg::OnBnClickedBtnSingleDown()
//{
//	auto index = ((CComboBox*)GetDlgItem(IDC_CBO_SingleNo))->GetCurSel();
//	delta.SingleDown(index);
//}
//
//void CECATSampleDlg::OnBnClickedButtonTest()
//{
//	int valid = 0;
//	if (status != SIXDOF_STATUS_READY)
//	{
//#if IS_USE_MESSAGE_BOX
//		MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
//#endif
//		return;
//	}
//	status = SIXDOF_STATUS_RUN;
//
//	memset(testPhase, 0, sizeof(double) * AXES_COUNT);
//	//位移单位mm 角度单位 度
//	auto xval = RANGE(GetCEditNumber(IDC_EDIT_X_VAL), -MAX_XYZ_X, MAX_XYZ_X); 
//	auto yval = RANGE(GetCEditNumber(IDC_EDIT_Y_VAL), -MAX_XYZ_Y, MAX_XYZ_Y);
//	auto zval = RANGE(GetCEditNumber(IDC_EDIT_Z_VAL), -MAX_XYZ_Z, MAX_XYZ_Z);
//	auto rollval = RANGE(GetCEditNumber(IDC_EDIT_ROLL_VAL), -MAX_DEG_ROLL, MAX_DEG_ROLL);
//	auto pitchval = RANGE(GetCEditNumber(IDC_EDIT_PITCH_VAL), -MAX_DEG_PITCH, MAX_DEG_PITCH);
//	auto yawval = RANGE(GetCEditNumber(IDC_EDIT_YAW_VAL), -MAX_DEG_YAW, MAX_DEG_YAW);
//	//频率单位1hz
//	auto xhz = RANGE(GetCEditNumber(IDC_EDIT_X_HZ), 0, MAX_HZ);
//	auto yhz = RANGE(GetCEditNumber(IDC_EDIT_Y_HZ), 0, MAX_HZ);
//	auto zhz = RANGE(GetCEditNumber(IDC_EDIT_Z_HZ), 0, MAX_HZ);
//	auto rollhz = RANGE(GetCEditNumber(IDC_EDIT_ROLL_HZ), 0, MAX_HZ);
//	auto pitchhz = RANGE(GetCEditNumber(IDC_EDIT_PITCH_HZ), 0, MAX_HZ);
//	auto yawhz = RANGE(GetCEditNumber(IDC_EDIT_YAW_HZ), 0, MAX_HZ);
//
//	auto xphase = RANGE(GetCEditNumber(IDC_EDIT_X_PHASE), 0, MAX_PHASE);
//	auto yphase = RANGE(GetCEditNumber(IDC_EDIT_Y_PHASE), 0, MAX_PHASE);
//	auto zphase = RANGE(GetCEditNumber(IDC_EDIT_Z_PHASE), 0, MAX_PHASE);
//	auto rollphase = RANGE(GetCEditNumber(IDC_EDIT_ROLL_PHASE), 0, MAX_PHASE);
//	auto pitchphase = RANGE(GetCEditNumber(IDC_EDIT_PITCH_PHASE), 0, MAX_PHASE);
//	auto yawphase = RANGE(GetCEditNumber(IDC_EDIT_YAW_PHASE), 0, MAX_PHASE);
//
//	auto xzeropos = RANGE(GetCEditNumber(IDC_EDIT_X_ZERO_POS), -MAX_XYZ_ZERO_POS_X, MAX_XYZ_ZERO_POS_X);
//	auto yzeropos = RANGE(GetCEditNumber(IDC_EDIT_Y_ZERO_POS), -MAX_XYZ_ZERO_POS_Y, MAX_XYZ_ZERO_POS_Y);
//	auto zzeropos = RANGE(GetCEditNumber(IDC_EDIT_Z_ZERO_POS), -MAX_XYZ_ZERO_POS_Z, MAX_XYZ_ZERO_POS_Z);
//	auto rollzeropos = RANGE(GetCEditNumber(IDC_EDIT_ROLL_ZERO_POS), -MAX_DEG_ZERO_POS_PITCH, MAX_DEG_ZERO_POS_PITCH);
//	auto pitchzeropos = RANGE(GetCEditNumber(IDC_EDIT_PITCH_ZERO_POS), -MAX_DEG_ZERO_POS_ROLL, MAX_DEG_ZERO_POS_ROLL);
//	auto yawzeropos = RANGE(GetCEditNumber(IDC_EDIT_YAW_ZERO_POS), -MAX_DEG_ZERO_POS_YAW, MAX_DEG_ZERO_POS_YAW);
//
//	testVal[0] = xval;
//	valid += abs(testVal[0]) < MAX_XYZ_X ? 0 : 1;
//	testVal[1] = yval;
//	valid += abs(testVal[1]) < MAX_XYZ_Y ? 0 : 1;
//	testVal[2] = zval;
//	valid += abs(testVal[2]) < MAX_XYZ_Z ? 0 : 1;
//	testVal[3] = rollval;
//	valid += abs(testVal[3]) < MAX_DEG_ROLL ? 0 : 1;
//	testVal[4] = pitchval;
//	valid += abs(testVal[4]) < MAX_DEG_PITCH ? 0 : 1;
//	testVal[5] = yawval;
//	valid += abs(testVal[5]) < MAX_DEG_YAW ? 0 : 1;
//	if(valid != 0)
//	{
//		MessageBox(_T("参数超限请重新输入"));
//		return ;
//	}
//
//	testHz[0] = xhz;
//	testHz[1] = yhz;
//	testHz[2] = zhz;
//	testHz[3] = rollhz;
//	testHz[4] = pitchhz;
//	testHz[5] = yawhz;
//
//	testPhase[0] = xphase;
//	testPhase[1] = yphase;
//	testPhase[2] = zphase;
//	testPhase[3] = rollphase;
//	testPhase[4] = pitchphase;
//	testPhase[5] = yawphase;
//
//	testZeroPos[0] = xzeropos;
//	testZeroPos[1] = yzeropos;
//	testZeroPos[2] = zzeropos;
//	testZeroPos[3] = rollzeropos;
//	testZeroPos[4] = pitchzeropos;
//	testZeroPos[5] = yawzeropos;
//
//	if (xphase != 0 || yphase != 0 || zphase != 0 || 
//		rollphase != 0 || pitchphase != 0 || yawphase != 0)
//	{
//		enableChirp = false;
//		isCosMode = true;
//	}
//	else
//	{
//		enableChirp = ENABLE_CHIRP;
//		isCosMode = false;
//	}
//	stopSCurve = false;
//
//	// 电机先停后启动
//	delta.StopRiseDownMove();
//	delta.RenewNowPulse();
//	delta.GetMotionAveragePulse();
//	delta.PidControllerInit();
//	// 正弦测试运动模式
//	isTest = true;
//	// 正弦时间清0
//	t = 0;
//	// 允许运动
//	closeDataThread = false;
//}
//
//
//void CECATSampleDlg::OnBnClickedButtonTest3()
//{
//	delta.TestHardware();
//}
//
//void CECATSampleDlg::OnBnClickedButtonStopTest()
//{
//	OnBnClickedBtnStopme();
//}
//
//void CECATSampleDlg::DataFromFile()
//{
//	CString targetHistoryPath = "";
//	CString defaultDir = L"C:\\";  //默认打开的文件路径
//	CString defaultFile = L"test.txt"; //默认打开的文件名
//	CFileDialog dlg(TRUE, _T("txt"), defaultDir + "\\" + defaultFile, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, _T("数据文件|*.txt||"));
//	if (dlg.DoModal() == IDOK)
//	{
//		targetHistoryPath = dlg.GetPathName();
//		if (targetHistoryPath == "")
//		{
//			MessageBox(_T("未选择文件！"));
//			return;
//		}
//	}
//
//	ifstream fin(targetHistoryPath);		
//	double *ptr = &SourceBuf[0]; 
//	int readcount = 0;
//	int arrcount = 0;
//	if (!fin.is_open())
//	{
//		AfxMessageBox(_T("未找到文件!"));
//		return;
//	}
//	roadSpectrum.DataBuffer.clear();
//	while (!fin.eof() && readcount < MAX_REPRODUCE_LINE * DATA_COL_NUM)
//	{
//		fin >> *ptr; 
//		ptr++;
//		readcount++;
//		arrcount++;
//		if (arrcount >= DATA_COL_NUM)
//		{
//			arrcount = 0;
//			roadSpectrum.DataBuffer.push_back(Signal::RoadSpectrumData::FromArray(SourceBuf));
//			ptr = &SourceBuf[0];
//		}
//	}
//	fin.close();
//}
//
//void CECATSampleDlg::OnBnClickedButtonReproduce()
//{
//#if _DEBUG
//#else
//	if (status != SIXDOF_STATUS_READY)
//	{
//#if IS_USE_MESSAGEBOX
//		MessageBox(_T(SIXDOF_NOT_BEGIN_MESSAGE));
//#endif
//		return;
//	}
//#endif
//	DataFromFile();
//	Sleep(20);
//	if (roadSpectrum.DataBuffer.size() <= 0)
//		return;
//#if _DEBUG
//	status = SIXDOF_STATUS_READY;
//#endif
//	OnBnClickedBtnStart();
//}
//
//void CECATSampleDlg::OnBnClickedButtonShowRoadData()
//{
//	DataFromFile();
//	isTest = false;
//	t = 0;
//	closeDataThread = false;
//}
//
//void CECATSampleDlg::OnBnClickedButtonOilStart()
//{
//	if (startFlag != 0)
//		return;
//	((CButton*) GetDlgItem(IDC_BUTTON_OIL_START))->EnableWindow(0);
//	GetDlgItem(IDC_BUTTON_OIL_START)->SetWindowTextW(_T(IDC_BUTTON_OIL_ING_START_SHOW_TEXT));
//	thread td(PlatformInitThread);
//	td.detach();
//}
//
//void CECATSampleDlg::OnBnClickedBtnPause()
//{
//	if (status == SIXDOF_STATUS_RUN)
//	{
//		stopSCurve = true;
//		closeDataThread = true;
//		Sleep(10);
//		delta.servoCurveStopThread();
//		delta.ServoStop();
//		Sleep(10);
//		delta.SetEnable(false);
//		status = SIXDOF_STATUS_PAUSING;
//	}
//}
//
//void CECATSampleDlg::OnBnClickedBtnRecover()
//{
//	if (status == SIXDOF_STATUS_PAUSING)
//	{
//		status = SIXDOF_STATUS_RUN;
//		delta.PidControllerInit();
//		Sleep(10);
//		delta.SetEnable(true);
//		Sleep(10);
//		stopSCurve = false;
//		closeDataThread = false;	
//	}
//}
