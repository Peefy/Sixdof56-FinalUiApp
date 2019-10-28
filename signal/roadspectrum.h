
#ifndef __ROAD_SPECTRUM_H
#define __ROAD_SPECTRUM_H

#include <queue>
#include <deque>
#include <vector>
#include <memory>
#include <string>

#include "signal.h"

using namespace std;

namespace Signal
{


struct RoadSpectrumData
{
	SixdofData Position; //位移
	SixdofData Speed;    //速度
	SixdofData Acc;      //加速度

	RoadSpectrumData(double x, double y, double z, 
		double yaw, double roll, double pitch)  
	{
		Position.X = x;
		Position.Y = y;
		Position.Z = z;
		Position.Yaw = yaw;
		Position.Roll = roll;
		Position.Pitch = pitch;	
	}

	RoadSpectrumData() {}

	virtual ~RoadSpectrumData() {}

	void SetPositions(double x, double y, double z, 
		double yaw, double roll, double pitch)  
	{
		Position.X = x;
		Position.Y = y;
		Position.Z = z;
		Position.Yaw = yaw;
		Position.Roll = roll;
		Position.Pitch = pitch;	
	}

	void SetSpeeds(double x, double y, double z, 
		double yaw, double roll, double pitch)  
	{
		Speed.X = x;
		Speed.Y = y;
		Speed.Z = z;
		Speed.Yaw = yaw;
		Speed.Roll = roll;
		Speed.Pitch = pitch;	
	}

	void SetAccs(double x, double y, double z, 
		double yaw, double roll, double pitch)  
	{
		Acc.X = x;
		Acc.Y = y;
		Acc.Z = z;
		Acc.Yaw = yaw;
		Acc.Roll = roll;
		Acc.Pitch = pitch;	
	}

	string ToString() 
	{
		return Position.ToString() + 
			Speed.ToString() + 
			Acc.ToString();
	}

	static RoadSpectrumData FromString()
	{
		return RoadSpectrumData();
	}

	// ser sixdof length = 6
	static RoadSpectrumData FromArray(double* arr)
	{
		return RoadSpectrumData(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]);
	}

};

class RoadSpectrum
{
public:
	RoadSpectrum();
	virtual ~RoadSpectrum();
	deque<RoadSpectrumData> DataBuffer;
	int OperateMode;
private:
	
};

}

#endif
