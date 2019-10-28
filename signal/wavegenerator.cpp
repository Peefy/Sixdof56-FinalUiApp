
#include "stdafx.h"
#include "wavegenerator.h"

namespace Signal
{

	static const double pi = 3.1415926;

	WaveGenerator::WaveGenerator() : 
		deltat(DEFAULT_DT), 
		t(0)
	{
	}

	WaveGenerator::~WaveGenerator()
	{
	}

	void WaveGenerator::SetSampleTime(double time)
	{
		deltat = time;
	}

	double WaveGenerator::GetSampleTime()
	{
		return deltat;
	}

	void WaveGenerator::ClearCountTime()
	{
		t = 0;
	}

	RoadSpectrumData WaveGenerator::GetNext()
	{
		auto noise = noiseGenerator.GenerateNoneNoise();
		auto x = Amp.X * sin(2 * pi * Freq.X * t) + noise.X;
		auto y = Amp.Y * sin(2 * pi * Freq.Y * t) + noise.Y;
		auto z = Amp.Z * sin(2 * pi * Freq.Z * t) + noise.Z;
		auto yaw = Amp.Yaw * sin(2 * pi * Freq.Yaw * t) + noise.Yaw;
		auto roll = Amp.Roll * sin(2 * pi * Freq.Roll * t) + noise.Roll;
		auto pitch = Amp.Pitch * sin(2 * pi * Freq.Pitch * t) + noise.Pitch;

		auto xspeed = (x - lastdata.Position.X) / deltat;
		auto yspeed = (y - lastdata.Position.Y) / deltat;
		auto zspeed = (z - lastdata.Position.Z) / deltat;
		auto yawspeed = (yaw - lastdata.Position.Yaw) / deltat;
		auto rollspeed = (roll - lastdata.Position.Roll) / deltat;
		auto pitchspeed = (pitch - lastdata.Position.Pitch) / deltat;

		auto xacc = (xspeed - lastdata.Speed.X) / deltat;
		auto yacc = (yspeed - lastdata.Speed.Y) / deltat;
		auto zacc = (zspeed - lastdata.Speed.Z) / deltat;
		auto yawacc = (yawspeed - lastdata.Speed.Yaw) / deltat;
		auto rollacc = (rollspeed - lastdata.Speed.Roll) / deltat;
		auto pitchacc = (pitchspeed - lastdata.Speed.Pitch) / deltat;

		t += deltat;
		data.SetPositions(x, y, z, yaw, roll, pitch);
		data.SetSpeeds(xspeed, yspeed, zspeed, yawspeed, rollspeed, pitchspeed);
		data.SetAccs(xacc, yacc, zacc, yawacc, rollacc, pitchacc);
		lastdata = data;
		return data;
	}

}


