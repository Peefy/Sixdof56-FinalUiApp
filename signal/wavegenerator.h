
#ifndef __WAVE_GENERATOR_H
#define __WAVE_GENERATOR_H

#include <math.h>

#include "signal.h"
#include "noise.h"
#include "roadspectrum.h"

namespace Signal
{
	class WaveGenerator
	{
	public:
		WaveGenerator();
		~WaveGenerator();
		SixdofData Amp;
		SixdofData Freq;
		void SetSampleTime(double time);
		double GetSampleTime();
		void ClearCountTime();
		RoadSpectrumData GetNext();
	private:
		NoiseGenerator noiseGenerator;
		RoadSpectrumData data;
		RoadSpectrumData lastdata;
		double deltat;
		double t;
	};

}

#endif
