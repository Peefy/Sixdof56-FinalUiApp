
#ifndef __NOISE_H
#define __NOISE_H

#include <stdlib.h>
#include <time.h>

#include "signal.h"

namespace Signal
{
	typedef enum 
	{
		NoiseTypeRandom, 
		NoiseTypeGaussian,
	}NoiseType;

	class NoiseGenerator
	{
	public:
		NoiseGenerator();
		~NoiseGenerator();
		SixdofData GenerateNoneNoise();
		SixdofData GenerateNoise(SixdofData val);
		SixdofData GenerateNoise(double val);
		SixdofData GenerateGaussianNoise(double val);
	private:
		NoiseType type;
		double dBval;
	};

}

#endif
