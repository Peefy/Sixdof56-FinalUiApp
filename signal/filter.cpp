
#include "stdafx.h"
#include "filter.h"


namespace Signal
{

	double RateLimiter(double dataIn, double risingSlewRate, double fallingSlewRate, double dT)
	{
		static double lastOut = dataIn;
		if (dT == 0)
			return dataIn;
		double rate = (dataIn - lastOut) / dT;
		double out = dataIn;
		if (rate >= risingSlewRate)
		{
			out = risingSlewRate * dT + lastOut;
		}
		else if (rate <= fallingSlewRate)
		{
			out = fallingSlewRate * dT + lastOut;
		}
		lastOut = out;
		return dataIn;
	}

	static int fac(int m)
	{
		if (m == 1 || m == 0)
		{
			return 1;
		}
		else
		{
			return fac(m - 1) * m;
		}
	}

	static int comb(int m, int n)
	{
		if (n == 0 || n == m)   //取边界条件
		{
			return 1;
		}
		else
		{
			return fac(m) / (fac(n) * fac(m - n));    //调用递归运算
		}
	}

	Ztrans::Ztrans(double dT, int order)
	{
		this->dT = dT;
		this->fs = 1.0 / dT; 
		Order = order + 1;
		for (int i = 0; i < Order; i++)
		{
			input.push_front(0);
			output.push_front(0);
		}
	}

	Ztrans::Ztrans(double dT, int order, double* nums, double* dens)
	{
		this->dT = dT;
		Order = order + 1;
		for (int i = 0; i < Order; i++)
		{
			input.push_front(0);
			output.push_front(0);
		}
		SetNumsAndDensZtrans(nums, dens);
	}

	void Ztrans::SetNumsAndDensZtrans(double* nums, double* dens)
	{
		if (nums == nullptr || dens == nullptr)
		{
			return;
		}
		memcpy(inner_nums, nums, sizeof(double) * Order);
		memcpy(inner_dens, dens, sizeof(double) * Order);
	}

	void Ztrans::SetNumsAndDensLaplace(double* nums, double* dens, double fs)
	{
		if (nums == nullptr || dens == nullptr)
		{
			return;
		}
		Bilinear(nums, dens, fs, Order, inner_nums, inner_dens);
	}

	void Ztrans::Bilinear(PARA_IN double* b, PARA_IN double* a, double fs, int dimensions, 
		PARA_OUT double* bprimes, PARA_OUT double* aprimes)
	{
		int Np = dimensions - 1;
		int Dp = dimensions - 1;
		int D = dimensions - 1;
		int N = dimensions - 1;
		int M = dimensions - 1;

		for (int j = 0; j < Np + 1; ++j){
			double val = 0.0;
			for (int i = 0; i < N + 1; ++i){
				for (int k = 0; k < i + 1; ++k){
					for (int l = 0; l < M - i + 1; ++l){
						if (k + l == j){
							val += (comb(i, k) * comb(M - i, l) * b[N - i] * 
								pow(2 * fs, i) * pow((double)-1, k));
						}
					}
				}
			}
			bprimes[j] = val;
		}

		for (int j = 0; j < Dp + 1; ++j){
			double val = 0.0;
			for (int i = 0; i < D + 1; ++i){
				for (int k = 0; k < i + 1; ++k){
					for (int l = 0; l < M - i + 1; ++l){
						if (k + l == j){
							val += (comb(i, k) * comb(M - i, l) * a[D - i] *
								pow(2 * fs, i) * pow((double)-1, k));
						}
					}
				}
			}
			aprimes[j] = val;
		}
		double div = aprimes[0];
		for (int i = 0;i < dimensions;++i)
		{
			bprimes[i] /= div;
			aprimes[i] /= div;
		}
	}

	void Ztrans::SetSampleTime(double sampleTime)
	{
		dT = sampleTime;
		fs = 1.0 / (double)dT;
	}

	double Ztrans::Update(double now)
	{
		double out = 0;
		input.push_front(now);
		input.pop_back();
		for (int i = 0; i < Order - 1; ++i)
		{
			out -= inner_dens[i + 1] * output[i];
		}
		for (int i = 0; i < Order; ++i)
		{
			out += inner_nums[i] * input[i];
		}
		output.push_front(out);
		output.pop_back();
		return out;
	}

	SignalLpf::SignalLpf(double dT) : Ztrans(dT, 10)
	{
		double nums[11] = 
			{
				0.18640591694089509, 
				0.029222173975752764, 
				0.030883795829023255,
				0.032081728777655513,                
				0.032710287230081583,                 
				0.033126092339065105,                 
				0.032710287230081583,                 
				0.032081728777655513,                 
				0.030883795829023255,                 
				0.029222173975752764,                 
				0.18640591694089509, 
			};
		// FIR滤波器的分母为1
		double dens[11] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		SetNumsAndDensZtrans(nums, dens);
	}


	ButterworthFilter::ButterworthFilter()
	{
	}

	ButterworthFilter::~ButterworthFilter()
	{
	}

}
