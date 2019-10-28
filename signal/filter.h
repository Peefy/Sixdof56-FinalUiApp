
#ifndef __FILTER_H
#define __FILTER_H

#include <queue>
#include <vector>
#include <deque>

#include "signal.h"

using namespace std;

// 标识数据输入
#define PARA_IN
// 标识数据输出
#define PARA_OUT
// 滤波器最大阶数
#define MAX_ORDER 10

namespace Signal
{

	// z变换传递函数
	class Ztrans
	{
	public:
		Ztrans(double dT, int order, double* nums, double* dens);
		Ztrans(double dT, int order);
		~Ztrans() {};
		int Order;
		// 设置采样周期
		void SetSampleTime(double sampleTime);
		// 输入经传递函数得到输出
		double Update(double now);
		// 设置z传递函数的分子分母系数
		void SetNumsAndDensZtrans(double* nums, double* dens);
		// 设置s传递函数的分子分母系数
		void SetNumsAndDensLaplace(double* nums, double* dens, double fs);
		// s域到z域的双线性变换
		void Bilinear(PARA_IN double* b, PARA_IN double* a, double fs, int dimensions, 
			PARA_OUT double* bprimes, PARA_OUT double* aprimes);
	private:
		double dT;
		double fs;
		deque<double> input;
		deque<double> output;
		double inner_nums[MAX_ORDER];
		double inner_dens[MAX_ORDER];
	};

	// 信号低通滤波器
	class SignalLpf : public Ztrans
	{
	public:
		SignalLpf(double dT = DEFAULT_DT); 
		~SignalLpf() {};

	private:
		double lowFreq;
		double highFreq;
		int order;
	};

	class ButterworthFilter
	{
	public:
		ButterworthFilter();
		~ButterworthFilter();

	private:

	};

}

#endif
