
#include "stdafx.h"
#include "recordpath.h"

#include <fstream>

using namespace std;

namespace config {

	static char platfilename[100] = "";

	FileRecord::FileRecord()
	{
		time_t currtime = time(NULL);
		struct tm* p = gmtime(&currtime);
		sprintf_s(platfilename, "./datas/platdata%d-%d-%d-%d-%d-%d.txt", p->tm_year + 1990, p->tm_mon + 1,
			p->tm_mday, p->tm_hour + 8, p->tm_min, p->tm_sec);
	}

	void FileRecord::Record(double x, double y, double z, double roll, double yaw, double pitch, 
		double * poleLength, double * pulses)
	{
		ofstream fout(platfilename, ios::app);
		fout << x << " ";
		fout << y << " ";
		fout << z << " ";
		fout << roll << " ";
		fout << yaw << " ";
		fout << pitch << " ";
		for (int i = 0;i < 6;++i)
		{
			fout << poleLength[i] << " ";
		}
		for (int i = 0;i < 6;++i)
		{
			fout << pulses[i] << " ";
		}
		fout << endl;
		fout.flush();
		fout.close();	
	}

	void RecordPath(const char * filename, double x, double y, double z, double roll, double yaw, double pitch)
	{
		ofstream fout(filename, ios::app);
		fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";
		fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";
		fout << x << " ";
		fout << y << " ";
		fout << z << " ";
		fout << roll << " ";
		fout << yaw << " ";
		fout << pitch << " ";
		fout << endl;
		fout.flush();
		fout.close();
	}

	void RecordData(const char * filename, double XAcc,double YAcc,double ZAcc,double RollSpeed,
		double PitchSpeed,double YawSpeed, double Roll, double Pitch,double Yaw, int type)
	{
		ofstream fout(filename, ios::app);
		fout << XAcc << " ";
		fout << YAcc << " ";
		fout << ZAcc << " ";
		fout << RollSpeed << " ";
		fout << PitchSpeed << " ";
		fout << YawSpeed << " ";
		fout << Roll << " ";
		fout << Pitch << " ";
		fout << Yaw << " ";
		fout << type << " ";
		fout << endl;
		fout.flush();
		fout.close();
	}

}
