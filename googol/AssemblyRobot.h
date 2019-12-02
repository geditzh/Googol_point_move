#pragma once
#include "RobotDefine.h"
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

class AssemblyRobot
{
private:
	short lastRtn;
	short lastRtnErr;
	short robotCoor;
    double lastEncPos[3], lastPrfPos[3], lastAng[3], lastPos[3];
	const double initAng[3] = {0,0,0};
    long flag_time=0;

protected:
	short ForwardKinematics(const double(&pos)[3], double(&ang)[3]);
public:
	short InverseKinematics(const double(&ang)[3], double(&pos)[3]);

	AssemblyRobot();
	~AssemblyRobot();
    short Initial(short coor);
	short Home();
	short Stop(short stopMode);
	short WaitForMotionComplete(short moveMode);
	short Close();
	
	short GetEncStatus(double(&Prf)[3],double(&Enc)[3]);
    short PointMove(double(*pAng)[3], long *pTime, unsigned int count);
//    short PointMove(vector<vector<double>> pAng, vector<long> pTime, unsigned int count);
    short AngMove(double(*pAng)[3], double (*profile)[3], unsigned int count);
    short CircleMove(double(*pAng)[3], double (*profile)[3], unsigned int count);
    short SetDo(short index, short value);
    short SetExtDo(short index, short value);
};
