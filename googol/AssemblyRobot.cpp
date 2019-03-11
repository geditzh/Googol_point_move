#include "AssemblyRobot.h"
#include "RobotDefine.h"
#include <Windows.h>
#include <math.h>
#include <tchar.h>
#include <stdio.h>
#include "gts.h"
#include "ExtMdl.h"

AssemblyRobot::AssemblyRobot()
{

}
AssemblyRobot::~AssemblyRobot()
{

}
short AssemblyRobot::Initial(short coor)
{
	robotCoor = coor;

	lastRtn = GT_Open();
	if (lastRtn)
	{
		printf("GT_Open():%d\n", lastRtn);
		return lastRtnErr = RobotFailure;		
	}

	lastRtn = GT_Reset();
	if (lastRtn)
	{
		printf("GT_Reset():%d\n", lastRtn);
		return lastRtnErr = RobotFailure;
	}

    lastRtn = GT_LoadConfig("C:/Users/df/Desktop/Googol/googol/GTS800.cfg");
	if (lastRtn)
	{
		printf("GT_LoadConfig():%d\n", lastRtn);
		return lastRtnErr = RobotFailure;
	}

	lastRtn = GT_ClrSts(1, 8);
	if (lastRtn)
	{
		printf("GT_ClrSts():%d\n", lastRtn);
		return lastRtnErr = RobotFailure;
	}

	/************************************************************************************/
	for (short i = 0; i < 3; i++)
	{
		lastRtn = GT_ZeroPos(axisNumber[i]);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

        lastRtn = GT_SetAxisBand(axisNumber[i], 20,5);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		lastRtn = GT_AxisOn(axisNumber[i]);
		if (lastRtn)
		{
			printf("GT_AxisOn():%d\n", lastRtn);
			return lastRtnErr = RobotFailure;
		}
	}

	if (Home() != RobotSuccess)
		return lastRtnErr = RobotFailure;

	return RobotSuccess;
}

short AssemblyRobot::Home()
{
	if (Stop(RbtAllAxis) != RobotSuccess)
		return lastRtnErr = RobotFailure;

	TJogPrm jog;
	int i;
	for (i = 0; i < 3; i++)
	{
		lastRtn = GT_PrfJog(axisNumber[i]);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		lastRtn = GT_GetJogPrm(axisNumber[i],&jog);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

        jog.acc = 0.0625;
        jog.dec = 0.0625;

		lastRtn = GT_SetJogPrm(axisNumber[i], &jog);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

        lastRtn = GT_SetVel(axisNumber[i], -25);
		if (lastRtn)
			return lastRtnErr = RobotFailure;
	}
	Sleep(200);

		lastRtn = GT_Update(
			(1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1)));
		if (lastRtn)
			return lastRtnErr = RobotFailure;
		
		long sts[3] = {0,0,0};
		do
		{
			lastRtn = GT_GetSts(axisNumber[0], sts, 3);
			if (lastRtn)
				return lastRtnErr = RobotFailure;
		} while (!((sts[0] & 0x40) && (sts[1] & 0x40) && (sts[2] & 0x40)));
		
		if (Stop(RbtAllAxis) != RobotSuccess)
			return lastRtnErr = RobotFailure;
		Sleep(500);

		lastRtn = GT_ClrSts(1, 8);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		for (i = 0; i < 3; i++)
		{
            lastRtn = GT_SetVel(axisNumber[i], 25);
			if (lastRtn)
				return lastRtnErr = RobotFailure;
		}
		Sleep(100);

		lastRtn = GT_Update(
			(1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1)));
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		for (i = 0; i < 3; i++)
		{
			lastRtn = GT_SetCaptureMode(axisNumber[i], CAPTURE_HOME);
			if (lastRtn)
				return lastRtnErr = RobotFailure;
		}

		short status[3];
		long pos[3];
		double encPos[3];
		do
		{
			lastRtn = GT_GetCaptureStatus(axisNumber[0], status, pos, 3);
			if (lastRtn)
				return lastRtnErr = RobotFailure;
			lastRtn = GT_GetEncPos(axisNumber[0],encPos,3);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

			//printf("status[0] = %d enc[0] = %-8.0lf\n", status[0], encPos[0]);
			//printf("status[1] = %d enc[1] = %-8.0lf\n", status[1], encPos[1]);
			//printf("status[2] = %d enc[2] = %-8.0lf\n", status[2], encPos[2]);
		} while ((status[0] == 0) || (status[1] == 0) || (status[2] == 0));

		for (i = 0; i < 3; i++)
		{
			printf("\n capture:%-8.0ld\n", pos[i]);
		}

		if (Stop(RbtAllAxis)!=RobotSuccess)
			return lastRtnErr = RobotFailure;
		Sleep(500);

		lastRtn = GT_ClrSts(1, 8);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		TrapPrm trap;
		for (i = 0; i < 3; i++)
		{
			lastRtn = GT_PrfTrap(axisNumber[i]);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

			lastRtn = GT_GetTrapPrm(axisNumber[i], &trap);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

            trap.acc = 0.0625;
            trap.dec = 0.0625;

			lastRtn = GT_SetTrapPrm(axisNumber[i], &trap);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

            lastRtn = GT_SetVel(axisNumber[i], 25);
			if (lastRtn)
				return lastRtnErr = RobotFailure;
		}
		Sleep(100);

        lastRtn = GT_SetPos(axisNumber[0], pos[0] + cHome - 3428);      //+498
        if (lastRtn)
            return lastRtnErr = RobotFailure;

        lastRtn = GT_SetPos(axisNumber[1], pos[1] + cHome - 355);        //+378
        if (lastRtn)
            return lastRtnErr = RobotFailure;

        lastRtn = GT_SetPos(axisNumber[2], pos[2] + cHome - 1794);      //-684
		if (lastRtn)
			return lastRtnErr = RobotFailure;
        Sleep(100);

		lastRtn = GT_Update(
			(1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1)));
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		if (WaitForMotionComplete(RbtAllAxis) != RobotSuccess)
			return RobotFailure;

		lastRtn = GT_ZeroPos(axisNumber[0],3);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		for (i = 0; i < 3; i++)
		{
			lastRtn = GT_SetPrfPos(axisNumber[i], cHome);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

			lastRtn = GT_SetEncPos(axisNumber[i], cHome);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

            lastAng[i] = initAng[i];
		}
        Sleep(1000);
				 
		if (GetEncStatus(lastPrfPos, lastEncPos) != RobotSuccess)
			return lastRtnErr = RobotFailure;

		for (int i = 0; i < 3; i++)
		{
			printf("\nencPos[%d]=%-8.0lf\t", i, lastEncPos[i]);
			printf("prfPos[%d]=%-8.0lf\n", i, lastPrfPos[i]);
		}

		for (i = 0; i < 3; i++)
		{
			lastRtn = GT_PrfPt(axisNumber[i], PT_MODE_DYNAMIC);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

			lastRtn = GT_SetPtMemory(axisNumber[i], 1);
			if (lastRtn)
				return lastRtnErr = RobotFailure;

			lastRtn = GT_PtClear(axisNumber[i]);
			if (lastRtn)
				return lastRtnErr = RobotFailure;
		}
		return RobotSuccess;
}

short AssemblyRobot::Stop(short stopMode)
{
	if (stopMode == RbtAllAxis)
	{
		lastRtn = GT_Stop(
			((1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1))),
			((1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1))));
	}
	return RobotSuccess;
}

short AssemblyRobot::WaitForMotionComplete(short moveMode)
{
	long sts[3];
	double encPos[3], prfPos[3];
	if (moveMode == RbtAllAxis)
	{
		do
		{
			lastRtn = GT_GetSts(axisNumber[0], sts, 3);
			if (lastRtn)
				return lastRtnErr = RobotFailure;
		} while (!((sts[0] & 0x800) && (sts[1] & 0x800) && (sts[2] & 0x800)));
		
		lastRtn = GT_GetEncPos(axisNumber[0], encPos, 3);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		lastRtn = GT_GetPrfPos(axisNumber[0],prfPos,3);
		if (lastRtn)
			return lastRtnErr = RobotFailure;

		for (int i = 0; i < 3;i++)
		{ 
			printf("\nencPos[%d]=%-8.0lf\t",i,encPos[i]);
			printf("prfPos[%d]=%-8.0lf\n",i,prfPos[i]);
		}
		return RobotSuccess;
	}
}


short AssemblyRobot::InverseKinematics(const double(&ang)[3], double(&pos)[3])
{
	double alpha, beta, gamma;
	double theta = 30.0 / 180 * PI;
	double r = 91.0;
	double L = 354.0;
	double h = 50.0;
	double H = 50.0;

	alpha = ang[0]/180.0*PI; 
	beta = ang[1]/180.0*PI; 
	gamma = ang[2]/180.0*PI;

	
	double R11 = cos(gamma) * cos(beta);
	double R12 = cos(gamma) * sin(beta) * sin(alpha) - sin(gamma) * cos(alpha);
	double R13 = cos(gamma) * sin(beta) * cos(alpha) + sin(gamma) * sin(alpha);
	double R21 = sin(gamma) * cos(beta);
	double R22 = sin(gamma) * sin(beta) * sin(alpha) + cos(gamma) * cos(alpha);
	double R23 = sin(gamma) * sin(beta) * cos(alpha) - cos(gamma) * sin(alpha);
	double R31 = -sin(beta);
	double R32 = cos(beta) * sin(alpha);
	double R33 = cos(beta) * cos(alpha);

	double a1[] = {sin(theta)*cos(-2 * PI / 3), -sin(theta)*sin(-2 * PI / 3), -cos(theta) };
	double a2[] = {sin(theta)*cos(2 * PI / 3), -sin(theta)*sin(2 * PI / 3), -cos(theta) };
	double a3[] = {sin(theta)*cos(0), -sin(theta)*sin(0), -cos(theta) };
	
//    double g1[] = { r * R11 * cos(-PI) - r * R12 *sin(-PI), r * R21 * cos(-PI) - r * R22 *sin(-PI), r * R31 * cos(-PI) - r * R32 *sin(-PI) + h };
//    double g2[] = { r * R11 * cos(PI / 3) - r * R12 *sin(PI / 3), r * R21 * cos(PI / 3) - r * R22 *sin(PI / 3), r * R31 * cos(PI / 3) - r * R32 *sin(PI / 3) + h };
//    double g3[] = { r * R11 * cos(-PI / 3) - r * R12 *sin(-PI / 3), r * R21 * cos(-PI / 3) - r * R22 *sin(-PI / 3), r * R31 * cos(-PI / 3) - r * R32 *sin(-PI / 3) + h };
    double g1[] = {r*(R11*cos(-PI)-R12*sin(-PI)+R13*h/r), r*(R21*cos(-PI)-R22*sin(-PI)+R23*h/r), r*(R31*cos(-PI)-R32*sin(-PI)+R33*h/r)};
    double g2[] = {r*(R11*cos(PI/3)-R12*sin(PI/3)+R13*h/r), r*(R21*cos(PI/3)-R22*sin(PI/3)+R23*h/r), r*(R31*cos(PI/3)-R32*sin(PI/3)+R33*h/r)};
    double g3[] = {r*(R11*cos(-PI/3)-R12*sin(-PI/3)+R13*h/r), r*(R21*cos(-PI/3)-R22*sin(-PI/3)+R23*h/r), r*(R31*cos(-PI/3)-R32*sin(-PI/3)+R33*h/r)};

	double U1 = pow(a1[0], 2) + pow(a1[1], 2) + pow(a1[2], 2);
	double U2 = pow(a2[0], 2) + pow(a2[1], 2) + pow(a2[2], 2);
	double U3 = pow(a3[0], 2) + pow(a3[1], 2) + pow(a3[2], 2);

	double V1 = -2 * (a1[0] * g1[0] + a1[1] * g1[1] + a1[2] * g1[2]);
	double V2 = -2 * (a2[0] * g2[0] + a2[1] * g2[1] + a2[2] * g2[2]);
	double V3 = -2 * (a3[0] * g3[0] + a3[1] * g3[1] + a3[2] * g3[2]);

	double W1 = pow(g1[0], 2) + pow(g1[1], 2) + pow(g1[2], 2) - pow(L, 2);
	double W2 = pow(g2[0], 2) + pow(g2[1], 2) + pow(g2[2], 2) - pow(L, 2);
	double W3 = pow(g3[0], 2) + pow(g3[1], 2) + pow(g3[2], 2) - pow(L, 2);

	pos[0] = (-(V1 - 2 * a1[2] * H) + sqrt(pow((V1 - 2 * a1[2] * H), 2) - 4 * U1 * (W1 + pow(H, 2) + 2 * H * g1[2]))) / (2 * U1);
	//d1_2 = (-(V1 - 2 * a1[2] * H) - sqrt(pow((V1 - 2 * a1[2] * H),2) - 4 * U1 * (W1 + pow(H,2) + 2 * H * g1[2]))) / (2 * U1);

	pos[1] = (-(V2 - 2 * a2[2] * H) + sqrt(pow((V2 - 2 * a2[2] * H), 2) - 4 * U2 * (W2 + pow(H, 2) + 2 * H * g2[2]))) / (2 * U2);
	//d2_2 = (-(V2 - 2 * a2[2] * H) - sqrt(pow((V2 - 2 * a2[2] * H), 2) - 4 * U2 * (W2 + pow(H, 2) + 2 * H * g2[2]))) / (2 * U2);

	pos[2] = (-(V3 - 2 * a3[2] * H) + sqrt(pow((V3 - 2 * a3[2] * H), 2) - 4 * U3 * (W3 + pow(H, 2) + 2 * H * g3[2]))) / (2 * U3);
	//d3_2 = (-(V3 - 2 * a3[2] * H) - sqrt(pow((V3 - 2 * a3[2] * H), 2) - 4 * U3 * (W3 + pow(H, 2) + 2 * H * g3[2]))) / (2 * U3);

	return RobotSuccess;
}

short AssemblyRobot::Close()
{
	for (int i = 0; i < 3; i++)
	{
		lastRtn = GT_AxisOff(axisNumber[i]);
		if (lastRtn)
			return lastRtnErr = RobotFailure;
	}
	Sleep(1000);

	lastRtn = GT_Close();
	if (lastRtn)
		return lastRtnErr = RobotFailure;

	return RobotSuccess;
}

short AssemblyRobot::GetEncStatus(double(&Prf)[3],double(&Enc)[3])
{
	lastRtn = GT_GetPrfPos(axisNumber[0], Prf, 3);
	if (lastRtn)
		return lastRtnErr = RobotFailure;

	lastRtn = GT_GetEncPos(axisNumber[0], Enc, 3);
	if (lastRtn)
		return lastRtnErr = RobotFailure;
	return RobotSuccess;
}


short AssemblyRobot::PointMove(vector<vector<double>> pAng, vector<long> pTime, unsigned int count)
{
    long i, j, start = 0;
    short space[3];
    double ang[3],pos[3];

    InverseKinematics(lastAng, lastPos);

    lastRtn = GT_ClrSts(1, 8);
    if (lastRtn)
        return lastRtnErr = RobotFailure;

    for (i = 0; i < 3; i++)
    {
        lastRtn = GT_PrfPt(axisNumber[i], PT_MODE_DYNAMIC);
        if (lastRtn)
            return lastRtnErr = RobotFailure;

        lastRtn = GT_SetPtMemory(axisNumber[i], 1);
        if (lastRtn)
            return lastRtnErr = RobotFailure;

        lastRtn = GT_PtClear(axisNumber[i]);
        if (lastRtn)
            return lastRtnErr = RobotFailure;
    }

    for (j = 0; j < count; j++)
    {
        for (i = 0; i < 3; i++)
        {
            ang[i] = pAng[j][i];
            if(j==count-1)
                lastAng[i] = pAng[j][i];
        }

        for (i = 0; i < 3; i++)
        {
            if ((ang[i] < angRange[i][0]) || (ang[i] > angRange[i][1]))
            {
                cout << "Servo out of range" << endl;
                return lastRtnErr = RobotOutOfRange;
            }
        }

        InverseKinematics(ang, pos);

        pos[0] = (pos[0] - lastPos[0])*cPosToImp;
        pos[1] = (pos[1] - lastPos[1])*cPosToImp;
        pos[2] = (pos[2] - lastPos[2])*cPosToImp;

        cout << j << ":" << pos[0] << "," << pos[1] << "," << pos[2] << endl;
        for (i = 0; i < 3; i++)
        {
            if (j == 0)
            {
                lastRtn = GT_PtData(axisNumber[i], pos[i], pTime[j]+1, PT_SEGMENT_NORMAL);
                if (lastRtn)
                    return lastRtnErr = RobotFailure;
            }

            else if (j == (count - 1))
            {
                lastRtn = GT_PtData(axisNumber[i], pos[i], pTime[j]+1, PT_SEGMENT_STOP);
                if (lastRtn){
                    return lastRtnErr = RobotFailure;}
            }

            else
            {
                lastRtn = GT_PtData(axisNumber[i], pos[i], pTime[j]+1, PT_SEGMENT_EVEN);
                if (lastRtn){
                    return lastRtnErr = RobotFailure;}
            }

            lastRtn = GT_PtSpace(axisNumber[i], &space[i]);
            if (lastRtn)
                return lastRtnErr = RobotFailure;
        }

        if (!(space[0] && space[1] && space[2]))
        {
            if (start == 0)
            {
                lastRtn = GT_PtStart((1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1)));
                if (lastRtn)
                    return lastRtnErr = RobotFailure;

                start = 1;
            }

            while (1)
            {
                for (i = 0; i < 3; i++)
                {
                    lastRtn = GT_PtSpace(axisNumber[i], &space[i]);
                    if (lastRtn)
                        return lastRtnErr = RobotFailure;
                }
                if ((space[0] && space[1] && space[2]))
                    break;
            }
        }
    }

    if (start == 0)
    {
        lastRtn = GT_PtStart((1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1)));
        if (lastRtn)
            return lastRtnErr = RobotFailure;
        start = 1;
    }

    if (WaitForMotionComplete(RbtAllAxis) != RobotSuccess)
        return lastRtnErr = RobotFailure;

    if (GetEncStatus(lastPrfPos, lastEncPos) != RobotSuccess)
        return lastRtnErr = RobotFailure;

    return RobotSuccess;
}


short AssemblyRobot::PointMove(double(*pAng)[3], long *pTime, unsigned int count)
{
    short i, j, start = 0;
    short space[3];
    double ang[3],pos[3];

    InverseKinematics(lastAng, lastPos);

    FILE *fp = fopen("C:/Users/df/Desktop/Googol_point_move/googol/test.txt","a");
    FILE *fp1 = fopen("C:/Users/df/Desktop/Googol_point_move/googol/time.txt","a");

    cout<<count<<endl;
    for(i=0;i<count;i++)
    {
        cout<<"Time:"<<pTime[i]<<"\t";
        fprintf(fp1,"%d\n",pTime[i]+flag_time);
        if(i != 0)
        {
//            cout<<"Point:"<<pAng[i][0]<<","<<pAng[i][1]<<","<<pAng[i][2]<<endl;
//            cout<<"Error:"<<pAng[i][0]-pAng[i-1][0]<<","
//                <<pAng[i][1]-pAng[i-1][1]<<","
//                <<pAng[i][2]-pAng[i-1][2]<<endl;
            cout<<"Point:"<<pAng[i][0]<<","<<pAng[i][1]<<","<<pAng[i][2]<<endl;
            fprintf(fp,"%f\t%f\t%f\n",pAng[i][0],pAng[i][1],pAng[i][2]);

        }
        else
        {
            cout<<"Point:"<<pAng[i][0]<<","<<pAng[i][1]<<","<<pAng[i][2]<<endl;
        }
    }

    fclose(fp);
    fclose(fp1);

    flag_time += pTime[count-1];

    lastRtn = GT_ClrSts(1, 8);
    if (lastRtn)
        return lastRtnErr = RobotFailure;

    for (i = 0; i < 3; i++)
    {
        lastRtn = GT_PrfPt(axisNumber[i], PT_MODE_DYNAMIC);
        if (lastRtn)
            return lastRtnErr = RobotFailure;

        lastRtn = GT_SetPtMemory(axisNumber[i], 1);
        if (lastRtn)
            return lastRtnErr = RobotFailure;

        lastRtn = GT_PtClear(axisNumber[i]);
        if (lastRtn)
            return lastRtnErr = RobotFailure;
    }

    for (j = 0; j < count; j++)
    {
        for (i = 0; i < 3; i++)
        {
            ang[i] = pAng[j][i];
            if(j==count-1)
                lastAng[i] = pAng[j][i];
        }

        for (i = 0; i < 3; i++)
        {
            if ((ang[i] < angRange[i][0]) || (ang[i] > angRange[i][1]))
            {
                cout << "Servo out of range" << endl;
                return lastRtnErr = RobotOutOfRange;
            }
        }

        InverseKinematics(ang, pos);

        pos[0] = (pos[0] - lastPos[0])*cPosToImp;
        pos[1] = (pos[1] - lastPos[1])*cPosToImp;
        pos[2] = (pos[2] - lastPos[2])*cPosToImp;

        cout << j << ":" << pos[0] << "," << pos[1] << "," << pos[2] << endl;
        for (i = 0; i < 3; i++)
        {
            if (j == 0)
            {
                lastRtn = GT_PtData(axisNumber[i], pos[i], pTime[j]+1, PT_SEGMENT_NORMAL);
                if (lastRtn)
                    return lastRtnErr = RobotFailure;
            }

            else if (j == (count - 1))
            {
                lastRtn = GT_PtData(axisNumber[i], pos[i], pTime[j]+1, PT_SEGMENT_STOP);
                if (lastRtn){
                    return lastRtnErr = RobotFailure;}
            }

            else
            {
                lastRtn = GT_PtData(axisNumber[i], pos[i], pTime[j]+1, PT_SEGMENT_EVEN);
                if (lastRtn){
                    return lastRtnErr = RobotFailure;}
            }

            lastRtn = GT_PtSpace(axisNumber[i], &space[i]);
            if (lastRtn)
                return lastRtnErr = RobotFailure;
        }

        if (!(space[0] && space[1] && space[2]))
        {
            if (start == 0)
            {
                lastRtn = GT_PtStart((1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1)));
                if (lastRtn)
                    return lastRtnErr = RobotFailure;

                start = 1;
            }

            while (1)
            {
                for (i = 0; i < 3; i++)
                {
                    lastRtn = GT_PtSpace(axisNumber[i], &space[i]);
                    if (lastRtn)
                        return lastRtnErr = RobotFailure;
                }
                if ((space[0] && space[1] && space[2]))
                    break;
            }
        }
    }

    if (start == 0)
    {
        lastRtn = GT_PtStart((1 << (axisNumber[0] - 1)) + (1 << (axisNumber[1] - 1)) + (1 << (axisNumber[2] - 1)));
        if (lastRtn)
            return lastRtnErr = RobotFailure;
        start = 1;
    }

    if (WaitForMotionComplete(RbtAllAxis) != RobotSuccess)
        return lastRtnErr = RobotFailure;

    if (GetEncStatus(lastPrfPos, lastEncPos) != RobotSuccess)
        return lastRtnErr = RobotFailure;

    return RobotSuccess;
}

short AssemblyRobot::AngMove(double(*pAng)[3], double (*profile)[3], unsigned int count)
{
    pAng[0][0] = lastAng[0];
    pAng[0][1] = lastAng[1];
    pAng[0][2] = lastAng[2];

    short i,j;
    double length;
    unsigned int (*itpCount)[3] = new unsigned int[count][3];
    unsigned int itpSum = 0;
    double *x = new double[count];
    double *y = new double[count];
    double *z = new double[count];

    double allowVel,allowAcc,allowDcc;

    for(i=0; i<count; i++)
    {
        x[i] = pAng[i+1][0]-pAng[i][0];
        y[i] = pAng[i+1][1]-pAng[i][1];
        z[i] = pAng[i+1][2]-pAng[i][2];

        length = sqrt(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]);

        allowAcc = profile[i][0]>MaxAcc?MaxAcc:profile[i][0];
        allowVel = profile[i][1]>MaxVel?MaxVel:profile[i][1];
        allowDcc = profile[i][2]>MaxDcc?MaxDcc:profile[i][2];

        if(allowVel*allowVel < 2*length*allowAcc*allowDcc/(allowAcc+allowDcc))
        {
            itpCount[i][0] = int(ceil(allowVel/allowAcc));
            itpCount[i][1] = int(ceil(length/allowVel-
                                      (allowAcc+allowDcc)*allowVel/(2*allowAcc*allowDcc)));
            itpCount[i][2] = int(ceil(allowVel/allowDcc));

        }
        else
        {
            itpCount[i][0] = int(ceil(sqrt(2*allowDcc/(allowAcc*allowDcc+allowAcc*allowAcc)*length)));
            itpCount[i][1] = 0;
            itpCount[i][2] = int(ceil(sqrt(2*allowAcc/(allowAcc*allowDcc+allowDcc*allowDcc)*length)));
        }
        length = (itpCount[i][0]+itpCount[i][2])/2.0+itpCount[i][1];

        x[i] = x[i]/length;
        y[i] = y[i]/length;
        z[i] = z[i]/length;

        itpSum += itpCount[i][0]+itpCount[i][1]+itpCount[i][2];
    }

        double(*pAngs)[3] = new double[itpSum+1][3];
        long *pTime = new long[itpSum+1];
        unsigned int temp=0;
        double percent;

        for(i=0; i<count; i++)
        {
            for(j=0; j<itpCount[i][0]; j++)
            {
                if(j==0) percent=0;
                else
                {
                    percent = (2*j-1)/2.0/itpCount[i][0]+percent;
                }
                pAngs[temp+j][0] = pAng[i][0]+percent*x[i];
                pAngs[temp+j][1] = pAng[i][1]+percent*y[i];
                pAngs[temp+j][2] = pAng[i][2]+percent*z[i];
                pTime[temp+j] = temp+j;
            }
            temp += itpCount[i][0];

            for(j=0; j<itpCount[i][1]; j++)
            {
                if(j==0)
                {
                    percent = (2*itpCount[i][0]-1)/2.0/itpCount[i][0] + percent;
                }
                else
                {
                    percent = 1+percent;
                }
                pAngs[temp+j][0] = pAng[i][0]+percent*x[i];
                pAngs[temp+j][1] = pAng[i][1]+percent*y[i];
                pAngs[temp+j][2] = pAng[i][2]+percent*z[i];
                pTime[temp+j] = temp+j;
            }
            temp +=itpCount[i][1];

            for(j=0; j<itpCount[i][2]; j++)
            {
                if(j==0)
                {
                    if(itpCount[i][1]==0)
                    {
                        percent = (2*itpCount[i][0]-1)/2.0/itpCount[i][0] + percent;
                    }
                    else
                    {
                        percent = percent+1;
                    }
                }
                else
                {
                    percent = (2*(itpCount[i][2]-j+1)-1)/2.0/itpCount[i][2] + percent;
                }
                pAngs[temp+j][0] = pAng[i][0]+percent*x[i];
                pAngs[temp+j][1] = pAng[i][1]+percent*y[i];
                pAngs[temp+j][2] = pAng[i][2]+percent*z[i];
                pTime[temp+j] = temp+j;
            }
            temp += itpCount[i][2];
        }

        pAngs[itpSum][0] = pAng[i][0];
        pAngs[itpSum][1] = pAng[i][1];
        pAngs[itpSum][2] = pAng[i][2];
        pTime[itpSum] = itpSum;

        PointMove(pAngs,pTime,itpSum+1);

        delete[] pAngs;delete[] pTime;
        delete[] z;delete[] y;delete[] x;
        delete[] itpCount;

        return RobotSuccess;
}


short AssemblyRobot::SetDo(short index, short value)
{
    lastRtn = GT_SetDoBit(MC_GPO,index,value);
    if(lastRtn)
        return lastRtnErr = RobotFailure;
    return RobotSuccess;
}

short AssemblyRobot::SetExtDo(short index, short value)
{
    lastRtn = GT_OpenExtMdl("gts.dll");
    if(lastRtn)
        return lastRtnErr = RobotFailure;

    lastRtn = GT_LoadExtConfig("C:/Users/df/Desktop/Googol/googol/ExtModule.cfg");
    if(lastRtn)
        return lastRtnErr = RobotFailure;

    lastRtn = GT_SetExtIoBit(0,index,value);
    lastRtn = GT_SetExtIoBit(0,index+1,value);
    if(lastRtn)
        return lastRtnErr = RobotFailure;

    lastRtn = GT_CloseExtMdl();
    if(lastRtn)
        return lastRtnErr = RobotFailure;

    return RobotSuccess;
}

