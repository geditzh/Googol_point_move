#include <QCoreApplication>
#include "AssemblyRobot.h"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include  <QFile>
#include "gts.h"
using namespace std;

int main(int argc,char *argv[])	//电机点位运动
{
    QCoreApplication a(argc, argv);
    AssemblyRobot myrobot;
    short count,flag=1;
    long lGpiValue;

    cout << "myrobot.Initial():"<<myrobot.Initial(RbtAllAxis)<<endl;

    while(flag == 1)
    {
        GT_GetDi(MC_GPI,&lGpiValue);
        if(lGpiValue& (1<<1))
        {
            double ang1[1][3];
            long time[1] = { 3000 };
            while(lGpiValue& (1<<1))
            {
                cout<<"input three numbers"<<endl;
                scanf("%lf%lf%lf",&ang1[0][0],&ang1[0][1],&ang1[0][2]);
                if(ang1[0][0] >= 100) break;
                count = sizeof(time) / sizeof(time[0]);
                cout << "AngMove(): = " << myrobot.PointMove(ang1, time, count) << endl;
            }
        }

        if(lGpiValue& (1<<powerIO))
        {
            flag = 0;
        }
    }

    cout <<"myrobot.Close():"<< myrobot.Close()<<endl;

    return a.exec();
}

