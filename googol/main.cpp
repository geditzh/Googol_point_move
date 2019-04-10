#include <QCoreApplication>
#include "AssemblyRobot.h"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include  <QFile>
#include "gts.h"
using namespace std;

int main(int argc, char *argv[])    //正逆街测试
{
    QCoreApplication e(argc, argv);

    vector<vector<double>> pos;
    QFile file("C:/Users/df/Desktop/Googol_point_move/googol/test.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        cout<<"Can't open the file!"<<endl;
    }

    QStringList list;
    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        QString str(line);
        list = str.split("\t");

        double x = list[0].toDouble();
        double y = list[1].toDouble();
        double z = list[2].toDouble();

        vector<double> temp;
        temp.push_back(x);
        temp.push_back(y);
        temp.push_back(z);
        pos.push_back(temp);
    }

    vector<vector<double>> pos1;
    QFile file1("C:/Users/df/Desktop/Googol_point_move/googol/luoxuanxian.txt");
    if(!file1.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        cout<<"Can't open the file!"<<endl;
    }

    QStringList list1;
    while(!file1.atEnd())
    {
        QByteArray line1 = file1.readLine();
        QString str(line1);
        list1 = str.split("\t");

        double x = list1[0].toDouble();
        double y = list1[1].toDouble();
        double z = list1[2].toDouble();

        vector<double> temp;
        temp.push_back(x);
        temp.push_back(y);
        temp.push_back(z);
        pos1.push_back(temp);
    }

    cout<<pos.size()<<endl;
    cout<<pos1.size()<<endl;

    cout.setf(ios::fixed);
    cout.precision(16);
    short flag=1;
    long lGpiValue;

    AssemblyRobot myrobot;
    cout << "myrobot.Initial():"<<myrobot.Initial(RbtAllAxis)<<endl;

    vector<long> time1;
    for(int i=0;i<pos.size();i++)
    {
        time1.push_back(1*i);
    }

    vector<long> time2;
    for(int i=0;i<pos1.size();i++)
    {
        time2.push_back(8*i);
    }

    while(flag == 1)
    {
        GT_GetDi(MC_GPI,&lGpiValue);
        if(lGpiValue& (1<<1))
        {
            if(myrobot.PointMove(pos,time1,pos.size())!=RobotSuccess)
                cout<<"oh my god"<<endl;

            if(myrobot.PointMove(pos1,time2,pos1.size())!=RobotSuccess)
                cout<<"oh my god"<<endl;

        }
        if(lGpiValue& (1<<powerIO))
        {
            flag = 0;
        }
    }
    cout <<"myrobot.Close():"<< myrobot.Close()<<endl;



//        cout.setf(ios::fixed);
//        cout.precision(16);
//        short flag=1;
//        long lGpiValue;

//        AssemblyRobot myrobot;
//        cout << "myrobot.Initial():"<<myrobot.Initial(RbtAllAxis)<<endl;

//        double vel_min[1][3] = {{0.005,0.04,0.005}};
//        double vel1[1][3] = {{0.2,0.8,0.2}};
//        double vel2[1][3] = {{0.1,0.8,0.1}};
//        double vel3[1][3] = {{0.01,0.15,0.01}};

//        double ang_xMax[2][3] = {{0,0,0},{30,0,0}};
//        double ang_xMin[2][3] = {{0,0,0},{-30,0,0}};
//        double ang_yMax[2][3] = {{0,0,0},{0,30,0}};
//        double ang_yMin[2][3] = {{0,0,0},{0,-30,0}};
//        double ang_zMax[2][3] = {{0,0,0},{0,0,-40}};
//        double ang_zMin[2][3] = {{0,0,0},{0,0,75}};

//        double ang0[2][3] = {{0,0,0},{0,0,0}};
//        double ang1[2][3] = {{0,0,0},{-25,0,0}};
//        double ang2[2][3] = {{0,0,0},{25,0,0}};
//        double ang3[2][3] = {{0,0,0},{0,-25,0}};
//        double ang4[2][3] = {{0,0,0},{0,25,0}};
//        double ang5[2][3] = {{0,0,0},{0,0,-25}};
//        double ang6[2][3] = {{0,0,0},{0,0,25}};

//        double ang7[2][3] = {{0,0,0},{-15,15,0}};
//        double ang8[2][3] = {{0,0,0},{-15,-15,0}};
//        double ang9[2][3] = {{0,0,0},{15,-15,0}};
//        double ang10[2][3] = {{0,0,0},{15,15,0}};

//        double ang11[2][3] = {{0,0,0},{-10,0,0}};
//        double ang12[2][3] = {{0,0,0},{10,0,0}};
//        double ang13[2][3] = {{0,0,0},{0,-10,0}};
//        double ang14[2][3] = {{0,0,0},{0,10,0}};

//        double ang15[2][3] = {{0,0,0},{-10,10,0}};
//        double ang16[2][3] = {{0,0,0},{-10,-10,0}};
//        double ang17[2][3] = {{0,0,0},{10,-10,0}};
//        double ang18[2][3] = {{0,0,0},{10,10,0}};

//        while(flag == 1)
//        {
//            GT_GetDi(MC_GPI,&lGpiValue);
//            if(lGpiValue& (1<<1))
//            {
////                for(int j=0;j<1;j++)
////                {
////                    if(myrobot.AngMove(ang_xMax,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang_xMin,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang0,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang_yMax,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang_yMin,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang0,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang_zMax,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang_zMin,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;

////                    if(myrobot.AngMove(ang0,vel_min,1)!=RobotSuccess)
////                        cout<<"oh my god"<<endl;
////                }
//                for(int o=0;o<1;o++)
//                {
//                    if(myrobot.AngMove(ang5,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang6,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang5,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang6,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang5,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang6,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang5,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang6,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang0,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang3,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang4,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang3,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang4,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang3,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang4,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang3,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang4,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang0,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang1,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang2,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang1,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang2,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang1,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang2,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang1,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang2,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;

//                    if(myrobot.AngMove(ang0,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                }

//                for(int o=0;o<1;o++)
//                {
//                    /*********************************************正星形******************************/
//                     if(myrobot.AngMove(ang14,vel1,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang16,vel2,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang12,vel1,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang15,vel2,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang13,vel1,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang18,vel2,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang11,vel1,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang17,vel2,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                     if(myrobot.AngMove(ang14,vel3,1)!=RobotSuccess)
//                         cout<<"oh my god"<<endl;
//                   /****************************************************************************/

//                   /*********************************************反星形******************************/
//                    if(myrobot.AngMove(ang14,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang17,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang11,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang18,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang13,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang15,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang12,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang16,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang14,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                  /****************************************************************************/
//                  /*********************************************正圆形******************************/
//                    if(myrobot.AngMove(ang4,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang7,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang1,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang8,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang3,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang9,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang2,vel1,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang10,vel2,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang4,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                    if(myrobot.AngMove(ang0,vel3,1)!=RobotSuccess)
//                        cout<<"oh my god"<<endl;
//                  /****************************************************************************/
//                    /*********************************************反圆形******************************/
//                      if(myrobot.AngMove(ang4,vel1,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang10,vel2,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang2,vel1,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang9,vel2,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang3,vel1,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang8,vel2,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang1,vel1,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang7,vel2,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang4,vel3,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                      if(myrobot.AngMove(ang0,vel3,1)!=RobotSuccess)
//                          cout<<"oh my god"<<endl;
//                    /****************************************************************************/
//                }
//            }

//            if(lGpiValue& (1<<powerIO))
//            {
//                flag = 0;
//            }
//        }
//        cout <<"myrobot.Close():"<< myrobot.Close()<<endl;

    return e.exec();
}

//int main(int argc,char *argv[])	//电机点位运动
//{
//    QCoreApplication a(argc, argv);
//    AssemblyRobot myrobot;
//    short count,flag=1;
//    long lGpiValue;

//    cout << "myrobot.Initial():"<<myrobot.Initial(RbtAllAxis)<<endl;

//    while(flag == 1)
//    {
//        GT_GetDi(MC_GPI,&lGpiValue);
//        if(lGpiValue& (1<<1))
//        {
//            double ang1[1][3];
//            long time[1] = { 5000 };
//            while(lGpiValue& (1<<1))
//            {
//                cout<<"input three numbers"<<endl;
//                scanf("%lf%lf%lf",&ang1[0][0],&ang1[0][1],&ang1[0][2]);
//                if(ang1[0][0] == 100) break;
//                count = sizeof(time) / sizeof(time[0]);
//                cout << "count = " << count << endl;
//                cout << "AngMove(): = " << myrobot.PointMove(ang1, time, count) << endl;
//            }
//        }

//        if(lGpiValue& (1<<powerIO))
//        {
//            flag = 0;
//        }
//    }

//    cout <<"myrobot.Close():"<< myrobot.Close()<<endl;

//    return a.exec();
//}

