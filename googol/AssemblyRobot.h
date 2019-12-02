#pragma once
#include "RobotDefine.h"
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

class AssemblyRobot
{
private:
    short lastRtn;          //用于读取每一次的函数调用标志位，0为正常，非0值含义定义在RobotDefine.h中的Return Value
    short lastRtnErr;       //当调用函数出错时，将该值置未非0并返回
    short robotCoor;    //设置机器人坐标系，目前没有区分，后期加了3T1R机构，可用于区分，分别初始化两个机器人。 对应值在RobotDefine.h中的Axis Mode
    double lastEncPos[3], lastPrfPos[3], lastAng[3], lastPos[3];    //接收三个轴的规划器直线电机位置以及编码器所读出的直线电机位置
    const double initAng[3] = {0,0,0};      //相对HOME点开始的起始Z-Y-X欧拉角
    long flag_time=0;

protected:
    short ForwardKinematics(const double(&pos)[3], double(&ang)[3]);    //3-PSS/S机器人正运动学
public:
    short InverseKinematics(const double(&ang)[3], double(&pos)[3]);    //3-PSS/S机器人逆运动学

    AssemblyRobot();
	~AssemblyRobot();
    short Initial(short coor);   //初始化，加载配置文件，设置误差带
    short Home();   //从负限位开始运动至标准HOME点
    short Stop(short stopMode);     //关闭各轴使能
    short WaitForMotionComplete(short moveMode);    //等待所有轴运动完毕（在PT等模式下，写入运动轨迹，等待所有轴都运动完毕后，该函数执行返回）
    short Close();      //关闭固高控制卡
	
    short GetEncStatus(double(&Prf)[3],double(&Enc)[3]);    //查看规划器的数值与编码器的数值
    short PointMove(double(*pAng)[3], long *pTime, unsigned int count);     //输入Z-Y-X欧拉角位置序列，以及时间序列，依次运动到指定位置
//    short PointMove(vector<vector<double>> pAng, vector<long> pTime, unsigned int count);     //外部读入离线规划好的txt文件时用到，与上个函数功能相同
    short AngMove(double(*pAng)[3], double (*profile)[3], unsigned int count);      //输入任务位姿的Z-Y-X欧拉角，以及运动速度，内部包含梯型速度规划，生成欧拉角位置时间序列，调用PointMove函数
    short SetDo(short index, short value);      //控制固高板卡自带的IO
    short SetExtDo(short index, short value);       //控制扩展的外部IO模块
};
