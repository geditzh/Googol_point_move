#pragma once

const int axisNumber[3] = { 1,2,3 };        //3-PSS/S机器人连接在固高控制卡的轴1，2，3上，后期3T1R可继续往后拓展
const int powerIO = 8;      //该值对应控制柜的红色按钮（非急停），用于软件程序结束控制，红色按钮按下，该值置1
const int cHome = 0;        //该值对应直线电机的Home点位置，已经用激光跟踪仪标定完成，所以该值相对于Home点的位置为0

#define PI         3.14159265358979323846

//定义3-PSS/S的各轴最大工作空间（Z-Y-X欧拉角单位度），以及最大速度加速约束
const double angRange[3][2] =
                        { {-37 , 35},
                          {-37 , 35},
                          {-45 , 80} };
const double MaxAcc = 0.10;
const double MaxDcc = 0.10;
const double MaxVel = 0.20;

//定义直线电机的实际位移与编码器脉冲数量的转换比例，3-PSS/S直线电机1个脉冲分辨率为0.5um，实际位移*.mm乘以2000可转换为对应脉冲数
#define cPosToImp					 2000

//Return Value
#define RobotFailure				   -1
#define RobotSuccess					0
#define RobotBusy						1
#define RobotOutOfRange					2

//Axis mode
#define RbtAllAxis                      0
#define Rbt3PSSAxis                     1
#define Rbt3T1RAxis                     2
