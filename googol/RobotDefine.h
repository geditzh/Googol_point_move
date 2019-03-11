#pragma once

const int axisNumber[3] = { 1,2,3 };
const int powerIO = 8;
const int cHome = 0;

#define PI         3.14159265358979323846

//const double angRange[3][2] =
//                        { {-25 , 35},
//                          {-35 , 30},
//                          {-40 , 60} };
const double angRange[3][2] =
                        { {-37 , 35},
                          {-37 , 35},
                          {-45 , 80} };
const double MaxAcc = 0.10;
const double MaxDcc = 0.10;
const double MaxVel = 0.20;


#define cPosToImp					 2000
//Coordination
#define CartesianCoor					0     
#define JointCoor						1		

//Return Value
#define RobotFailure				   -1
#define RobotSuccess					0
#define RobotBusy						1
#define RobotOutOfRange					2

//Axis mode
#define RbtAllAxis                      0
#define RbtSingleAxis                   1
