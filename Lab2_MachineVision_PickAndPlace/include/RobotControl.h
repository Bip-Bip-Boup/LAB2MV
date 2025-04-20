#ifndef _ROBOT_CONTROL_
#define _ROBOT_CONTROL_

#include "DynamixelHandler.h"
#include "Kinematics.h"

void initRobot(DynamixelHandler& dxlHandler, std::string portName, float protocol, int baudRate);
void closeRobot(DynamixelHandler& dxlHandler);
void moveJoint(DynamixelHandler& dxlHandler, float q1, float q2, float q3, float qgripper);
void openGripper(DynamixelHandler& dxlHandler);
void closeGripper(DynamixelHandler& dxlHandler);


#endif