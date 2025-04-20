#include "RobotControl.h"

#define GRIPPER_OPEN -60.0
#define GRIPPER_CLOSE 20.0

void initRobot(DynamixelHandler& dxlHandler, std::string portName, float protocol, int baudRate)
{
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	dxlHandler.setDeviceName(portName);
	dxlHandler.setProtocolVersion(protocol);
	dxlHandler.openPort();
	dxlHandler.setBaudRate(baudRate);
	dxlHandler.enableTorque(true);
	std::cout << std::endl;
}

void closeRobot(DynamixelHandler& dxlHandler)
{
	dxlHandler.enableTorque(false);
	dxlHandler.closePort();
}

void moveJoint(DynamixelHandler& dxlHandler, float q1, float q2, float q3, float qgripper)
{
	std::vector<float> vTargetJointPosition;
		vTargetJointPosition.push_back(q1); 
		vTargetJointPosition.push_back(q2); 
		vTargetJointPosition.push_back(q3); 
		vTargetJointPosition.push_back(qgripper); 
	dxlHandler.sendTargetJointPosition(vTargetJointPosition);
}

void openGripper(DynamixelHandler& dxlHandler)
{
	dxlHandler.controlGripper(deg2rad(GRIPPER_OPEN));
}

void closeGripper(DynamixelHandler& dxlHandler)
{
	dxlHandler.controlGripper(deg2rad(GRIPPER_CLOSE));
}