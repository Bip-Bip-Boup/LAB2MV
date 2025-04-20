#include <chrono>
#include <thread>

#include "Kinematics.h"
#include "DynamixelHandler.h"

DynamixelHandler _oDxlHandler;
std::string _robotDxlPortName = "/dev/ttyUSB0";
float _robotDxlProtocol = 2.0;
int _robotDxlBaudRate = 1000000;


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


int main(int argc, char** argv)
{
	if (argc == 7)
	{
		// Retrieves the args and stores them into variables
		float L1 = atof(argv[1]); // in cm
		float L2 = atof(argv[2]); // in cm
		float L3 = atof(argv[3]); // in cm
		float x = atof(argv[4]); // in cm
		float y = atof(argv[5]); // in cm
		float z = atof(argv[6]); // in cm
		float qgripper = deg2rad(0); // in rad
			
		// Initializes the robot
		initRobot(_oDxlHandler, _robotDxlPortName, _robotDxlProtocol, _robotDxlBaudRate);
		
		// Computes IK
		std::vector<float> qi = computeInverseKinematics(x, y, z, L1, L2, L3);
		
		// Computes FK
		computeForwardKinematics(qi[1], qi[2], qi[3], atof(argv[1]), atof(argv[2]), atof(argv[3]));
		// Computes Differential Kinematics
		std::vector<float> vJacobianMatrix = computeDifferentialKinematics(qi[1], qi[2], qi[3], L1, L2, L3);
		int rank =  computeJacobianMatrixRank(vJacobianMatrix, 0.1);
		
		// Sends the target joint values received only if there is at least a solution
		if (qi.size() >= 3)
		{
			std::cout << "IK has " << qi[0] << " solutions" << std::endl;
			std::cout << "qi =  (" << qi[1] << ", " << qi[2] << ", " << qi[3] << ")" << std::endl;
			std::vector<float> vTargetJointPosition;
				vTargetJointPosition.push_back(qi[4]); 
				vTargetJointPosition.push_back(qi[5]); 
				vTargetJointPosition.push_back(qi[6]); 
				vTargetJointPosition.push_back(qgripper); 
			_oDxlHandler.sendTargetJointPosition(vTargetJointPosition);
		}
		
		// Waits 2s
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		// Closes robot connection
		_oDxlHandler.closePort();
		
	}
	else
	{
		std::cout << "[WARNING] Cartesian control" << std::endl;
		std::cout << ">> cartControl L1(cm) L2(cm) L3(cm) x(cm) y(cm) z(cm)"<< std::endl;
		std::cout << ">> cartControl 5.5 7 12 0 0 24.5"<< std::endl;
	}
		
	return 0;
}