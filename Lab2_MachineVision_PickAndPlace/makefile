all:    kinematics dynamixel joint cartesian readwrite calibCamRobot robotControl utils image track
	g++ -o bin/jointControl lib/jointControl.o lib/Kinematics.o lib/DynamixelHandler.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/cartControl lib/cartControl.o lib/Kinematics.o lib/DynamixelHandler.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/calibrateCamRobotReferenceFrames lib/Utils.o lib/calibrateCamRobotReferenceFrames.o lib/ImageProcessing.o lib/RobotControl.o lib/ReadWriteFunctions.o lib/Kinematics.o  lib/DynamixelHandler.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt  -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/trackCube lib/Utils.o lib/trackCube.o lib/ImageProcessing.o lib/RobotControl.o lib/ReadWriteFunctions.o lib/Kinematics.o  lib/DynamixelHandler.o -L/usr/local/lib/ -ldxl_x64_cpp -lrt  -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	
dynamixel: src/DynamixelHandler.cpp
	g++ -c src/DynamixelHandler.cpp -o lib/DynamixelHandler.o -I./include -I/usr/local/include
	
kinematics: src/Kinematics.cpp
	g++ -c src/Kinematics.cpp -o lib/Kinematics.o -I./include -I/usr/include/opencv4

joint: src/jointControl.cpp
	g++ -c src/jointControl.cpp -o lib/jointControl.o -I./include -I/usr/include/opencv4
	
cartesian: src/cartControl.cpp
	g++ -c src/cartControl.cpp -o lib/cartControl.o -I./include -I/usr/include/opencv4
		
readwrite: src/ReadWriteFunctions.cpp
	g++ -c src/ReadWriteFunctions.cpp -o lib/ReadWriteFunctions.o -I./include -I/usr/include/opencv4

calibCamRobot: src/calibrateCamRobotReferenceFrames.cpp
	g++ -c src/calibrateCamRobotReferenceFrames.cpp -o lib/calibrateCamRobotReferenceFrames.o -I./include -I/usr/include/opencv4
	
robotControl: src/RobotControl.cpp
	g++ -c src/RobotControl.cpp -o lib/RobotControl.o -I./include -I/usr/include/opencv4
	
utils: src/Utils.cpp
	g++ -c src/Utils.cpp -o lib/Utils.o -I./include -I/usr/include/opencv4
	
image: src/ImageProcessing.cpp
	g++ -c src/ImageProcessing.cpp -o lib/ImageProcessing.o -I./include -I/usr/include/opencv4
	
track: src/trackCube.cpp
	g++ -c src/trackCube.cpp -o lib/trackCube.o -I./include -I/usr/include/opencv4
		
clean:
	rm lib/*.o
	rm bin/*
	