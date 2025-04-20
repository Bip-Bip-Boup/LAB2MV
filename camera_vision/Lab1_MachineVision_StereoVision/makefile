#LINKING
all: calibMono calibStereo calibDisparity calibObjColor trackObjPosition readwrite image movie3D
	g++ lib/calibrateMonoCamera.o lib/ReadWriteFunctions.o -o bin/calibrateMonoCamera -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/calibrateStereoCamera lib/calibrateStereoCamera.o lib/ReadWriteFunctions.o -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/calibrateDisparity lib/calibrateDisparity.o lib/ReadWriteFunctions.o -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/calibrateObjectColor lib/calibrateObjectColor.o lib/ReadWriteFunctions.o -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/trackObjectPosition lib/trackObjectPosition.o lib/ReadWriteFunctions.o lib/ImageProcessing.o -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
	g++ -o bin/launch3DMovie lib/launch3DMovie.o -L/usr/lib/x86_64-linux-gnu `pkg-config --libs opencv4`
		
#COMPILATION
calibMono: src/calibrateMonoCamera.cpp
	g++ -c src/calibrateMonoCamera.cpp -o lib/calibrateMonoCamera.o -I./include -I/usr/include/opencv4
	
calibStereo: src/calibrateStereoCamera.cpp
	g++ -c src/calibrateStereoCamera.cpp -o lib/calibrateStereoCamera.o -I./include -I/usr/include/opencv4
	
calibDisparity: src/calibrateDisparity.cpp
	g++ -c src/calibrateDisparity.cpp -o lib/calibrateDisparity.o -I./include -I/usr/include/opencv4
	
calibObjColor: src/calibrateObjectColor.cpp
	g++ -c src/calibrateObjectColor.cpp -o lib/calibrateObjectColor.o -I./include -I/usr/include/opencv4
	
trackObjPosition: src/trackObjectPosition.cpp
	g++ -c src/trackObjectPosition.cpp -o lib/trackObjectPosition.o -I./include -I/usr/include/opencv4
	
readwrite: src/ReadWriteFunctions.cpp
	g++ -c src/ReadWriteFunctions.cpp -o lib/ReadWriteFunctions.o -I./include -I/usr/include/opencv4
	
image: src/ImageProcessing.cpp
	g++ -c src/ImageProcessing.cpp -o lib/ImageProcessing.o -I./include -I/usr/include/opencv4

movie3D: src/launch3DMovie.cpp
	g++ -c src/launch3DMovie.cpp -o lib/launch3DMovie.o -I./include -I/usr/include/opencv4	

#CLEANING
clean:
	rm lib/*.o
	rm bin/*
	