CXX := g++
CXXFLAGS :=  -std=c++17 -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus -INetworks -IMath -IBallDetection
LIBFLAGS := -L/usr/include -lbcm_host `pkg-config --cflags --libs opencv4`
PI3HAT_PATH := mjbots/pi3hat/pi3hat.o 
NETWORK_PATH := objectFiles/UDP.o objectFiles/decode.o
MATH_PATH := Math/wheel_velocity.o
DETECT_BALL_PATH := objectFiles/detect_ball.o


RUNFILE1 := MultiMotorRun
RUNFILE2 := StopMotor
RUNFILEMAIN := RobotFramework


objectFiles/detect_ball.o: BallDetection/detect_ball.cpp
	$(CXX) -c BallDetection/detect_ball.cpp $(CXXFLAGS) $(LIBFLAGS) -o objectFiles/detect_ball.o 

objectFiles/UDP.o: Networks/UDP.cpp
	$(CXX) -c Networks/UDP.cpp $(CXXFLAGS) -o objectFiles/UDP.o

objectFiles/decode.o: Networks/decode.cpp 
	$(CXX) -c Networks/decode.cpp $(CXXFLAGS) -o objectFiles/decode.o

objectFiles/MultiMotor.o: MultiMotor.cpp
	$(CXX) -c MultiMotor.cpp $(CXXFLAGS) -o objectFiles/MultiMotor.o 

objectFiles/StopMotor.o: StopMotor.cpp
	$(CXX) -c StopMotor.cpp $(CXXFLAGS) -o objectFiles/StopMotor.o 

objectFiles/RobotFramework.o: RobotFramework.cpp
	$(CXX) -c RobotFramework.cpp $(CXXFLAGS) $(LIBFLAGS)  -o objectFiles/RobotFramework.o 

$(RUNFILE1): $(PI3HAT_PATH) MultiMotor.o
	$(CXX) MultiMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o $(RUNFILE1)

$(RUNFILEMAIN): $(PI3HAT_PATH) objectFiles/RobotFramework.o $(NETWORK_PATH) $(MATH_PATH) $(DETECT_BALL_PATH)
	$(CXX) objectFiles/RobotFramework.o $(PI3HAT_PATH) $(DETECT_BALL_PATH) $(NETWORK_PATH) $(MATH_PATH) $(LIBFLAGS) -o $(RUNFILEMAIN) 

$(RUNFILE2): $(PI3HAT_PATH) objectFiles/StopMotor.o
	$(CXX) objectFiles/StopMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o $(RUNFILE2)



run:$(RUNFILE1)
	sudo ./$(RUNFILEMAIN)

stop:$(RUNFILE2)
	
	sudo ./$(RUNFILE2)

pi3hat: 
	(cd mjbots/pi3hat && make)

network:
	(cd Networks/ && make network)

math:
	(cd Math/ && make)

BallDetection: 
	(cd BallDetection/ && make)

build: pi3hat $(RUNFILE2) $(RUNFILEMAIN)
	make pi3hat
	make math
	make BallDetection
	make $(RUNFILEMAIN)
	make $(RUNFILE2)
remove:
	rm $(RUNFILE1) $(RUNFILE2) $(RUNFILEMAIN)
clean:
	rm -f *.o mjbots/pi3hat/*.o Math/*.o Networks/*.o objectFiles/*.o program
