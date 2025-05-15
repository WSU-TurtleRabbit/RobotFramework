CXX := g++
CXXFLAGS :=  -std=c++17 -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus -INetworks -IMath
LIBFLAGS := -L/usr/include -lbcm_host
PI3HAT_PATH := mjbots/pi3hat/pi3hat.o 
NETWORK_PATH := Networks/UDP.o Networks/decode.o
MATH_PATH := Math/wheel_velocity.o

RUNFILE1 := MultiMotorRun
RUNFILE2 := StopMotor
RUNFILEMAIN := MotorRun

detect_orange_ball: detect_orange_ball.cpp
	g++ detect_orange_ball.cpp -o detect_orange_ball `pkg-config --cflags --libs opencv4`

Networks/UDP.o: Networks/UDP.cpp
	$(CXX) -c Networks/UDP.cpp $(CXXFLAGS) -o Networks/UDP.o

Networks/decode.o: Networks/decode.cpp 
	$(CXX) -c Networks/decode.cpp $(CXXFLAGS) -o Networks/decode.o

MultiMotor.o: MultiMotor.cpp
	$(CXX) -c MultiMotor.cpp $(CXXFLAGS) -o MultiMotor.o 

StopMotor.o: StopMotor.cpp
	$(CXX) -c StopMotor.cpp $(CXXFLAGS) -o StopMotor.o 

MotorRun.o: MotorRun.cpp
	$(CXX) -c MotorRun.cpp $(CXXFLAGS) -o MotorRun.o 

$(RUNFILE1): $(PI3HAT_PATH) MultiMotor.o
	$(CXX) MultiMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o $(RUNFILE1)

$(RUNFILEMAIN): $(PI3HAT_PATH) MotorRun.o $(NETWORK_PATH) $(MATH_PATH)
	$(CXX) MotorRun.o $(PI3HAT_PATH) $(LIBFLAGS) $(NETWORK_PATH) $(MATH_PATH) -o $(RUNFILEMAIN)

$(RUNFILE2): $(PI3HAT_PATH) StopMotor.o
	$(CXX) StopMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o $(RUNFILE2)



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

build: pi3hat $(RUNFILE2) $(RUNFILEMAIN)
	make pi3hat
	make math
	make $(RUNFILEMAIN)
	make $(RUNFILE2)
remove:
	rm $(RUNFILE1) $(RUNFILE2) $(RUNFILEMAIN)
clean:
	rm -f *.o mjbots/pi3hat/*.o Math/*.o Networks/*.o program
