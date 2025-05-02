CXX := g++
CXXFLAGS :=  -std=c++17 -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus
LIBFLAGS := -L/usr/include -lbcm_host
PI3HAT_PATH := mjbots/pi3hat/pi3hat.o 

MultiMotor.o: MultiMotor.cpp
	$(CXX) -c MultiMotor.cpp $(CXXFLAGS) -o MultiMotor.o 

StopMotor.o: StopMotor.cpp
	$(CXX) -c StopMotor.cpp $(CXXFLAGS) -o StopMotor.o 

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(PI3HAT_PATH) MultiMotor.o
	$(CXX) MultiMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o run

motor_stopper: $(PI3HAT_PATH) StopMotor.o
	$(CXX) StopMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o motor_stopper

pi3hat: 
	(cd mjbots/pi3hat && make)

build: pi3hat run
	make pi3hat
	make run
	make motor_stopper 

clean:
	rm -f *.o mjbots/pi3hat/*.o program