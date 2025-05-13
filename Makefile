CXX := g++
CXXFLAGS :=  -std=c++17 -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus
LIBFLAGS := -L/usr/include -lbcm_host
PI3HAT_PATH := mjbots/pi3hat/pi3hat.o 


RUNFILE1 := MultiMotorRun
RUNFILE2 := StopMotor
RUNFILEMAIN := MotorRun



MultiMotor.o: MultiMotor.cpp
	$(CXX) -c MultiMotor.cpp $(CXXFLAGS) -o MultiMotor.o 

StopMotor.o: StopMotor.cpp
	$(CXX) -c StopMotor.cpp $(CXXFLAGS) -o StopMotor.o 

MotorRun.o: MotorRun.cpp
	$(CXX) -c MotorRun.cpp $(CXXFLAGS) -o MotorRun.o 

$(RUNFILE1): $(PI3HAT_PATH) MultiMotor.o
	$(CXX) MultiMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o $(RUNFILE1)

$(RUNFILEMAIN): $(PI3HAT_PATH) MultiMotor.o
	$(CXX) MultiMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o $(RUNFILEMAIN)

$(RUNFILE2): $(PI3HAT_PATH) StopMotor.o
	$(CXX) StopMotor.o $(PI3HAT_PATH) $(LIBFLAGS) -o $(RUNFILE2)



run:$(RUNFILE1)
	sudo ./$(RUNFILE1)

stop:$(RUNFILE2)
	
	sudo ./$(RUNFILE2)

pi3hat: 
	(cd mjbots/pi3hat && make)

build: pi3hat $(RUNFILE1) $(RUNFILE2)
	make pi3hat
	make $(RUNFILE1)
	make $(RUNFILE2)
remove:
	rm $(RUNFILE1) $(RUNFILE2)
clean:
	rm -f *.o mjbots/pi3hat/*.o program