CXX := g++
CXXFLAGS :=  -std=c++17 -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus
LIBFLAGS := -L/usr/include -lbcm_host
PI3HAT_PATH := mjbots/pi3hat/pi3hat.o 

multiple_cycle.o: multiple_cycle.cpp
	$(CXX) -c multiple_cycle.cpp $(CXXFLAGS) -o multiple_cycle.o 

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(PI3HAT_PATH) multiple_cycle.o
	$(CXX) multiple_cycle.o $(PI3HAT_PATH) $(LIBFLAGS) -o run
pi3hat: 
	(cd mjbots/pi3hat && make)

build: pi3hat run
	make pi3hat
	make run
	make clean

clean:
	rm -f *.o mjbots/pi3hat/*.o program

Hello: