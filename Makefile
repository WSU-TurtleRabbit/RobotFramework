CXX := g++
CXXFLAGS :=  -std=c++17 -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus
LIBFLAGS := -L/usr/include -lbcm_host
PI3HAT_PATH := mjbots/pi3hat/pi3hat.o 

# run: motor.o mjbots/pi3hat/pi3hat.o 
# 	g++ motor.o g++mjbots/pi3hat/pi3hat.o -Imjbots/pi3hat -Imjbots/moteus -L/usr/include -lbcm_host -o run

# run2: Simple.o mjbots/pi3hat/pi3hat.o 
# 	g++ Simple.o mjbots/pi3hat/pi3hat.o -Imjbots/pi3hat -Imjbots/moteus -L/usr/include -lbcm_host -o run2

# Simple.o: Simple.cpp
# 	g++ -c Simple.cpp -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus -o Simple.o 

multiple_cycle.o: multiple_cycle.cpp
	$(CXX) -c multiple_cycle.cpp $(CXXFLAGS) -o multiple_cycle.o 

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(PI3HAT_PATH) multiple_cycle.o
	$(CXX) multiple_cycle.o $(PI3HAT_PATH) $(LIBFLAGS) -o run
pi3hat: 
	(cd mjbots/pi3hat && make)
clean:
	rm -f *.o program