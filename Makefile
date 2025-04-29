run: motor.o mjbots/pi3hat/pi3hat.o 
	g++ motor.o mjbots/pi3hat/pi3hat.o -Imjbots/pi3hat -Imjbots/moteus -L/usr/include -lbcm_host -o run

run2: Simple.o mjbots/pi3hat/pi3hat.o 
	g++ Simple.o mjbots/pi3hat/pi3hat.o -Imjbots/pi3hat -Imjbots/moteus -L/usr/include -lbcm_host -o run2

Simple.o: Simple.cpp
	g++ -c Simple.cpp -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus -o Simple.o 

motor.o: motor.cpp
	g++ -c motor.cpp -Wall -Werror -Imjbots/pi3hat -Imjbots/moteus -o motor.o 

clean:
	rm -f *.o program

