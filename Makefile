run: motor.o moteus_pi3hat.o  
	g++ motor.o mjbots/pi3hat/moteus_pi3hat.o -o run

motor.o: motor.cpp
	g++ -c -Imjbots/pi3hat -Imjbots/moteus -Wall -Werror -o motor.o 

clean:
    rm -f *.o program

