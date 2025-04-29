# moteus_pi3hat.o: pi3hat.o pi3hat_tool.o
# 	g++ pi3hat.o pi3hat_tool.o -Wall -Werror -L/usr/include -lbcm_host -o moteus_pi3hat.o

pi3hat.o: pi3hat.cc
	g++ -c pi3hat.cc -Wall -Werror -o pi3hat.o

pi3hat_tool.o: pi3hat_tool.cc
	g++ -c pi3hat_tool.cc -Wall -Werror -o pi3hat_tool.o

clean:
	rm -f *.o program