UDP.o:UDP.cpp 
	g++ -c UDP.cpp -Wall -Werror -o UDP.o

decode.o:decode.cpp 
	g++ -c UDP.cpp -Wall -Werror -o decode.o

network: UDP.o decode.o
	make UDP.o
	make decode.o
clean:
	rm -f *.o program