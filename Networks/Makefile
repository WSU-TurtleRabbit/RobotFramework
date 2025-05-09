Reciever: Socket_reciever.cpp 
	g++ Socket_reciever.cpp -o Reciever

Sender: Socket_sender.cpp
	g++ Socket_sender.cpp -o Sender

UDP: UDPReciver.cpp
	g++ UDPReciver.cpp -o UDP

UDP_test: ../Math/Wheel_velocity.cpp Decoder.cpp
	g++ ../Math/Wheel_velocity.cpp Decoder.cpp UDPReciver.cpp -I../Math/ -o UDP_test 

clean:
	rm -f *.o program