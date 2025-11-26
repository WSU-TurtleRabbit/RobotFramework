#include <iostream>
#include <libserial/serial_port.h>
#include <libserial/exceptions.h>

int main() {
  LibSerial::SerialPort serial_port;
  serial_port.Open("/dev/ttyUSB0");

  serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

  char kick = 'K';
  char dribble = 'D';
  char stop = 'S'
  char command;
  while true{
    std::cout<<"What would you like to send, the arduino"<<std::endl;
    std::cin >> command;
    if(command == "K")
    {
      serial_port.Write(&kick, 1);
    }
    else if (command == "D")
    {
    serial_port.Write(&dribble, 1);
    }
    else{
    serial_port.Write(&stop, 1);
    } 

  return 0;
}
}