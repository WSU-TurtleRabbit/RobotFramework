#include <iostream>
#include <libserial/serial_port.h>
#include <libserial/exceptions.h>

int main() {
  try {
    serial_port sp("/dev/ttyACM0"); // Replace with your Arduino's port
    sp.set_baudrate(9600); // Or your desired baud rate
    sp.open();

    // ... read and write data to/from the serial port ...
    sp.close();
  } catch (const serial_exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}