#include "moteus.h"
#include "pi3hat_moteus_transport.h"

class Telemetry {
public:
    // Assuming the motor CAN IDs are 1,2,3 and 4
    std::vector<int> motorControllers;

    Telemetry();
    vector<double> queryTemperature();
    vector<double> queryVoltage();
    double queryVelocity(int motorController);

};
