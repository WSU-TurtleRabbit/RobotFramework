#include <iostream>
#include <vector>
#include "Telemetry.h"

namespace std
{

int main()
{
    Telemetry t = new Telemetry();

    cout << "Gathering data for temperature from each motor...\n";
    vector<double> temperature = t.queryTemperature();
    cout << "The temperatures are:\n";
    for (auto i : temperature)
    {
        cout << i << endl;
    }

    cout << "Gathering data for voltage from each motor...\n";
    vector<double> voltage = t.queryVoltage();
    cout << "The voltages are:\n";
    for (auto i : voltage)
    {
        cout << i << endl;
    }

    cout << "Gathering data for velocity from motor with CAN ID 1...\n";
    double velocity = t.queryVelocity(1);
    cout << "The velocity of motor with ID 1 is: " << velocity << endl;
}

}