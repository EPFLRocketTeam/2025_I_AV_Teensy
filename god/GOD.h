#ifndef GOD_H
#define GOD_H
#include <iostream>
#include <vector>
#include <string>
struct GOD // Global Object Dictionary
{
    // Control
    std::vector<double> control_memory; // [d1, d2, thrust, mz]
    // Navigation
    std::vector<double> current_state_memory; // [temperature, altitude, ax, ay, az, gx, gy, gz]
    // Constructor
    GOD();
    ~GOD() = default;
    // Log data
};
#endif // GOD_H