#ifndef GOD_H
#define GOD_H

#include <iostream>
#include <vector>
#include <string>

struct GOD // Global Object Dictionary
{
    // Control
    std::vector<double> control_output_memory; // [d1, d2, thrust, mz]

    // Navigation
    std::vector<double> navigation_output_memory; // [altitude, ax, ay, az, gx, gy, gz]

    // Constructor
    GOD() {
        control_output_memory = {0, 0, 0, 0};
        navigation_output_memory = {0, 0, 0, 0, 0, 0, 0};
    }
    ~GOD() = default;
};

#endif // GOD_H
