#ifndef PLANT_H
#define PLANT_H

#include "init.h"
#include <array>


// Process
std::array<double, N> plant_process(const std::array<double, N>& x, 
                                    double u, 
                                    const std::array<double, N>& process_noise, 
                                    double Rw);

// Measurment
double plant_measurement(const std::array<double, N>& x, 
                         double u, 
                         const std::array<double, 2>& measurement_noise, 
                         double Rv);

#endif 